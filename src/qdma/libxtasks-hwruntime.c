/*--------------------------------------------------------------------
  (C) Copyright 2017-2020 Barcelona Supercomputing Center
                          Centro Nacional de Supercomputacion

  This file is part of OmpSs@FPGA toolchain.

  This code is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; either version 3 of
  the License, or (at your option) any later version.

  OmpSs@FPGA toolchain is distributed in the hope that it will be
  useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this code. If not, see <www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "libxtasks.h"
#include "libxdma.h"
#include "libxdma_version.h"
#include "platform.h"
#include "util/common.h"
#include "util/ticket-lock.h"

#define CMD_IN_QUEUE_LEN 1024    ///< Total number of entries in the cmd_in_queue
#define CMD_IN_SUBQUEUE_LEN 64   ///< Number of entries in the sub-queue of cmd_in_queue for one accelerator
#define CMD_OUT_QUEUE_LEN 1024   ///< Total number of entries in the cmd_out_queue
#define CMD_OUT_SUBQUEUE_LEN 64  ///< Number of entries in the sub-queue of cmd_out_queue for one accelerator

#define MAX_NUM_ACC         (CMD_IN_QUEUE_LEN/CMD_IN_SUBQUEUE_LEN)
#define ACC_INFO_MAX_LEN    4096
#define STR_BUFFER_SIZE     128

#define QUEUE_VALID 0x80
#define QUEUE_RESERVED 0x40
#define QUEUE_INVALID 0x00
#define DEF_EXEC_TASK_SIZE 256  ///< Size of hw task when using the defult num. of args.
#define DEF_EXEC_TASK_ARGS_LEN \
    14  // NOTE: (DEF_EXEC_TASK_SIZE - sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
// NOTE: A task in extended mode MUST fit into the a sub-queue of cmd_in queue
#define EXT_HW_TASK_SIZE 512  ///< Size of hw task when using the extended num. of args.
#define EXT_HW_TASK_ARGS_LEN \
    30  // NOTE: (EXT_HW_TASK_SIZE -
        // sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
// NOTE: The value NUM_RUN_TASKS may be changed to increase the number of concurrent tasks submited into the
// accelerators
#define NUM_RUN_TASKS 1024  ///< Maximum number of concurrently running tasks
#define NEW_TASK_COPY_FLAGS_WORDOFFSET \
    0  ///< Offset of new_task_copy_t->flags field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_SIZE_WORDOFFSET \
    32  ///< Offset of new_task_copy_t->size field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_OFFSET_WORDOFFSET \
    0  ///< Offset of new_task_copy_t->offset field in the 3rd word forming new_task_copy_t
#define NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET \
    32  ///< Offset of new_task_copy_t->accessedLen field in the 3rd word forming new_task_copy_t

#define CMD_IN_QUEUE_ADDR 0x00004000
#define CMD_OUT_QUEUE_ADDR 0x00008000
#define HWRUNTIME_RESET_ADDRESS 0x0000C000
#define SPAWN_OUT_QUEUE_ADDRESS 0x00014000
#define SPAWN_IN_QUEUE_ADDRESS 0x000018000
#define BRAM_INFO_ADDRESS       0x00020000

#define HWCOUNTER_ADDRESS 0x00010000

#define PCI_DEV_BASE_PATH   "/sys/bus/pci/devices/"
#define PCI_DEV             "0000:02:00.0"
#define PCI_BAR_FILE        "resource2"
#define PCI_MAX_PATH_LEN    64
#define PCI_BAR_SIZE        0x40000     //map only 256KB even if the BAR is larger

typedef struct {
    char descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
    unsigned short volatile cmdInWrIdx;    ///< Writing index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile cmdInRdIdx;    ///< Reading index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile cmdInAvSlots;  ///< Counter for available slots in cmd_in sub-queue
    ticketLock_t cmdInLock;                ///< Lock for atomic operations over cmd_in sub-queue
    unsigned short volatile cmdOutIdx;     ///< Reading index of the accelerator sub-queue in the cmd_out queue
    unsigned short volatile cmdOutLock;    ///< Lock for atomic operations over cmd_out sub-queue
    unsigned short volatile instrIdx;      ///< Reading index of the accelerator instrumentation buffer
    unsigned short volatile instrLock;     ///< Lock for atomic operations over instrumentation buffers
} acc_t;
//
//! \brief Internal library task information
typedef struct {
    xtasks_task_id id;                 ///< External task identifier
    cmd_header_t *cmdHeader;           ///< Pointer to the cmd_header_t struct
    cmd_exec_task_arg_t *cmdExecArgs;  ///< Pointer to the array of cmd_exec_task_arg_t structs
    acc_t *accel;                      ///< Accelerator where the task will run
    uint8_t extSize : 1;               ///< Whether the space available in args is extended or not
    uint8_t periTask : 1;              ///< Whether the tasks is a periodic task or not
} task_t;

static int _pciBarFd;
static uint32_t *_pciBar;
static acc_t _accs[MAX_NUM_ACC];
static task_t *_tasks;
static int _numAccs;

static uint64_t     *_cmdInQueue;
static uint64_t     *_cmdOutQueue;
static uint32_t     *_hwruntimeRst;
static uint8_t      *_cmdExecTaskBuff;

//! \brief Platform and Backend strings
const char _platformName[] = "qdma";
const char _backendName[] = "hwruntime";

//forward declarations
static int getAccRawInfo(char *accInfo, const uint32_t *rawBitInfo);
static int initAccList(acc_t *accs, const char *accInfo);
static inline __attribute__((always_inline)) void resetHWRuntime(volatile uint32_t *resetReg);

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
#error Installed libxdma is not supported (use >= 3.0)
#endif

xtasks_stat xtasksInit() {

    xdma_status st;
    xtasks_stat ret = XTASKS_ERROR; //Initialize DMA
    st = xdmaOpen();
    if (st != XDMA_SUCCESS) {
        PRINT_ERROR("Could not initialize XDMA\n");
        return XTASKS_ERROR;
    }
    st = xdmaInitMem();
    if (st != XDMA_SUCCESS) {
        PRINT_ERROR("Could not initialize device memory\n");
        goto init_mem_err;

    }
    //Map PCI BAR into user space
    char pciBarPath[PCI_MAX_PATH_LEN];
    snprintf(pciBarPath, PCI_MAX_PATH_LEN, "%s/%s/%s",
            PCI_DEV_BASE_PATH, PCI_DEV, PCI_BAR_FILE);
    _pciBarFd = open(pciBarPath, O_RDWR);
    if (_pciBarFd < 0) {
        perror("XTASKS: Could not open PCIe register window");
        goto init_open_bar_err;

    }
    _pciBar = mmap(NULL, PCI_BAR_SIZE, PROT_READ|PROT_WRITE, MAP_PRIVATE, _pciBarFd, 0);
    if (_pciBar ==MAP_FAILED) {
        ret = XTASKS_ERROR;
        perror("XTASKS: Could not map BAR into process memory space");
        goto init_map_bar_err;
    }
    uint32_t *bitInfo = malloc(BITINFO_MAX_WORDS*sizeof(uint32_t));
    memcpy(bitInfo, _pciBar + BRAM_INFO_ADDRESS/sizeof(*_pciBar), BITINFO_MAX_WORDS*sizeof(uint32_t));

    // Check if bitstream is compatible
    //
    bit_compatibility_t compat = checkbitstreamCompatibility(bitInfo);
    if (compat == BIT_NO_COMPAT || compat == BIT_COMPAT_UNKNOWN) {
        printErrorBitstreamCompatibility();
        ret = XTASKS_ENOAV;
        goto init_compat_err;
    }

    // Check if bitstream has the hwruntime feature
    bit_feature_t feature = checkbitstreamFeature("hwruntime", bitInfo);
    if (feature == BIT_FEATURE_NO_AVAIL) {
        PRINT_ERROR("HW runtime not available in the loaded FPGA bitstream");
        ret = XTASKS_ENOAV;
        goto init_compat_err;
    }

    char * accInfo = malloc(ACC_INFO_MAX_LEN);
    getAccRawInfo(accInfo, bitInfo);
    _numAccs = initAccList(_accs, accInfo);

    //Initialize command queues
    _cmdInQueue = (uint64_t*)(_pciBar + (CMD_IN_QUEUE_ADDR/sizeof(*_pciBar)));
    memset(_cmdInQueue, 0, CMD_IN_QUEUE_LEN*sizeof(*_cmdInQueue));
    _cmdOutQueue = (uint64_t*)(_pciBar + (CMD_OUT_QUEUE_ADDR/sizeof(_cmdOutQueue)));
    memset(_cmdOutQueue, 0, CMD_OUT_QUEUE_LEN*sizeof(_cmdInQueue));
    //TODO: Implement remote queues

    //initialize reset
    _hwruntimeRst = (uint32_t*)(_pciBar + (HWRUNTIME_RESET_ADDRESS/sizeof(*_pciBar)));
    resetHWRuntime(_hwruntimeRst);


    // Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS * sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto init_alloc_tasks_err;
    }
    _cmdExecTaskBuff = (uint8_t *)malloc(NUM_RUN_TASKS * DEF_EXEC_TASK_SIZE);
    if (_cmdExecTaskBuff == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for exec. tasks buffer");
        goto init_alloc_exec_tasks_err;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].cmdHeader = (cmd_header_t *)&_cmdExecTaskBuff[idx * DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)0;
        _tasks[idx].extSize = 0;
        _tasks[idx].periTask = 0;
    }
    free(accInfo);
    free(bitInfo);

    return XTASKS_SUCCESS;

    //Error cleanup
init_alloc_exec_tasks_err:
    free(_tasks);
init_alloc_tasks_err:
init_compat_err:
    munmap(_pciBar, PCI_BAR_SIZE);
    free(bitInfo);
    free(accInfo);
init_map_bar_err:
    close(_pciBarFd);
init_open_bar_err:
    xdmaFiniMem();
init_mem_err:
    xdmaClose();
    return ret;
}

xtasks_stat xtasksFini() {

    free(_tasks);
    munmap(_pciBar, PCI_BAR_SIZE);
    close(_pciBarFd);

    xdmaFiniMem();
    xdmaClose();
    return XTASKS_SUCCESS;
}

static inline __attribute__((always_inline)) void resetHWRuntime(volatile uint32_t *resetReg)
{
    // Nudge one register
    *resetReg = 0x01;
    usleep(1);     //Wait for the reset to propagate
    *resetReg = 0x00;
}
static int initAccList(acc_t *accs, const char *accInfo) {

    unsigned long long int t;
    int retScanf, ret, numRead, numAccs;
    float freq;
    size_t num, total;
    total = 0;
    char buffer[STR_BUFFER_SIZE];
    //Parse acc information
    while ((retScanf = sscanf(accInfo, "%llu %zu %128s %f%n", &t, &num, buffer, &freq, &numRead)) == 4) {
        accInfo += numRead;  //Advance ponter
        total += num;
        for (size_t i = total - num; i < total; ++i) {
            _accs[i].info.id = i;
            _accs[i].info.type = t;
            _accs[i].info.freq = freq;
            _accs[i].info.maxTasks = -1;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
            _accs[i].cmdInWrIdx = 0;
            _accs[i].cmdInAvSlots = CMD_IN_SUBQUEUE_LEN;
            _accs[i].cmdInRdIdx = 0;
            _accs[i].cmdOutIdx = 0;
        }
    }
    ret = (total < numAccs) ? total : numAccs;

    if (retScanf != EOF && retScanf != 0) {
        // Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    } else if (numAccs > MAX_NUM_ACC) {
        ret = -1;
        PRINT_ERROR("The maximum number of accelerators supported by the library was reached");
    }
    return ret;
}

static int getAccRawInfo(char *accInfo, const uint32_t *rawBitInfo) {
    char *filePath;

    int size;
    filePath = getConfigFilePath();
    if (filePath) { //User config takes precedence over bitinfo data
        FILE *accFile = fopen(filePath, "r");
        size = fread(accInfo, BITINFO_MAX_WORDS*sizeof(*rawBitInfo), 1, accFile);
        free(filePath);
    } else {    //Read from bit info
        //Apply the offset directly as its returned in 32-bit words
        rawBitInfo += getBitinfoOffset(BITSTREAM_INFO_ACC_LIST, rawBitInfo);
        //Find out the size of the acc info field in
        int idx;
        for (idx=0; rawBitInfo[idx] != BITINFO_FIELD_SEP; idx++);
        size = idx * sizeof(*rawBitInfo);
        memcpy(accInfo, rawBitInfo, size);
        accInfo[size] = '\0';   //terminate the string
    }
    return size;

}

xtasks_stat xtasksGetPlatform(const char **name) {
    *name = _platformName;
    return XTASKS_SUCCESS;
}


xtasks_stat xtasksGetBackend(const char **name) {
    *name = _backendName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(size_t *count) {
    *count =  _numAccs;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccs(size_t const maxCount, xtasks_acc_handle *array, size_t *count)
{
    if (array == NULL || count == NULL) return XTASKS_EINVAL;

    size_t tmp = maxCount > _numAccs ? _numAccs : maxCount;
    for (size_t i = 0; i < tmp; ++i) {
        array[i] = (xtasks_acc_handle)(&_accs[i]);
    }
    *count = tmp;

    return XTASKS_SUCCESS;
}
xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info *info)

{
    if (info == NULL) return XTASKS_EINVAL;

    acc_t *ptr = (acc_t *)handle;
    *info = ptr->info;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId, xtasks_task_id const parent,
    xtasks_comp_flags const compute, xtasks_task_handle *handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps, unsigned int const period,
    xtasks_task_handle *handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle *handle) { return XTASKS_ENOSYS; }

xtasks_stat xtasksAddArg(
    xtasks_arg_id const id, xtasks_arg_flags const flags, xtasks_arg_val const value, xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksAddArgs(
    size_t const num, xtasks_arg_flags const flags, xtasks_arg_val *const values, xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle) { return XTASKS_ENOSYS; }

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle) { return XTASKS_ENOSYS; }

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle *handle, xtasks_task_id *id) { return XTASKS_ENOSYS; }

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel, xtasks_task_handle *task, xtasks_task_id *id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const handle, xtasks_ins_event *events, size_t const maxCount)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask **task) { return XTASKS_ENOSYS; }

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id) { return XTASKS_ENOSYS; }
