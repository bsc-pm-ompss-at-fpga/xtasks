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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "libxdma.h"
#include "libxdma_version.h"
#include "libxtasks.h"
#include "platform.h"
#include "util/common.h"
#include "util/ticket-lock.h"

#define CMD_IN_QUEUE_LEN 1024    ///< Total number of entries in the cmd_in_queue
#define CMD_IN_SUBQUEUE_LEN 64   ///< Number of entries in the sub-queue of cmd_in_queue for one accelerator
#define CMD_OUT_QUEUE_LEN 1024   ///< Total number of entries in the cmd_out_queue
#define CMD_OUT_SUBQUEUE_LEN 64  ///< Number of entries in the sub-queue of cmd_out_queue for one accelerator
#define SPWN_OUT_QUEUE_LEN 1024  ///< Total number of entries in the spawn_out queue (tasks created inside the FPGA)
#define SPWN_IN_QUEUE_LEN 1024   ///< Total number of entries in the spawn_in queue

#define MAX_NUM_ACC (CMD_IN_QUEUE_LEN / CMD_IN_SUBQUEUE_LEN)
#define ACC_INFO_MAX_LEN 4096
#define STR_BUFFER_SIZE 128

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
#define BRAM_INFO_ADDRESS 0x00020000

#define HWCOUNTER_ADDRESS 0x00010000

#define PCI_DEV_BASE_PATH "/sys/bus/pci/devices/"
#define PCI_DEV "0000:02:00.0"
#define PCI_BAR_FILE "resource2"
#define PCI_MAX_PATH_LEN 64
#define PCI_BAR_SIZE 0x40000  // map only 256KB even if the BAR is larger

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

static uint64_t *_cmdInQueue;
static uint64_t *_cmdOutQueue;
static uint64_t *_spawnOutQueue;
static size_t _spawnOutQueueIdx;
static uint64_t *_spawnInQueue;
static size_t _spawnInQueueIdx;
static uint32_t *_hwruntimeRst;
static uint8_t *_cmdExecTaskBuff;

static bit_feature_t _instrAvail;
static size_t _numInstrEvents;
static xtasks_ins_event *_instrBuffPhy;
static xtasks_ins_event *_instrBuff;
static xdma_buf_handle _instrBuffHandle;
static uint64_t *_instrCounter;

//! \brief Platform and Backend strings
const char _platformName[] = "qdma";
const char _backendName[] = "hwruntime";

// forward declarations
static int getAccRawInfo(char *accInfo, const uint32_t *rawBitInfo);
static int initAccList(acc_t *accs, const char *accInfo);
static inline __attribute__((always_inline)) void resetHWRuntime(volatile uint32_t *resetReg);
static int getFreeTaskEntry();
static void initializeTask(
    task_t *task, const xtasks_task_id id, acc_t *accel, xtasks_task_id const parent, xtasks_comp_flags const compute);
static xtasks_stat setExtendedModeTask(task_t *task);
static xtasks_stat submitCommand(acc_t *acc, uint64_t *command, const size_t length, uint64_t *queue);
static int getAccEvents(acc_t *acc, xtasks_ins_event *events, size_t count, xtasks_ins_event *accBuffer);
static void getNewTaskFromQ(xtasks_newtask **task, uint64_t *spawnQueue, int idx);

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
#error Installed libxdma is not supported (use >= 3.0)
#endif

xtasks_stat xtasksInit()
{
    xdma_status st;
    xtasks_stat ret = XTASKS_ERROR;  // Initialize DMA
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
    // Map PCI BAR into user space
    char pciBarPath[PCI_MAX_PATH_LEN];
    snprintf(pciBarPath, PCI_MAX_PATH_LEN, "%s/%s/%s", PCI_DEV_BASE_PATH, PCI_DEV, PCI_BAR_FILE);
    _pciBarFd = open(pciBarPath, O_RDWR);
    if (_pciBarFd < 0) {
        perror("XTASKS: Could not open PCIe register window");
        goto init_open_bar_err;
    }
    _pciBar = mmap(NULL, PCI_BAR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _pciBarFd, 0);
    if (_pciBar == MAP_FAILED) {
        ret = XTASKS_ERROR;
        perror("XTASKS: Could not map BAR into process memory space");
        goto init_map_bar_err;
    }
    uint32_t *bitInfo = malloc(BITINFO_MAX_WORDS * sizeof(uint32_t));
    memcpy(bitInfo, _pciBar + BRAM_INFO_ADDRESS / sizeof(*_pciBar), BITINFO_MAX_WORDS * sizeof(uint32_t));

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

    char *accInfo = malloc(ACC_INFO_MAX_LEN);
    getAccRawInfo(accInfo, bitInfo);
    _numAccs = initAccList(_accs, accInfo);

    // Initialize command queues
    _cmdInQueue = (uint64_t *)(_pciBar + (CMD_IN_QUEUE_ADDR / sizeof(*_pciBar)));
    memset(_cmdInQueue, 0, CMD_IN_QUEUE_LEN * sizeof(*_cmdInQueue));
    _cmdOutQueue = (uint64_t *)(_pciBar + (CMD_OUT_QUEUE_ADDR / sizeof(*_pciBar)));
    memset(_cmdOutQueue, 0, CMD_OUT_QUEUE_LEN * sizeof(*_cmdOutQueue));
    // TODO: Implement remote queues

    // initialize reset
    _hwruntimeRst = (uint32_t *)(_pciBar + (HWRUNTIME_RESET_ADDRESS / sizeof(*_pciBar)));
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

    feature = checkbitstreamFeature("hwruntime_ext", bitInfo);
    if (feature == BIT_FEATURE_NO_AVAIL) {
        _spawnOutQueue = NULL;
        _spawnInQueue = NULL;
    } else {
        _spawnOutQueue = (uint64_t *)(_pciBar + SPAWN_OUT_QUEUE_ADDRESS / sizeof(*_pciBar));
        _spawnInQueue = (uint64_t *)(_pciBar + SPAWN_IN_QUEUE_ADDRESS / sizeof(*_pciBar));
        _spawnInQueueIdx = 0;
        _spawnOutQueueIdx = 0;

        // Invalidate entries
        for (int i = 0; i < SPWN_OUT_QUEUE_LEN; i++) {
            _spawnOutQueue[i] = 0;
        }
        for (int i = 0; i < SPWN_IN_QUEUE_LEN; i++) {
            _spawnInQueue[i] = 0;
        }
    }

    // Check for instrumentation
    _instrAvail = checkbitstreamFeature("hwcounter", bitInfo);

    free(accInfo);
    free(bitInfo);

    return XTASKS_SUCCESS;

    // Error cleanup
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

xtasks_stat xtasksInitHWIns(const size_t nEvents)
{
    xtasks_stat ret;
    size_t insBufferSize;

    if (nEvents < 1) {
        return XTASKS_EINVAL;
    }

    if (_instrAvail == BIT_FEATURE_NO_AVAIL || _instrAvail == BIT_FEATURE_UNKNOWN) {
        return XTASKS_ENOAV;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents * _numAccs * sizeof(xtasks_ins_event);

    xdma_status s = xdmaAllocate(&_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ENOMEM;
    }
    unsigned long phyAddr;
    xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    // Invalidate all entries
    _instrBuff = (xtasks_ins_event *)malloc(insBufferSize);
    for (int i = 0; i < _numInstrEvents * _numAccs; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }
    s = xdmaMemcpy(_instrBuff, _instrBuffHandle, insBufferSize, 0, XDMA_TO_DEVICE);

    if (s != XDMA_SUCCESS) {
        PRINT_ERROR("Could not initialize instrumentation buffer");
        ret = XTASKS_ERROR;
        goto hwins_buff_init_err;
    }

    // Send instr setup command to acc
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    memcpy((void *)cmd.header.commandArgs, (void *)&_numInstrEvents, CMD_SETUP_INS_ARGS_NUMEVS_BYTES);
    for (size_t i = 0; i < _numAccs; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents * i);

        xtasks_stat ret =
            submitCommand(_accs + i, (uint64_t *)&cmd, sizeof(cmd_setup_hw_ins_t) / sizeof(uint64_t), _cmdInQueue);
        if (ret != XTASKS_SUCCESS) {
            PRINT_ERROR("Error setting up instrumentation");
            goto hwins_cmd_err;
        }

        _accs[i].instrIdx = 0;
        _accs[i].instrLock = 0;
    }

    // Init HW counter
    if (_pciBar == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("xtasks has not been initialized");
        goto hwins_no_pcibar;
    }
    _instrCounter = (uint64_t *)(_pciBar + HWCOUNTER_ADDRESS / sizeof(*_pciBar));

    return XTASKS_SUCCESS;

hwins_no_pcibar:
hwins_cmd_err:
hwins_buff_init_err:
    free(_instrBuff);
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuffPhy = NULL;
    _instrBuff = NULL;
    return ret;
}

xtasks_stat xtasksFiniHWIns()
{
    if (_instrBuff == NULL || _numInstrEvents == 0) return XTASKS_SUCCESS;  // Instrumentation is not initialized
    free(_instrBuff);
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuffPhy = NULL;
    _instrBuff = NULL;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksFini()
{
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
    usleep(1);  // Wait for the reset to propagate
    *resetReg = 0x00;
}
static int initAccList(acc_t *accs, const char *accInfo)
{
    unsigned long long int t;
    int retScanf, ret, numRead, numAccs;
    float freq;
    size_t num, total;
    total = 0;
    char buffer[STR_BUFFER_SIZE];
    // Parse acc information
    // discard first line containing headers
    accInfo = index(accInfo, '\n') + 1;  // set pointer to next character after \n
    while ((retScanf = sscanf(accInfo, "%llu %zu %128s %f%n", &t, &num, buffer, &freq, &numRead)) == 4) {
        accInfo += numRead + (*(accInfo + numRead) == '\n' ? 1 : 0);  // Advance ponter
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
    ret = total;

    if (retScanf != EOF && retScanf != 0) {
        // Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    } else if (numAccs > MAX_NUM_ACC) {
        ret = -1;
        PRINT_ERROR("The maximum number of accelerators supported by the library was reached");
    }
    return ret;
}

static int getAccRawInfo(char *accInfo, const uint32_t *rawBitInfo)
{
    char *filePath;

    int size;
    filePath = getConfigFilePath();
    if (filePath) {  // User config takes precedence over bitinfo data
        FILE *accFile = fopen(filePath, "r");
        size = fread(accInfo, 1, BITINFO_MAX_WORDS * sizeof(*rawBitInfo), accFile);
        free(filePath);
    } else {  // Read from bit info
        // Apply the offset directly as its returned in 32-bit words
        rawBitInfo += getBitinfoOffset(BITSTREAM_INFO_ACC_LIST, rawBitInfo);
        // Find out the size of the acc info field in
        int idx;
        for (idx = 0; rawBitInfo[idx] != BITINFO_FIELD_SEP; idx++)
            ;
        size = idx * sizeof(*rawBitInfo);
        memcpy(accInfo, rawBitInfo, size);
        accInfo[size] = '\0';  // terminate the string
    }
    return size;
}

xtasks_stat xtasksGetPlatform(const char **name)
{
    *name = _platformName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetBackend(const char **name)
{
    *name = _backendName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(size_t *count)
{
    *count = _numAccs;
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
    acc_t *accel = (acc_t *)accId;
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOENTRY;
    }

    initializeTask(&_tasks[idx], id, accel, parent, compute);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

static int getFreeTaskEntry()
{
    for (int i = 0; i < NUM_RUN_TASKS; ++i) {
        if (_tasks[i].id == 0) {
            if (__sync_bool_compare_and_swap(&_tasks[i].id, 0, 1)) {
                return i;
            }
        }
    }
    return -1;
}

static void initializeTask(
    task_t *task, const xtasks_task_id id, acc_t *accel, xtasks_task_id const parent, xtasks_comp_flags const compute)
{
    task->id = id;
    task->accel = accel;
    task->periTask = 0;
    cmd_exec_task_header_t *cmdHeader = (cmd_exec_task_header_t *)task->cmdHeader;
    task->cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_EXEC_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_TM;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(task);
}

static void initializePeriodicTask(task_t *task, const xtasks_task_id id, acc_t *accel, xtasks_task_id const parent,
    xtasks_comp_flags const compute, int numReps, int period)
{
    task->id = id;
    task->accel = accel;
    task->periTask = 1;
    cmd_peri_task_header_t *cmdHeader = (cmd_peri_task_header_t *)task->cmdHeader;
    task->cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_PERI_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_TM;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(task);
    cmdHeader->numReps = numReps;
    cmdHeader->period = period;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps, unsigned int const period,
    xtasks_task_handle *handle)
{
    acc_t *accel = (acc_t *)accId;
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    initializePeriodicTask(&_tasks[idx], id, accel, parent, compute, numReps, period);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle *handle)
{
    task_t *task = (task_t *)(*handle);
    *handle = NULL;
    task->id = 0;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(
    xtasks_arg_id const id, xtasks_arg_flags const flags, xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (argsCnt == DEF_EXEC_TASK_ARGS_LEN) {
        // Entering in extended mode
        setExtendedModeTask(task);
    }

    argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET]++;
    task->cmdExecArgs[argsCnt].flags = flags;
    task->cmdExecArgs[argsCnt].id = id;
    task->cmdExecArgs[argsCnt].value = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(
    size_t const num, xtasks_arg_flags const flags, xtasks_arg_val *const values, xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (argsCnt == DEF_EXEC_TASK_ARGS_LEN) {
        // Entering in extended mode
        xtasks_stat rv;
        rv = setExtendedModeTask(task);
        if (rv != XTASKS_SUCCESS) {
            return rv;
        }
    }

    for (size_t i = 0, idx = argsCnt; i < num; ++i, ++idx) {
        task->cmdExecArgs[idx].flags = flags;
        task->cmdExecArgs[idx].id = idx;
        task->cmdExecArgs[idx].value = values[i];
    }
    task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] += num;

    return XTASKS_SUCCESS;
}

static xtasks_stat setExtendedModeTask(task_t *task)
{
    cmd_header_t *prevHeader = task->cmdHeader;
    task->cmdHeader = (cmd_header_t *)malloc(EXT_HW_TASK_SIZE);
    if (task->cmdHeader == NULL) {
        task->cmdHeader = prevHeader;
        return XTASKS_ENOMEM;
    }
    task->extSize = 1;
    task->cmdExecArgs =
        (cmd_exec_task_arg_t *)(((unsigned char *)task->cmdHeader) +
                                (task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t)));
    memcpy(task->cmdHeader, prevHeader, DEF_EXEC_TASK_SIZE);  //< Move the hw task header and args
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    acc_t *acc = task->accel;

    // Update the task/command information
    task->cmdHeader->valid = QUEUE_VALID;

    // NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numHeaderBytes = task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t);
    size_t const numCmdWords = (numHeaderBytes + sizeof(cmd_exec_task_arg_t) * argsCnt) / sizeof(uint64_t);
    return submitCommand(acc, (uint64_t *)task->cmdHeader, numCmdWords, _cmdInQueue);
}

static xtasks_stat submitCommand(acc_t *acc, uint64_t *command, const size_t length, uint64_t *queue)
{
    size_t idx;
    uint64_t cmdHeader;
    size_t const offset = acc->info.id * CMD_IN_SUBQUEUE_LEN;
    cmd_header_t *const cmdHeaderPtr = (cmd_header_t *const) & cmdHeader;

    // While there is not enough space in the queue, look for already read commands
    while (acc->cmdInAvSlots < length) {
        ticketLockAcquire(&acc->cmdInLock);
        if (acc->cmdInAvSlots < length) {
        SUB_CMD_CHECK_RD:
            idx = acc->cmdInRdIdx;
            cmdHeader = queue[offset + idx];
            if (cmdHeaderPtr->valid == QUEUE_INVALID) {
#ifdef XTASKS_DEBUG
                uint8_t const cmdNumArgs = cmdHeaderPtr->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
                if (cmdHeaderPtr->commandCode == CMD_EXEC_TASK_CODE && cmdNumArgs > EXT_HW_TASK_ARGS_LEN) {
                    PRINT_ERROR("Found unexpected data when executing xtasksSubmitCommand");
                    ticketLockRelease(&acc->cmdInLock);
                    return XTASKS_ERROR;
                }
#endif /* XTASKS_DEBUG */
                size_t const cmdNumWords = getCmdLength(cmdHeaderPtr);
                acc->cmdInRdIdx = (idx + cmdNumWords) % CMD_IN_SUBQUEUE_LEN;
                acc->cmdInAvSlots += cmdNumWords;
            }
        } else {
            // NOTE: At this point the thread has the lock acquired, so directly bypass it into the queue idx update.
            //      The lock will be released in the code after the atomic operations
            goto SUB_CMD_UPDATE_IDX;
        }
        ticketLockRelease(&acc->cmdInLock);
    }

    ticketLockAcquire(&acc->cmdInLock);
    if (acc->cmdInAvSlots >= length) {
    SUB_CMD_UPDATE_IDX:
        idx = acc->cmdInWrIdx;
        acc->cmdInWrIdx = (idx + length) % CMD_IN_SUBQUEUE_LEN;
        acc->cmdInAvSlots -= length;
        cmdHeader = queue[offset + idx];
        cmdHeaderPtr->valid = QUEUE_RESERVED;
        queue[offset + idx] = cmdHeader;

        // Release the lock as it is not needed anymore
        __sync_synchronize();
        ticketLockRelease(&acc->cmdInLock);

        // Do no write the header (1st word pointer by command ptr) until all payload is write
        // Check if 2 writes have to be done because there is not enough space at the end of subqueue
        const size_t count = min(CMD_IN_SUBQUEUE_LEN - idx - 1, length - 1);
        memcpy(&queue[offset + idx + 1], command + 1, count * sizeof(uint64_t));
        if ((length - 1) > count) {
            memcpy(&queue[offset], command + 1 + count, (length - count) * sizeof(uint64_t));
        }
        cmdHeader = *command;
        cmdHeaderPtr->valid = QUEUE_VALID;

        __sync_synchronize();
        // Write the header now
        queue[offset + idx] = cmdHeader;
    } else {
        // NOTE: At this point the thread has the lock acquired, so directly bypass it into the check.
        //      The lock will be released in the code after the check
        goto SUB_CMD_CHECK_RD;
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    acc_t *acc = task->accel;
    size_t tries = 0;
    static size_t const MAX_WAIT_TASKS_TRIES = 0xFFFFFFFF;

    // NOTE: This implementation loses some tasks if waitTask and tryGetFinishedTask are combined.
    //      Force waiting for the first submited task?
    while (task->cmdHeader->valid == QUEUE_VALID && tries++ < MAX_WAIT_TASKS_TRIES) {
        xtasks_task_id id;
        xtasks_task_handle h;
        xtasksTryGetFinishedTaskAccel(acc, &h, &id);
    }
    return tries > MAX_WAIT_TASKS_TRIES ? XTASKS_PENDING : XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle *handle, xtasks_task_id *id)
{
    if (handle == NULL || id == NULL) {
        return XTASKS_EINVAL;
    }

    xtasks_stat ret = XTASKS_PENDING;
    size_t i = 0;
    do {
        ret = xtasksTryGetFinishedTaskAccel(&_accs[i], handle, id);
    } while (++i < _numAccs && ret == XTASKS_PENDING);

    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel, xtasks_task_handle *handle, xtasks_task_id *id)
{
    acc_t *acc = (acc_t *)accel;
    uint64_t *const subqueue = _cmdOutQueue + acc->info.id * CMD_OUT_SUBQUEUE_LEN;

    if (handle == NULL || id == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }

    size_t idx = acc->cmdOutIdx;
    uint64_t cmdBuffer = subqueue[idx];
    cmd_header_t *cmd = (cmd_header_t *)&cmdBuffer;

    if (cmd->valid == QUEUE_VALID) {
        if (__sync_lock_test_and_set(&acc->cmdOutLock, 1) == 0) {
            // Read the command header
            idx = acc->cmdOutIdx;
            cmdBuffer = subqueue[idx];

            if (cmd->valid != QUEUE_VALID) {
                __sync_lock_release(&acc->cmdOutLock);
                return XTASKS_PENDING;
            }

#ifdef XTASKS_DEBUG
            if (cmd->commandCode != CMD_FINI_EXEC_CODE ||
                cmd->commandArgs[CMD_FINI_EXEC_ARGS_ACCID_OFFSET] != acc->info.id) {
                PRINT_ERROR("Found unexpected data when executing xtasksTryGetFinishedTaskAccel");
                __sync_lock_release(&acc->cmdOutLock);
                return XTASKS_ERROR;
            }
#endif /* XTASKS_DEBUG */

            // Read the command payload (task identifier)
            size_t const dataIdx = (idx + 1) % CMD_OUT_SUBQUEUE_LEN;
            uint64_t const taskID = subqueue[dataIdx];
            subqueue[dataIdx] = 0;  //< Clean the buffer slot

            // Invalidate the buffer entry
            cmd = (cmd_header_t *)&subqueue[idx];
            cmd->valid = QUEUE_INVALID;

            // Update the read index and release the lock
            acc->cmdOutIdx = (idx + sizeof(cmd_out_exec_task_t) / sizeof(uint64_t)) % CMD_OUT_SUBQUEUE_LEN;
            __sync_lock_release(&acc->cmdOutLock);

#ifdef XTASKS_DEBUG
            if (taskID < (uintptr_t)_tasks || taskID >= (uintptr_t)(_tasks + NUM_RUN_TASKS)) {
                PRINT_ERROR("Found an invalid task identifier when executing xtasksTryGetFinishedTaskAccel");
                return XTASKS_ERROR;
            }
#endif /* XTASKS_DEBUG */

            uintptr_t taskPtr = (uintptr_t)taskID;
            task_t *task = (task_t *)taskPtr;
            *handle = (xtasks_task_handle)task;
            *id = task->id;

            // Mark the task as executed (using the valid field as it is not used in the cached copy)<
            task->cmdHeader->valid = QUEUE_INVALID;

            return XTASKS_SUCCESS;
        }
    }
    return XTASKS_PENDING;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event *events, size_t const maxCount)
{
    acc_t *acc = (acc_t *)(accel);
    xtasks_ins_event *accBuffer = _instrBuff + acc->info.id * _numInstrEvents;
    size_t count, validEvents;

    if (events == NULL || (acc - _accs) >= _numAccs || maxCount <= 0)
        return XTASKS_EINVAL;
    else if (_instrBuff == NULL)
        return XTASKS_ENOAV;

    count = min(maxCount, _numInstrEvents - acc->instrIdx);
    validEvents = getAccEvents(acc, events, count, accBuffer);
    if (validEvents < 0) {
        return XTASKS_ERROR;
    }
    if (validEvents < maxCount) {
        // Ensure invalid type of first non-wrote event slot in the caller buffer
        events[validEvents].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    return XTASKS_SUCCESS;
}

// TODO: accBuffer sems just an scratchpad to invalidate event entries
static int getAccEvents(acc_t *acc, xtasks_ins_event *events, size_t count, xtasks_ins_event *accBuffer)
{
    size_t devInstroff;
    accBuffer += acc->instrIdx;
    devInstroff = (acc->info.id * _numInstrEvents + acc->instrIdx) * sizeof(xtasks_ins_event);

    if (__sync_lock_test_and_set(&acc->instrLock, 1)) {
        // There is another thread reading the buffer for this accelerator
        events->eventType = XTASKS_EVENT_TYPE_INVALID;
        return 0;
    } else {
        xdma_status stat;
        int i;

        // stat = xdmaMemcpy(events, _instrBuffHandle, count * sizeof(xtasks_ins_event),
        //    (accBuffer - _instrBuff) * sizeof(xtasks_ins_event), XDMA_FROM_DEVICE);
        stat = xdmaMemcpy(events, _instrBuffHandle, count * sizeof(xtasks_ins_event), devInstroff, XDMA_FROM_DEVICE);
        if (stat != XDMA_SUCCESS) {
            __sync_lock_release(&acc->instrLock);
            return -1;
        }
        i = 0;
        // TODO: Preinitialize invalidation event buffers
        while (i < count && events[i].eventType != XTASKS_EVENT_TYPE_INVALID) {
            // Invalidate all read entries in the accelerator buffer
            accBuffer[i].eventType = XTASKS_EVENT_TYPE_INVALID;
            i++;
        }
        if (i > 0) {
            // stat = xdmaMemcpy(accBuffer, _instrBuffHandle, i * sizeof(xtasks_ins_event),
            //    (accBuffer - _instrBuff) * sizeof(xtasks_ins_event), XDMA_TO_DEVICE);
            stat = xdmaMemcpy(accBuffer, _instrBuffHandle, i * sizeof(xtasks_ins_event), devInstroff, XDMA_TO_DEVICE);
            if (stat != XDMA_SUCCESS) {
                __sync_lock_release(&acc->instrLock);
                return -1;
            }
        }
        acc->instrIdx = (acc->instrIdx + i) % _numInstrEvents;
        __sync_lock_release(&acc->instrLock);
        return i;
    }
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask **task)
{
    if (_spawnOutQueue == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the spawn out queue
    size_t idx, next, taskSize;
    new_task_header_t *hwTaskHeader;
    do {
        idx = _spawnOutQueueIdx;
        hwTaskHeader = (new_task_header_t *)(&_spawnOutQueue[idx]);
        if (hwTaskHeader->valid != QUEUE_VALID) {
            return XTASKS_PENDING;
        }
        taskSize =
            (sizeof(new_task_header_t) + sizeof(uint64_t) * hwTaskHeader->numArgs +
                sizeof(new_task_dep_t) * hwTaskHeader->numDeps + sizeof(new_task_copy_t) * hwTaskHeader->numCopies) /
            sizeof(uint64_t);
        next = (idx + taskSize) % SPWN_OUT_QUEUE_LEN;
    } while (!__sync_bool_compare_and_swap(&_spawnOutQueueIdx, idx, next));

    getNewTaskFromQ(task, _spawnOutQueue, idx);
    return XTASKS_SUCCESS;
}

static void getNewTaskFromQ(xtasks_newtask **task, uint64_t *spawnQueue, int idx)
{
    new_task_header_t *hwTaskHeader = (new_task_header_t *)&spawnQueue[idx];
    // Extract the information from the new buffer
    *task = realloc(*task, sizeof(xtasks_newtask) + sizeof(xtasks_newtask_arg) * hwTaskHeader->numArgs +
                               sizeof(xtasks_newtask_dep) * hwTaskHeader->numDeps +
                               sizeof(xtasks_newtask_copy) * hwTaskHeader->numCopies);
    (*task)->args = (xtasks_newtask_arg *)(*task + 1);
    (*task)->numArgs = hwTaskHeader->numArgs;
    (*task)->deps = (xtasks_newtask_dep *)((*task)->args + (*task)->numArgs);
    (*task)->numDeps = hwTaskHeader->numDeps;
    (*task)->copies = (xtasks_newtask_copy *)((*task)->deps + (*task)->numDeps);
    (*task)->numCopies = hwTaskHeader->numCopies;

    idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;  // NOTE: new_task_header_t->taskID field is the 2nd word
    (*task)->taskId = spawnQueue[idx];
    spawnQueue[idx] = 0;  //< Cleanup the memory position

    idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;  // NOTE: new_task_header_t->parentID field is the 3th word
    (*task)->parentId = spawnQueue[idx];   //< NOTE: We don't know what is that ID (SW or HW)
    spawnQueue[idx] = 0;                   //< Cleanup the memory position

    idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;  // NOTE: new_task_header_t->typeInfo field is the 4th word
    (*task)->typeInfo = spawnQueue[idx];
    spawnQueue[idx] = 0;  //< Cleanup the memory position

    for (size_t i = 0; i < (*task)->numDeps; ++i) {
        idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;
        new_task_dep_t *hwTaskDep = (new_task_dep_t *)(&spawnQueue[idx]);

        // Parse the dependence information
        (*task)->deps[i].address = hwTaskDep->address;
        (*task)->deps[i].flags = hwTaskDep->flags;

        // Cleanup the memory position
        spawnQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numCopies; ++i) {
        // NOTE: Each copy uses 3 uint64_t elements in the newQueue
        //      After using each memory position, we have to clean it
        uint64_t tmp;

        // NOTE: new_task_copy_t->address field is the 1st word
        idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;
        (*task)->copies[i].address = (void *)((uintptr_t)spawnQueue[idx]);
        spawnQueue[idx] = 0;

        // NOTE: new_task_copy_t->flags and new_task_copy_t->size fields are the 2nd word
        idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;
        tmp = spawnQueue[idx];
        uint8_t copyFlags = tmp >> NEW_TASK_COPY_FLAGS_WORDOFFSET;
        (*task)->copies[i].flags = copyFlags;
        uint32_t copySize = tmp >> NEW_TASK_COPY_SIZE_WORDOFFSET;
        (*task)->copies[i].size = copySize;
        spawnQueue[idx] = 0;

        // NOTE: new_task_copy_t->offset and new_task_copy_t->accessedLen fields are the 2nd word
        idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;
        tmp = spawnQueue[idx];
        uint32_t copyOffset = tmp >> NEW_TASK_COPY_OFFSET_WORDOFFSET;
        (*task)->copies[i].offset = copyOffset;
        uint32_t copyAccessedLen = tmp >> NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET;
        (*task)->copies[i].accessedLen = copyAccessedLen;
        spawnQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numArgs; ++i) {
        // Check that arg pointer is not out of bounds
        idx = (idx + 1) % SPWN_OUT_QUEUE_LEN;
        (*task)->args[i] = spawnQueue[idx];

        // Cleanup the memory position
        spawnQueue[idx] = 0;
    }

    // Free the buffer slot
    // NOTE: This word cannot be set to 0 as the task size information must be keept
    __sync_synchronize();
    hwTaskHeader->valid = QUEUE_INVALID;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id)
{
    // Get an empty slot into the TM remote finished queue
    size_t idx, next;
    rem_fini_task_t *entryHeader;
    do {
        idx = _spawnInQueueIdx;
        entryHeader = (rem_fini_task_t *)(&_spawnInQueue[idx]);
        if (entryHeader->valid == QUEUE_VALID) {
            return XTASKS_ENOENTRY;
        }
        next = (idx + sizeof(rem_fini_task_t) / sizeof(uint64_t)) % SPWN_IN_QUEUE_LEN;
    } while (!__sync_bool_compare_and_swap(&_spawnInQueueIdx, idx, next));

    // NOTE: rem_fini_task_t->taskId is the 1st word
    idx = (idx + 1) % SPWN_IN_QUEUE_LEN;
    _spawnInQueue[idx] = id;

    // NOTE: rem_fini_task_t->parentId is the 2nd word
    idx = (idx + 1) % SPWN_IN_QUEUE_LEN;
    _spawnInQueue[idx] = parent;

    __sync_synchronize();
    entryHeader->valid = QUEUE_VALID;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksMalloc(size_t len, xtasks_mem_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaAllocate(handle, len);
    return toXtasksStat(status);
}

xtasks_stat xtasksFree(xtasks_mem_handle handle)
{
    xdma_status status = xdmaFree(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetAccAddress(xtasks_mem_handle const handle, uint64_t *addr)
{
    if (addr == NULL) return XTASKS_EINVAL;

    unsigned long devAddr = 0;
    xdma_status status = xdmaGetDeviceAddress(handle, &devAddr);
    *addr = devAddr;
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpy(
    xtasks_mem_handle const handle, size_t offset, size_t len, void *usr, xtasks_memcpy_kind const kind)
{
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpy(usr, handle, len, offset, mode);
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpyAsync(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
    xtasks_memcpy_kind const kind, xtasks_memcpy_handle *cpyHandle)
{
    if (handle == NULL) return XTASKS_EINVAL;
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpyAsync(usr, handle, len, offset, mode, cpyHandle);
    return toXtasksStat(status);
}

xtasks_stat xtasksTestCopy(xtasks_memcpy_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaTestTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksSyncCopy(xtasks_memcpy_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaWaitTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp *timestamp)
{
    *timestamp = *_instrCounter;
    return XTASKS_SUCCESS;
}
