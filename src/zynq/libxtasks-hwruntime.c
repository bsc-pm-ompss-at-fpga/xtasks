/*--------------------------------------------------------------------
  (C) Copyright 2017-2021 Barcelona Supercomputing Center
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

#include "../libxtasks.h"
#include "../util/common.h"
#include "../util/ticket-lock.h"
#include "platform.h"

#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <libxdma.h>
#include <libxdma_version.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include <assert.h>

#define DEF_ACCS_LEN 8  ///< Def. allocated slots in the accs array

#define CMD_IN_QUEUE_PATH "/dev/ompss_fpga/hwruntime/cmd_in_queue"
#define CMD_OUT_QUEUE_PATH "/dev/ompss_fpga/hwruntime/cmd_out_queue"
#define SPAWN_OUT_QUEUE_PATH "/dev/ompss_fpga/hwruntime/spawn_out_queue"
#define SPAWN_IN_QUEUE_PATH "/dev/ompss_fpga/hwruntime/spawn_in_queue"
#define HWRUNTIME_RST_PATH "/dev/ompss_fpga/hwruntime/ctrl"

//! Check that libxdma version is compatible
#define LIBXTASKS_MIN_MAJOR 3
#define LIBXTASKS_MIN_MINOR 1
#if !defined(LIBXDMA_VERSION_MAJOR) || !defined(LIBXDMA_VERSION_MINOR) || \
    LIBXDMA_VERSION_MAJOR < LIBXTASKS_MIN_MAJOR || \
    (LIBXDMA_VERSION_MAJOR == LIBXTASKS_MIN_MAJOR && LIBXDMA_VERSION_MINOR < LIBXTASKS_MIN_MINOR)
#error Installed libxdma is not supported (use >= 3.1)
#endif

#define MAX_ACC_CONFIG_SIZE 4096

//! \brief Platform and Backend strings
const char _platformName[] = "zynq";
const char _backendName[] = "hwruntime";

static int _init_cnt = 0;                 ///< Counter of calls to init/fini
static size_t _numAccs;                   ///< Number of accelerators in the system
static acc_t *_accs;                      ///< Accelerators data
static uint8_t *_cmdExecTaskBuff;         ///< Buffer to send the HW tasks
static task_t *_tasks;                    ///< Array with internal task information
static int _cmdInQFd;                     ///< File descriptior of command IN queue
static int _cmdOutQFd;                    ///< File descriptior of command OUT queue
static int _spawnOutQFd;                  ///< File descriptior of spawn OUT queue
static int _spawnInQFd;                   ///< File descriptior of spawn IN queue
static int _ctrlFd;                       ///< File descriptior of CTRL device
static uint64_t *_cmdInQueue;             ///< Command IN queue
static uint64_t *_cmdOutQueue;            ///< Command OUT queue
static uint64_t *_spawnOutQueue;          ///< Spawn OUT queue (buffer of FPGA spawned tasks)
static size_t _spawnOutQueueIdx;          ///< Reading index of the _spawnOutQueue
static uint64_t *_spawnInQueue;           ///< Spawn IN queue (buffer of finished FPGA spawned tasks)
static size_t _spawnInQueueIdx;           ///< Writing index of the _spawnInQueue
static uint32_t volatile *_hwruntimeRst;  ///< Register to reset the HW runtime
static uint32_t _cmdInSubqueueLen;        ///< Size (words) of each subqueue of command IN queue
static uint32_t _cmdOutSubqueueLen;       ///< Size (words) of each subqueue of command OUT queue
static uint32_t _spawnInQueueLen;         ///< Size (words) of spawn IN queue
static uint32_t _spawnOutQueueLen;        ///< Size (words) of spawn OUT queue

static size_t _numInstrEvents;            ///< Number of instrumentation events for each accelerator buffer
static xtasks_ins_event *_instrBuff;      ///< Buffer of instrumentation events
static xtasks_ins_event *_instrBuffPhy;   ///< Physical address of _instrBuff
static xdma_buf_handle _instrBuffHandle;  ///< Handle of _instrBuff in libxdma
static xtasks_ins_event *_invalBuffer;    ///< Invalidated event buffer used to push invalidations into device mem

/*!
 * \breif Set n times the byte c in dst
 * \note Assuming: n is multiple of 32, dst and src are aligned to 32 bits
 * \note Avoid optimizations for the function implementation as aarch64 cannot execute
 *       some fast copy instructions over non-cacheable memory
 */
void _memset(void *dst, int c, size_t n);
inline void __attribute__((optimize("O1"))) _memset(void *dst, int c, size_t n)
{
#if __aarch64__
    uint32_t *d = (uint32_t *)dst;
    char cc = c;
    uint32_t v;
    v = cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    for (size_t i = 0; i < n / sizeof(uint32_t); ++i) {
        d[i] = v;
    }
#else
    memset(dst, c, n);
#endif
}

xtasks_stat xtasksInitHWIns(size_t const nEvents)
{
    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;
    size_t insBufferSize;
    unsigned long phyAddr;

    // At least we need 1 event + last event mark
    if (nEvents <= 1) return XTASKS_EINVAL;

    // Check if bitstream has the HW instrumentation feature
    if (checkbitstreamFeature("hwcounter") == BIT_FEATURE_NO_AVAIL) {
        return XTASKS_ENOAV;
    }

    s = xdmaInitHWInstrumentation();
    if (s != XDMA_SUCCESS) {
        // Return as there's nothing to undo
        return XTASKS_ENOAV;
        // goto intrInitErr;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents * _numAccs * sizeof(xtasks_ins_event);
    s = xdmaAllocateHost((void **)&_instrBuff, &_instrBuffHandle, insBufferSize);
    _invalBuffer = malloc(_numInstrEvents * sizeof(*_invalBuffer));
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        goto instrAllocErr;
    }

    // get phy address
    s = xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        goto instrGetAddrErr;
    }
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    // Invalidate all entries
    for (size_t i = 0; i < _numInstrEvents * _numAccs; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    // Initialize invalidation buffer
    for (size_t i = 0; i < _numInstrEvents; ++i) {
        _invalBuffer[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    // Send the instrumentation buffer to each accelerator
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    memcpy((void *)cmd.header.commandArgs, (void *)&_numInstrEvents, CMD_SETUP_INS_ARGS_NUMEVS_BYTES);
    for (size_t i = 0; i < _numAccs; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents * i);

        ret = submitCommand(
            _accs + i, (uint64_t *)&cmd, sizeof(cmd_setup_hw_ins_t) / sizeof(uint64_t), _cmdInQueue, _cmdInSubqueueLen);

        if (ret != XTASKS_SUCCESS) {
            goto instrSendInit;
        }

        _accs[i].instrIdx = 0;
        _accs[i].instrLock = 0;
    }

    return XTASKS_SUCCESS;

instrSendInit:
instrGetAddrErr:
instrAllocErr:
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuff = NULL;
    _instrBuffPhy = NULL;
    return ret;
}

xtasks_stat xtasksFiniHWIns()
{
    xdma_status s0 = XDMA_SUCCESS;
    xdma_status s1 = XDMA_SUCCESS;

    if (_instrBuff == NULL) {
        // Instrumentation is not initialized or has been already finished
        return XTASKS_SUCCESS;
    }
    s1 = xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuff = NULL;
    _instrBuffPhy = NULL;
    s0 = xdmaFiniHWInstrumentation();

    return (s0 == XDMA_SUCCESS && s1 == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksInit()
{
    // Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    // Always initialize instrumentation as disabled
    _instrBuff = NULL;
    _numInstrEvents = 0;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    // Check if bitstream is compatible
    bit_compatibility_t compat = checkbitstreamCompatibility();
    if (compat == BIT_NO_COMPAT || compat == BIT_COMPAT_UNKNOWN) {
        ret = XTASKS_ERROR;
        goto INIT_ERR_0;
    }

    // Initialize xdma memory subsystem
    s = xdmaInitMem();
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        if (s == XDMA_ENOENT) {
            PRINT_ERROR("xdmaInitMem failed. Check if xdma device exist in the system");
        } else if (s == XDMA_EACCES) {
            PRINT_ERROR("xdmaInitMem failed. Current user cannot access xdma device");
        } else {
            PRINT_ERROR("xdmaInitMem failed");
        }
        goto INIT_ERR_0;
    }

    // Open cdma device
    s = xdmaOpen();
    if (s != XDMA_SUCCESS) {
        PRINT_ERROR("could not open xdma device");
        goto INIT_ERR_1;
    }

    // Generate the configuration file path
    char *buffer = getConfigFilePath();
    if (buffer == NULL) {
        ret = XTASKS_EFILE;
        printErrorMsgCfgFile();
        goto INIT_ERR_2;
    }

    // Open the configuration file and parse it
    FILE *accMapFile = fopen(buffer, "r");
    free(buffer);
    if (accMapFile == NULL) {
        ret = XTASKS_EFILE;
        printErrorMsgCfgFile();
        goto INIT_ERR_2;
    }

    char *accInfo = malloc(MAX_ACC_CONFIG_SIZE);
    size_t nread = fread(accInfo, 1, MAX_ACC_CONFIG_SIZE, accMapFile);
    fclose(accMapFile);
    if (nread == 0) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot read accelerator config");
        free(accInfo);
        goto INIT_ERR_2;
    }

    uint32_t hwruntimeIOStruct[BITINFO_HWRIO_STRUCT_WORDS];
    if (getBitStreamHwrIOStruct(hwruntimeIOStruct) < 0) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot read hwruntime io struct");
        free(accInfo);
        goto INIT_ERR_2;
    }
    _cmdInSubqueueLen = hwruntimeIOStruct[CMD_IN_BITINFO_LEN_OFFSET];
    _cmdOutSubqueueLen = hwruntimeIOStruct[CMD_OUT_BITINFO_LEN_OFFSET];
    int numAccs = getBitStreamNumAccs();
    if (numAccs < 0) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot read number of accelerators");
        free(accInfo);
        goto INIT_ERR_2;
    }
    _accs = malloc(numAccs * sizeof(acc_t));
    if (_accs == NULL) {
        PRINT_ERROR("Could not allocate accelerators array");
        free(accInfo);
        goto INIT_ERR_2;
    }
    _numAccs = initAccList(_accs, accInfo, _cmdInSubqueueLen);
    free(accInfo);

    // Open and map the Task Manager queues into library memory
    _cmdInQFd = open(CMD_IN_QUEUE_PATH, O_RDWR, (mode_t)0600);
    _cmdOutQFd = open(CMD_OUT_QUEUE_PATH, O_RDWR, (mode_t)0600);
    _ctrlFd = open(HWRUNTIME_RST_PATH, O_RDWR, (mode_t)0600);
    if (_cmdInQFd < 0 || _cmdOutQFd < 0 || _ctrlFd < 0) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open basic hwruntime device files");
        goto INIT_ERR_OPEN_COMM;
    }

    if (checkbitstreamFeature("hwruntime_ext") == BIT_FEATURE_NO_AVAIL) {
        // Do not try to open the extended queues as we know that are not available
        _spawnOutQFd = -1;
        _spawnInQFd = -1;
    } else {
        _spawnInQueueLen = hwruntimeIOStruct[SPWN_IN_BITINFO_LEN_OFFSET];
        _spawnOutQueueLen = hwruntimeIOStruct[SPWN_OUT_BITINFO_LEN_OFFSET];
        _spawnOutQFd = open(SPAWN_OUT_QUEUE_PATH, O_RDWR, (mode_t)0600);
        if (_spawnOutQFd < 0) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open hwruntime/spawn_out_queue device file");
            goto INIT_ERR_OPEN_SPWN_OUT;
        }
        _spawnInQFd = open(SPAWN_IN_QUEUE_PATH, O_RDWR, (mode_t)0600);
        if (_spawnInQFd < 0) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open hwruntime/spawn_in_queue device file");
            goto INIT_ERR_OPEN_SPWN_IN;
        }
    }

    _cmdInQueue = (uint64_t *)mmap(
        NULL, _cmdInSubqueueLen * numAccs * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, _cmdInQFd, 0);
    if (_cmdInQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_in_queue");
        goto INIT_ERR_MMAP_CMD_IN;
    }

    // If any, invalidate commands in cmd_in_queue
    _memset(_cmdInQueue, 0, _cmdInSubqueueLen * numAccs * sizeof(uint64_t));

    _cmdOutQueue = (uint64_t *)mmap(
        NULL, _cmdOutSubqueueLen * numAccs * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, _cmdOutQFd, 0);
    if (_cmdOutQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_out_queue");
        goto INIT_ERR_MMAP_CMD_OUT;
    }

    // If any, invalidate commands in cmd_out_queue
    _memset(_cmdOutQueue, 0, _cmdOutSubqueueLen * numAccs * sizeof(uint64_t));

    _spawnOutQueueIdx = 0;
    if (_spawnOutQFd == -1) {
        _spawnOutQueue = NULL;
    } else {
        _spawnOutQueue = (uint64_t *)mmap(
            NULL, _spawnOutQueueLen * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, _spawnOutQFd, 0);
        if (_spawnOutQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_out_queue");
            goto INIT_ERR_MAP_SPWN_OUT;
        }

        // If any, invalidate tasks in spawn_out_queue
        _memset(_spawnOutQueue, 0, _spawnOutQueueLen * sizeof(uint64_t));
    }

    _spawnInQueueIdx = 0;
    if (_spawnInQFd == -1) {
        _spawnInQueue = NULL;
    } else {
        _spawnInQueue = (uint64_t *)mmap(
            NULL, _spawnInQueueLen * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, _spawnInQFd, 0);
        if (_spawnInQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_in_queue");
            goto INIT_ERR_MAP_SPWN_IN;
        }

        // If any, invalidate tasks in spawn_in_queue
        _memset(_spawnInQueue, 0, _spawnInQueueLen * sizeof(uint64_t));
    }

    _hwruntimeRst = (uint32_t *)mmap(NULL, sizeof(uint32_t), PROT_READ | PROT_WRITE, MAP_SHARED, _ctrlFd, 0);
    if (_hwruntimeRst == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/ctrl");
        goto INIT_ERR_MMAP_RST;
    }

    resetHWRuntime(_hwruntimeRst);

    // Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS * sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto INIT_ERR_ALLOC_TASKS;
    }
    _cmdExecTaskBuff = (uint8_t *)malloc(NUM_RUN_TASKS * DEF_EXEC_TASK_SIZE);
    if (_cmdExecTaskBuff == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for exec. tasks buffer");
        goto INIT_ERR_ALLOC_EXEC_TASKS_BUFF;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].cmdHeader = (cmd_header_t *)&_cmdExecTaskBuff[idx * DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)0;
        _tasks[idx].extSize = 0;
        _tasks[idx].periTask = 0;
    }

    return ret;

    // Error handling code
    free(_cmdExecTaskBuff);
INIT_ERR_ALLOC_EXEC_TASKS_BUFF:
    free(_tasks);
INIT_ERR_ALLOC_TASKS:
INIT_ERR_MMAP_RST:
    if (_spawnInQueue != NULL) munmap(_spawnInQueue, sizeof(uint64_t) * _spawnInQueueLen);
INIT_ERR_MAP_SPWN_IN:
    if (_spawnOutQueue != NULL) munmap(_spawnOutQueue, sizeof(uint64_t) * _spawnOutQueueLen);
INIT_ERR_MAP_SPWN_OUT:
    munmap(_cmdOutQueue, sizeof(uint64_t) * _cmdOutSubqueueLen * numAccs);
INIT_ERR_MMAP_CMD_OUT:
    munmap(_cmdInQueue, sizeof(uint64_t) * _cmdInSubqueueLen * numAccs);
INIT_ERR_MMAP_CMD_IN:
    if (_spawnInQFd != -1) close(_spawnInQFd);
INIT_ERR_OPEN_SPWN_IN:
    if (_spawnOutQFd != -1) close(_spawnOutQFd);
INIT_ERR_OPEN_SPWN_OUT:
INIT_ERR_OPEN_COMM:
    free(_accs);
    close(_ctrlFd);
    close(_cmdOutQFd);
    close(_cmdInQFd);
    _numAccs = 0;
INIT_ERR_2:
    xdmaClose();
INIT_ERR_1:
    xdmaFiniMem();
INIT_ERR_0:
    __sync_sub_and_fetch(&_init_cnt, 1);
    return ret;
}

xtasks_stat xtasksFini()
{
    // Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    // Free tasks array
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        if (_tasks[idx].extSize) {
            // Free tasks in extended mode
            free((void *)_tasks[idx].cmdHeader);
        }
    }
    free(_cmdExecTaskBuff);
    free(_tasks);
    free(_accs);
    _tasks = NULL;

    // Finialize the HW instrumentation if needed
    if (xtasksFiniHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    // Unmap the HW runtime queues
    int statusRd, statusFi, statusNw, statusRFi, statusCtrl;
    statusCtrl = munmap((void *)_hwruntimeRst, sizeof(uint32_t));
    statusRFi = _spawnInQueue != NULL ? munmap(_spawnInQueue, sizeof(uint64_t) * _spawnInQueueLen) : 0;
    statusNw = _spawnOutQueue != NULL ? munmap(_spawnOutQueue, sizeof(uint64_t) * _spawnOutQueueLen) : 0;
    statusFi = munmap(_cmdOutQueue, sizeof(uint64_t) * _numAccs * _cmdOutSubqueueLen);
    statusRd = munmap(_cmdInQueue, sizeof(uint64_t) * _numAccs * _cmdInSubqueueLen);
    if (statusRd == -1 || statusFi == -1 || statusNw == -1 || statusRFi == -1 || statusCtrl == -1) {
        ret = XTASKS_EFILE;
    }

    statusCtrl = close(_ctrlFd);
    statusRFi = _spawnInQFd != -1 ? close(_spawnOutQFd) : 0;
    statusNw = _spawnOutQFd != -1 ? close(_spawnInQFd) : 0;
    statusFi = close(_cmdOutQFd);
    statusRd = close(_cmdInQFd);
    if (statusRd == -1 || statusFi == -1 || statusNw == -1 || statusRFi == -1 || statusCtrl == -1) {
        ret = XTASKS_EFILE;
    }

    // Free the accelerators array
    for (size_t idx = 0; idx < _numAccs; ++idx) {
        ticketLockFini(&_accs[idx].cmdInLock);
    }
    _numAccs = 0;

    if (xdmaClose() != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    // Close xdma memory management
    if (xdmaFiniMem() != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    return ret;
}

xtasks_stat xtasksGetPlatform(const char **name)
{
    if (name == NULL) return XTASKS_EINVAL;

    *name = _platformName;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetBackend(const char **name)
{
    if (name == NULL) return XTASKS_EINVAL;

    *name = _backendName;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(size_t *count)
{
    if (count == NULL) return XTASKS_EINVAL;

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
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    initializeTask(&_tasks[idx], id, accel, parent, compute);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps, unsigned int const period,
    xtasks_task_handle *handle)
{
    acc_t *accel = (acc_t *)accId;
    int idx = getFreeTaskEntry(_tasks);
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
    if (num > EXT_HW_TASK_ARGS_LEN - argsCnt) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (num > (DEF_EXEC_TASK_ARGS_LEN - argsCnt) && argsCnt <= DEF_EXEC_TASK_ARGS_LEN) {
        // Entering in extended mode because:
        // 1) The number of args to add does not fit in current allocated space
        // 2) We are not in extended mode
        // 3) The number of args will fit in extended mode
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

    for (size_t i = 0, idx = argsCnt; i < num; ++i, ++idx) {
        task->cmdExecArgs[idx].flags = flags;
        task->cmdExecArgs[idx].id = idx;
        task->cmdExecArgs[idx].value = values[i];
    }
    task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    acc_t *acc = task->accel;

    if (task == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }
    // Update the task/command information
    task->cmdHeader->valid = QUEUE_VALID;

    // NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numHeaderBytes = task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t);
    size_t const numCmdWords = (numHeaderBytes + sizeof(cmd_exec_task_arg_t) * argsCnt) / sizeof(uint64_t);
    return submitCommand(acc, (uint64_t *)task->cmdHeader, numCmdWords, _cmdInQueue, _cmdInSubqueueLen);
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
    uint64_t *const subqueue = _cmdOutQueue + acc->info.id * _cmdOutSubqueueLen;

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
            if (cmd->commandCode != CMD_FINI_EXEC_CODE) {
                PRINT_ERROR("Found unexpected data when executing xtasksTryGetFinishedTaskAccel");
                __sync_lock_release(&acc->cmdOutLock);
                return XTASKS_ERROR;
            }
#endif /* XTASKS_DEBUG */

            // Read the command payload (task identifier)
            size_t const dataIdx = (idx + 1) % _cmdOutSubqueueLen;
            uint64_t const taskID = subqueue[dataIdx];
            subqueue[dataIdx] = 0;  //< Clean the buffer slot

            // Invalidate the buffer entry
            cmd = (cmd_header_t *)&subqueue[idx];
            cmd->valid = QUEUE_INVALID;

            // Update the read index and release the lock
            acc->cmdOutIdx = (idx + sizeof(cmd_out_exec_task_t) / sizeof(uint64_t)) % _cmdOutSubqueueLen;
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
    size_t count, validEvents;

    if (events == NULL || (acc - _accs) >= _numAccs || maxCount <= 0)
        return XTASKS_EINVAL;
    else if (_instrBuff == NULL)
        return XTASKS_ENOAV;

    count = min(maxCount, _numInstrEvents - acc->instrIdx);
    validEvents = getAccEvents(acc, events, count, _numInstrEvents, _instrBuffHandle, _invalBuffer);
    if (validEvents < 0) {
        return XTASKS_ERROR;
    }
    if (validEvents < maxCount) {
        // Ensure invalid type of first non-wrote event slot in the caller buffer
        events[validEvents].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    return XTASKS_SUCCESS;
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
        next = (idx + taskSize) % _spawnOutQueueLen;
    } while (!__sync_bool_compare_and_swap(&_spawnOutQueueIdx, idx, next));

    getNewTaskFromQ(task, _spawnOutQueue, idx, _spawnOutQueueLen);

    return XTASKS_SUCCESS;
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
        next = (idx + sizeof(rem_fini_task_t) / sizeof(uint64_t)) % _spawnInQueueLen;
    } while (!__sync_bool_compare_and_swap(&_spawnInQueueIdx, idx, next));

    // NOTE: rem_fini_task_t->taskId is the 1st word
    idx = (idx + 1) % _spawnInQueueLen;
    _spawnInQueue[idx] = id;

    // NOTE: rem_fini_task_t->parentId is the 2nd word
    idx = (idx + 1) % _spawnInQueueLen;
    _spawnInQueue[idx] = parent;

    __sync_synchronize();
    entryHeader->valid = QUEUE_VALID;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp *timestamp)
{
    if (timestamp == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaGetDeviceTime(timestamp);
    return toXtasksStat(status);
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

#ifdef XTASKS_DEBUG
uint64_t cmdInQueue(size_t const accID, size_t const idx) { return _cmdInQueue[accID * _cmdInSubqueueLen + idx]; }

uint64_t cmdOutQueue(size_t const accID, size_t const idx) { return _cmdOutQueue[accID * _cmdOutSubqueueLen + idx]; }

uint64_t spawnOutQueue(size_t const idx) { return _spawnOutQueue[idx]; }

uint64_t spawnInQueue(size_t const idx) { return _spawnInQueue[idx]; }

xtasks_ins_event accelGetEvent(size_t const aIdx, size_t const evIdx)
{
    return _instrBuff[aIdx * _numInstrEvents + evIdx];
}

void accelPrintInstrBuffer(size_t const aIdx)
{
    xtasks_ins_event *event = _instrBuff + aIdx * _numInstrEvents;
    fprintf(stderr, "timestamp, accid, eventid, value\n");
    while (event->eventId != XTASKS_EVENT_TYPE_INVALID) {
        fprintf(stderr, "%llu,\t%u,\t%u,\t%llu\n", (unsigned long long int)event->timestamp, event->eventType,
            event->eventId, (unsigned long long int)event->value);
        event++;
    }
}

#endif
