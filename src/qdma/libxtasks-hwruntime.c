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

#define ACC_INFO_MAX_LEN 4096

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
    uint32_t *bitInfo = malloc(BITINFO_MAX_SIZE);
    memcpy(bitInfo, _pciBar + BRAM_INFO_ADDRESS / sizeof(*_pciBar), BITINFO_MAX_SIZE);

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
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOENTRY;
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
    validEvents = getAccEvents(acc, events, count, _numInstrEvents, _instrBuffHandle, accBuffer);
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
        next = (idx + taskSize) % SPWN_OUT_QUEUE_LEN;
    } while (!__sync_bool_compare_and_swap(&_spawnOutQueueIdx, idx, next));

    getNewTaskFromQ(task, _spawnOutQueue, idx);
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
