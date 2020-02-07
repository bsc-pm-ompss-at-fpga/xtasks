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

#include "../libxtasks.h"
#include "../util/common.h"
#include "platform.h"
#include "../util/queue.h"

#include <libxdma.h>
#include <libxdma_version.h>
#include <elf.h>
#include <stdio.h>
#include <stddef.h>
#include <alloca.h>

#define MAX_CNT_FIN_TASKS       16              ///< Max. number of finished tasks processed for other accels before return
#define DEF_EXEC_TASK_SIZE      256             ///< Size of hw task when using the defult num. of args.
#define DEF_EXEC_TASK_ARGS_LEN  14              //NOTE: (DEF_EXEC_TASK_SIZE - sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
#define NUM_RUN_TASKS           (8192/256)      //NOTE: 2 pages (8192 bytes) considering that each task is 256 bytes

//! Check that libxdma version is compatible
#define LIBXTASKS_MIN_MAJOR 3
#define LIBXTASKS_MIN_MINOR 1
#if !defined(LIBXDMA_VERSION_MAJOR) || !defined(LIBXDMA_VERSION_MINOR) || \
    LIBXDMA_VERSION_MAJOR < LIBXTASKS_MIN_MAJOR || \
    (LIBXDMA_VERSION_MAJOR == LIBXTASKS_MIN_MAJOR && LIBXDMA_VERSION_MINOR < LIBXTASKS_MIN_MINOR)
# error Installed libxdma is not supported (use >= 3.1)
#endif

//! \brief Platform and Backend strings
const char _platformName[] = "zynq";
const char _backendName[] = "stream";

//! \brief HW accelerator representation
typedef struct {
    xdma_device              xdmaDev;
    xdma_channel             inChannel;
    xdma_channel             outChannel;
    char                     descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info          info;
    queue_t *                tasksQueue;
    unsigned short volatile  tasksQueueLock;
    unsigned short volatile  instrIdx;          ///< Reading index of the accelerator instrumentation buffer
    unsigned short volatile  instrLock;         ///< Lock for atomic operations over instrumentation buffers
} acc_t;

typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    cmd_exec_task_header_t *cmdHeader;     ///< Pointer to the cmd_exec_task_header_t struct
    cmd_exec_task_arg_t *   cmdExecArgs;   ///< Pointer to the array of cmd_exec_task_arg_t structs
    uint8_t                 argsCnt;       ///< Number of arguments in the task
    acc_t *                 accel;         ///< Accelerator where the task will run
    xdma_transfer_handle    cmdExecTx;     ///< Execute command transfer handle
    xdma_transfer_handle    syncTx;        ///< Task sync transfer handle
    xdma_buf_handle         taskHandle;    ///< Task buffer handle (only used if: argsCnt > DEF_EXEC_TASK_ARGS_LEN)
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint8_t *            _cmdExecTaskBuff;   ///< Buffer to send the HW tasks
static xdma_buf_handle      _cmdExecTaskBuffHandle; ///< Handle of _cmdExecTaskBuff in libxdma
static task_t *             _tasks;             ///< Array with internal task information

static size_t               _numInstrEvents;    ///< Number of instrumentation events for each accelerator buffer
static xtasks_ins_event    *_instrBuff;         ///< Buffer of instrumentation events
static xtasks_ins_event    *_instrBuffPhy;      ///< Physical address of _instrBuff
static xdma_buf_handle      _instrBuffHandle;   ///< Handle of _instrBuff in libxdma

xtasks_stat xtasksInitHWIns(size_t const nEvents)
{
    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;
    size_t insBufferSize;
    unsigned long phyAddr;

    //At least we need 1 event + last event mark
    _numInstrEvents = 0;
    if (nEvents <= 1) return XTASKS_EINVAL;

    //Check if bitstream has the HW instrumentation feature
    if (checkbitstreamFeature("hwcounter") == BIT_FEATURE_NO_AVAIL) {
        return XTASKS_ENOAV;
    }

    s = xdmaInitHWInstrumentation();
    if (s != XDMA_SUCCESS) {
        return XTASKS_ENOAV;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents*_numAccs*sizeof(xtasks_ins_event);
    s = xdmaAllocateHost((void **)&_instrBuff, &_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        goto INIT_INS_ERR_ALLOC;
    }

    //get phy address
    s = xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        goto INIT_INS_ERR_GET_ADDR;
    }
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    //Invalidate all entries
    for (size_t i = 0; i < _numInstrEvents*_numAccs; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    //Send the instrumentation buffer to each accelerator
    xdma_buf_handle cmdBufferHandle;
    s = xdmaAllocate(&cmdBufferHandle, sizeof(cmd_setup_hw_ins_t));
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        goto INIT_INS_ERR_ALLOC_CMD;
    }
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    uint32_t * cmdArgs = (uint32_t *)&cmd.header.commandArgs;
    *cmdArgs = _numInstrEvents;
    for (size_t i = 0; i < _numAccs; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents*i);
        s = xdmaMemcpy((void *)&cmd, cmdBufferHandle, sizeof(cmd_setup_hw_ins_t), 0 /*offset*/, XDMA_TO_DEVICE);
        if (s != XDMA_SUCCESS) {
            ret = XTASKS_ERROR;
            goto INIT_INS_ERR_SEND_CMD;
        }
        s = xdmaStream(cmdBufferHandle, sizeof(cmd_setup_hw_ins_t), 0 /*offset*/, _accs[i].xdmaDev, _accs[i].inChannel);
        if (ret != XTASKS_SUCCESS) {
            ret = XTASKS_ERROR;
            goto INIT_INS_ERR_SEND_CMD;
        }

        _accs[i].instrIdx = 0;
        _accs[i].instrLock = 0;
    }
    xdmaFree(cmdBufferHandle);

    return XTASKS_SUCCESS;

INIT_INS_ERR_SEND_CMD:
    xdmaFree(cmdBufferHandle);
INIT_INS_ERR_ALLOC_CMD:
INIT_INS_ERR_GET_ADDR:
    xdmaFree(_instrBuffHandle);
INIT_INS_ERR_ALLOC:
    _numInstrEvents = 0;
    _instrBuff = NULL;
    _instrBuffPhy = NULL;
    return ret;
}

xtasks_stat xtasksFiniHWIns()
{
    if (_numInstrEvents > 0) {
        xdma_status s0, s1;
        s0 = xdmaFiniHWInstrumentation();
        s1 = xdmaFree(_instrBuffHandle);
        _numInstrEvents = 0;
        _instrBuff = NULL;
        _instrBuffPhy = NULL;
        return (s0 == XDMA_SUCCESS && s1 == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
    }
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    //Check if bitstream is compatible
    if (checkbitstreamCompatibility() == BIT_NO_COMPAT) {
        printErrorBitstreamCompatibility();
        return XTASKS_ERROR;
    }

    //Check if bitstream has the dma feature
    if (checkbitstreamFeature("dma") == BIT_FEATURE_NO_AVAIL) {
        ret = XTASKS_ENOAV;
        PRINT_ERROR("OmpSs@FPGA DMA feature not available in the loaded FPGA bitstream");
        goto INIT_ERR_CHECK_FEAT;
    }

    s = xdmaInitMem();
    if (s != XDMA_SUCCESS) {
        switch (s) {
            case XDMA_ENOENT:
                PRINT_ERROR("InitMem failed: File not found");
                ret = XTASKS_ENOENTRY;
                break;
            case XDMA_EACCES:
                PRINT_ERROR("InitMem failed: Permission denied");
                ret = XTASKS_ERROR;
            default:
                PRINT_ERROR("InitMem failed.");
                ret = XTASKS_ERROR;
        }
        goto INIT_ERR_INIT_MEM;
    }

    //Open libxdma
    s = xdmaOpen();
    if (s != XDMA_SUCCESS) {
        if (s == XDMA_ENOENT) {
            PRINT_ERROR("xdmaOpen failed. Check if xdma device exist in the system");
            ret = XTASKS_ENOENTRY;
        } else if (s == XDMA_EACCES) {
            PRINT_ERROR("xdmaOpen failed. Current user cannot access xdma device");
            ret = XTASKS_ERROR;
        } else {
            PRINT_ERROR("xdmaOpen failed");
            ret = XTASKS_ERROR;
        }
        goto INIT_ERR_XDMA_OPEN;
    }

    //Get the number of devices from libxdma
    int xdmaDevices;
    if (xdmaGetNumDevices(&xdmaDevices) != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("xdmaGetNumDevices failed");
        goto INIT_ERR_GET_NUM_DEVS;
    }
    _numAccs = (size_t)xdmaDevices;

    //Preallocate accelerators array
    _accs = malloc(sizeof(acc_t)*_numAccs);
    if (_accs == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for accelerators info");
        goto INIT_ERR_ALLOC_ACCS;
    }

    //Generate the configuration file path
    char * buffer = getConfigFilePath();
    if (buffer == NULL) {
        ret = XTASKS_EFILE;
        printErrorMsgCfgFile();
        goto INIT_ERR_CFG_FILE;
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(buffer, "r");
    if (accMapFile == NULL) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open FPGA configuration file");
        free(buffer);
        goto INIT_ERR_CFG_FILE;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("First line of FPGA configuration file is not valid");
        free(buffer);
        fclose(accMapFile);
        goto INIT_ERR_CFG_FILE;
    }
    xtasks_acc_type t;
    int retFscanf;
    float freq;
    size_t num, total;
    total = 0;
    while ((retFscanf = fscanf(accMapFile, "%u %zu %s %f", &t, &num, buffer, &freq)) == 4) { //< Parse the file
    //while (fgets(buffer, STR_BUFFER_SIZE, accMapFile)) {
        total += num;
        for (size_t i = total - num; i < total && i < _numAccs; ++i) {
            _accs[i].info.id = i;
            _accs[i].info.type = t;
            _accs[i].info.freq = freq;
            _accs[i].info.maxTasks = -1;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
        }
    }
    fclose(accMapFile);
    free(buffer);

    if (retFscanf != EOF && retFscanf != 0) {
        //Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    }

    //Update number of accelerators
    if (total > _numAccs) {
        fprintf(stderr, "WARN: xTasks configuration file contains %zu ", total);
        fprintf(stderr, "accelerators, but only %zu FPGA devices have been found\n", _numAccs);
        fprintf(stderr, "      Using only the first accelerators of configuration file\n");
    }
    _numAccs = (total < _numAccs) ? total : _numAccs;

    //Get device and channel handles from libxdma for each accelerator
    xdma_device * devs = (xdma_device *)alloca(sizeof(xdma_device)*_numAccs);
    int numAccs = _numAccs;
    if (xdmaGetDevices(numAccs, &devs[0], &xdmaDevices) != XDMA_SUCCESS || xdmaDevices != numAccs) {
        ret = XTASKS_EFILE;
        goto INIT_ERR_DET_DEVS;
    }
    for (size_t i = 0; i < _numAccs; ++i) {
        _accs[i].xdmaDev = devs[i];
        xdmaGetDeviceChannel(devs[i], XDMA_TO_DEVICE, &_accs[i].inChannel);
        xdmaGetDeviceChannel(devs[i], XDMA_FROM_DEVICE, &_accs[i].outChannel);

        //Init the tasks queue
        _accs[i].tasksQueue = queueInit();
        _accs[i].tasksQueueLock = 0;
    }

    //Allocate tasks array
    _tasks = (task_t *)malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto INIT_ERR_ALLOC_TASKS;
    }

    //Allocate the tasks buffer
    s = xdmaAllocateHost((void**)&_cmdExecTaskBuff, &_cmdExecTaskBuffHandle, NUM_RUN_TASKS*DEF_EXEC_TASK_SIZE);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate kernel memory for execute task commands");
        goto INIT_ERR_ALLOC_EXEC_TASKS_BUFF;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].accel = NULL;
        _tasks[idx].cmdHeader = (cmd_exec_task_header_t *)&_cmdExecTaskBuff[idx*DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)
            &_cmdExecTaskBuff[idx*DEF_EXEC_TASK_SIZE + sizeof(cmd_exec_task_header_t)];
        _tasks[idx].taskHandle = 0;
    }

    return XTASKS_SUCCESS;

    //Error handling code
        xdmaFree(_cmdExecTaskBuffHandle);
    INIT_ERR_ALLOC_EXEC_TASKS_BUFF:
        free(_tasks);
    INIT_ERR_ALLOC_TASKS:
    INIT_ERR_DET_DEVS:
        _cmdExecTaskBuff = NULL;
    INIT_ERR_CFG_FILE:
        free(_accs);
    INIT_ERR_ALLOC_ACCS:
    INIT_ERR_GET_NUM_DEVS:
        _numAccs = 0;
        xdmaClose();
    INIT_ERR_XDMA_OPEN:
        xdmaFiniMem();
    INIT_ERR_INIT_MEM:
    INIT_ERR_CHECK_FEAT:
        __sync_sub_and_fetch(&_init_cnt, 1);
    return ret;
}

xtasks_stat xtasksFini()
{
    //Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    //Free execute task command buffer
    xdma_status s = xdmaFree(_cmdExecTaskBuffHandle);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }
    _cmdExecTaskBuff = NULL;

    //Free tasks array
    free(_tasks);

    //Finialize HW instrumentation
    if (xtasksFiniHWIns() != XTASKS_SUCCESS) {
        return XTASKS_ERROR;
    }

    //Finialize accelerators' structures
    for (size_t i = 0; i < _numAccs; ++i) {
        queueFini(_accs[i].tasksQueue);
    }

    //Free the accelerators array
    _numAccs = 0;
    free(_accs);
    if (xdmaClose() != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetPlatform(const char ** name)
{
    if (name == NULL) return XTASKS_EINVAL;

    *name = _platformName;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetBackend(const char ** name)
{
    if (name == NULL) return XTASKS_EINVAL;

    *name = _backendName;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(size_t * count)
{
    if (count == NULL) return XTASKS_EINVAL;

    *count = _numAccs;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccs(size_t const maxCount, xtasks_acc_handle * array, size_t * count)
{
    if (array == NULL || count == NULL) return XTASKS_EINVAL;

    size_t tmp = maxCount > _numAccs ? _numAccs : maxCount;
    for (size_t i = 0; i < tmp; ++i) {
        array[i] = (xtasks_acc_handle)(&_accs[i]);
    }
    *count = tmp;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info * info)
{
    if (info == NULL) return XTASKS_EINVAL;

    acc_t * ptr = (acc_t *)handle;
    *info = ptr->info;

    return XTASKS_SUCCESS;
}

static int getFreeTaskEntry(acc_t * accel)
{
    for (int i = 0; i < NUM_RUN_TASKS; ++i) {
        if (_tasks[i].accel == NULL) {
            if (__sync_bool_compare_and_swap(&_tasks[i].accel, 0, accel)) {
                return i;
            }
        }
    }
    return -1;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
    acc_t * accel = (acc_t *)accId;
    int idx = getFreeTaskEntry(accel);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].argsCnt = 0;
    //_tasks[idx].accel = accel; //NOTE: Done in getFreeTaskEntry
    _tasks[idx].cmdExecTx = 0;
    _tasks[idx].syncTx = 0;

    cmd_exec_task_header_t cmdHeader;
    cmdHeader.header.commandCode = CMD_EXEC_TASK_CODE;
    //cmdHeader.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0; //NOTE: Done in xtasksSubmitTask
    cmdHeader.header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader.header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_PS;
    cmdHeader.parentID = (uintptr_t)(parent);
    cmdHeader.taskID = (uintptr_t)(&_tasks[idx]);
    memcpy(_tasks[idx].cmdHeader, &cmdHeader, sizeof(cmd_exec_task_header_t));

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps,
    unsigned int const period, xtasks_task_handle * handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    task_t * task = (task_t *)(*handle);

    *handle = NULL;
    task->accel = NULL;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    if (1 > DEF_EXEC_TASK_ARGS_LEN - task->argsCnt) {
        //TODO: Allocate a new chunk for the hw_task_t struct
        return XTASKS_ENOMEM;
    }

    cmd_exec_task_arg_t arg;
    arg.flags = flags;
    arg.id = task->argsCnt++;
    arg.value = value;
    memcpy(task->cmdExecArgs + arg.id, &arg, sizeof(cmd_exec_task_arg_t));

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    if (num > DEF_EXEC_TASK_ARGS_LEN - task->argsCnt) {
        //TODO: Allocate a new chunk for the hw_task_t struct
        return XTASKS_ENOMEM;
    }

    cmd_exec_task_arg_t args[num];
    for (size_t i = 0, idx = task->argsCnt; i < num; ++i, ++idx) {
        args[i].flags = flags;
        args[i].id = idx;
        args[i].value = values[i];
    }
    memcpy(task->cmdExecArgs + task->argsCnt, args, num*sizeof(cmd_exec_task_arg_t));
    task->argsCnt += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);

    //Update execute command with the number of args information
    task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = task->argsCnt;

    size_t const size = sizeof(cmd_exec_task_header_t) + task->argsCnt*sizeof(cmd_exec_task_arg_t);
    size_t const descOffset = (uintptr_t)(task->cmdHeader) - (uintptr_t)(_cmdExecTaskBuff);
    xdma_status retD, retS;
    retD = xdmaStreamAsync(_cmdExecTaskBuffHandle, size, descOffset,
           task->accel->xdmaDev, task->accel->inChannel, &task->cmdExecTx);
    retS = xdmaStreamAsync(_cmdExecTaskBuffHandle, sizeof(cmd_out_exec_task_t) + sizeof(uint64_t)/*parentId*/,
           descOffset, task->accel->xdmaDev, task->accel->outChannel, &task->syncTx);
    queuePush(task->accel->tasksQueue, (void *)task);

    return (retD == XDMA_SUCCESS && retS == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

static xtasks_stat xtasksWaitTaskInternal(task_t * task)
{
    xdma_status retD, retS;
    retD = xdmaWaitTransfer(&task->cmdExecTx);
    retS = xdmaWaitTransfer(&task->syncTx);

#ifdef XTASKS_DEBUG
    if (task->cmdHeader->header.commandCode != CMD_FINI_EXEC_CODE ||
        task->cmdHeader->header.commandArgs[CMD_FINI_EXEC_ARGS_ACCID_OFFSET] != task->accel->info.id ||
        task->cmdHeader->taskID != (uintptr_t)task)
    {
        PRINT_ERROR("Unexpected data received from accelerator in xtasksWaitTaskInternal");
        return XTASKS_ERROR;
    }
#endif /* XTASKS_DEBUG */

    return (retD == XDMA_SUCCESS && retS == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);

    if (task != (task_t *)queueFront(task->accel->tasksQueue)) {
        //Only waiting for the first task of the queue is supported
        return XTASKS_PENDING;
    } else if (task != (task_t *)queueTryPop(task->accel->tasksQueue)) {
        //Some other thread stoled the task from the queue (in previous check tasks matched)
        return XTASKS_ERROR;
    }

    return xtasksWaitTaskInternal(task);
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle * handle, xtasks_task_id * id)
{
    xtasks_stat ret = XTASKS_PENDING;
    for (size_t i = 0; i < _numAccs && ret == XTASKS_PENDING; ++i) {
        ret = xtasksTryGetFinishedTaskAccel(&_accs[i], handle, id);
    }
    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel,
    xtasks_task_handle * task, xtasks_task_id * id)
{
    acc_t * acc = (acc_t *)accel;

    xtasks_stat ret = XTASKS_PENDING;
#if 0
    task_t * t = NULL;
    if (acc->tasksQueueLock == 0 && __sync_lock_test_and_set(&acc->tasksQueueLock, 1) == 0) {
        //NOTE: Only pop the task if finished -> atomically get the front, test and ¿pop?
        t = (task_t *)queueFront(acc->tasksQueue);
        if (t != NULL) {
            ret = xtasksWaitTaskInternal(t); //xtasksTryWaitTask(t);
            if (ret != XTASKS_PENDING) {
                //NOTE: Remove the task from the queue if it has finished or an error ocurred
                queuePop(acc->tasksQueue);
                *task = (xtasks_task_handle)t;
                *id = (xtasks_task_id)t->id;
            }
        }
        __sync_lock_release(&acc->tasksQueueLock);
    }
#else
    task_t * t = (task_t *)queueTryPop(acc->tasksQueue);
    if (t != NULL) {
        ret = xtasksWaitTaskInternal(t);
        *task = (xtasks_task_handle)t;
        *id = (xtasks_task_id)t->id;
    }
#endif
    return ret;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event *events, size_t const maxCount)
{
    acc_t * acc = (acc_t *)(accel);
    xtasks_ins_event * accBuffer = _instrBuff + acc->info.id*_numInstrEvents;
    size_t count, i;

    if (events == NULL || (acc - _accs) >= _numAccs || maxCount <= 0) return XTASKS_EINVAL;
    else if (_instrBuff == NULL) return XTASKS_ENOAV;

    if (__sync_lock_test_and_set(&acc->instrLock, 1)) {
        //There is another thread reading the buffer for this accelerator
        events->eventType = XTASKS_EVENT_TYPE_INVALID;
    } else {
        count = min(maxCount, _numInstrEvents - acc->instrIdx);
        accBuffer += acc->instrIdx;
        memcpy(events, accBuffer, count*sizeof(xtasks_ins_event));

        i = 0;
        while (i < count && events[i].eventType != XTASKS_EVENT_TYPE_INVALID)
        {
            //Invalidate all read entries in the accelerator buffer
            accBuffer[i].eventType = XTASKS_EVENT_TYPE_INVALID;
            i++;
        }
        acc->instrIdx = (acc->instrIdx + i)%_numInstrEvents;
        __sync_lock_release(&acc->instrLock);

        if (i < maxCount) {
            //Ensure invalid type of first non-wrote event slot in the caller buffer
            events[i].eventType = XTASKS_EVENT_TYPE_INVALID;
        }
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask ** task)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp * timestamp)
{
    if (timestamp == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaGetDeviceTime(timestamp);
    return toXtasksStat(status);
}

xtasks_stat xtasksMalloc(size_t len, xtasks_mem_handle * handle)
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

xtasks_stat xtasksGetAccAddress(xtasks_mem_handle const handle, uint64_t * addr)
{
    if (addr == NULL) return XTASKS_EINVAL;

    unsigned long devAddr = 0;
    xdma_status status = xdmaGetDeviceAddress(handle, &devAddr);
    *addr = devAddr;
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpy(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
    xtasks_memcpy_kind const kind)
{
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpy(usr, handle, len, offset, mode);
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpyAsync(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
    xtasks_memcpy_kind const kind, xtasks_memcpy_handle * cpyHandle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpyAsync(usr, handle, len, offset, mode, cpyHandle);
    return toXtasksStat(status);
}

xtasks_stat xtasksTestCopy(xtasks_memcpy_handle * handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaTestTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksSyncCopy(xtasks_memcpy_handle * handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaWaitTransfer(handle);
    return toXtasksStat(status);
}

#ifdef XTASKS_DEBUG
uint64_t cmdExecTaskBuff(size_t const taskIdx, size_t const wordIdx)
{
    uint64_t * data = (uint64_t *)(_cmdExecTaskBuff + taskIdx*DEF_EXEC_TASK_SIZE);
    return data[wordIdx];
}
#endif /* XTASKS_DEBUG */
