/*
* Copyright (c) 2017, BSC (Barcelona Supercomputing Center)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the <organization> nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY BSC ''AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "libxtasks.h"

#include <libxdma.h>
#include <sys/auxv.h>
#include <elf.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>

#define STR_BUFFER_SIZE     128
#define HW_TASK_HEAD_BYTES  24              ///< Size of hw_task_t without the args field
#define HW_TASK_NUM_ARGS    14              ///< (256 - TASK_INFO_HEAD_BYTES)/sizeof(xdma_task_arg)
#define INS_BUFFER_SIZE     8192            ///< 2 pages
#define INS_NUM_ENTRIES  (INS_BUFFER_SIZE/sizeof(xtasks_ins_times))
#define NUM_RUN_TASKS       INS_NUM_ENTRIES ///< Maximum number of concurrently running tasks
#define HW_TASK_DEST_ID_PS  0x0000001F      ///< Processing System identifier for the destId field

typedef struct {
    xdma_device     xdmaDev;
    xdma_channel    inChannel;
    xdma_channel    outChannel;
    char            descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
} acc_t;

typedef struct {
    xtasks_arg_flags  cacheFlags;          ///< Flags
    xtasks_arg_id     paramId;             ///< Argument ID
    xtasks_arg_val    address;             ///< Address
} hw_arg_t;

typedef struct __attribute__ ((__packed__)) {
    struct {
        uint64_t timer;                    ///< Timer address for instrumentation
        uint64_t buffer;                   ///< Buffer address to store instrumentation info
    } profile;
    xtasks_comp_flags compute;             ///< Compute flag
    uint32_t destID;                       ///< Where the accelerator will send the comp. signal
    hw_arg_t args[HW_TASK_NUM_ARGS];       ///< Task arguments info
} hw_task_t;

typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    hw_task_t *             hwTask;        ///< Pointer to the hw_task_t struct
    size_t                  argsCnt;       ///< Number of arguments in the task
    acc_t *                 accel;         ///< Accelerator where the task will run
    xdma_transfer_handle    descriptorTx;  ///< Task descriptor transfer handle
    xdma_transfer_handle    syncTx;        ///< Task sync transfer handle
    xdma_buf_handle         taskHandle;    ///< Task buffer handle (only used if: argsCnt > HW_TASK_NUM_ARGS)
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint64_t             _insTimerAddr;      ///< Physical address of HW instrumentation timer
static xtasks_ins_times *   _insBuff;           ///< Buffer for the HW instrumentation
static xtasks_ins_times *   _insBuffPhy;        ///< Physical address of _insBuff
static xdma_buf_handle      _insBuffHandle;     ///< Handle of _insBuff in libxdma
static hw_task_t *          _tasksBuff;         ///< Buffer to send the HW tasks
static hw_task_t *          _tasksBuffPhy;      ///< Physical address of _tasksBuff
static xdma_buf_handle      _tasksBuffHandle;   ///< Handle of _tasksBuff in libxdma
static task_t *             _tasks;             ///< Array with internal task information

static void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library requires reading a '.xtasks.config' file to obtain the");
    fprintf(stderr, "current FPGA configuration\n");
    fprintf(stderr, "       However, XTASKS_CONFIG_FILE environment variable was not defined and");
    fprintf(stderr, "library was not able to build the filename based on the application binary\n");
}

static xtasks_stat initHWIns()
{
    //allocate instrumentation buffer & get its physical address
    xdma_status s;
    s = xdmaAllocateKernelBuffer((void**)&_insBuff, &_insBuffHandle, INS_BUFFER_SIZE);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }
    unsigned long phyAddr;
    s = xdmaGetDMAAddress(_insBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        xdmaFreeKernelBuffer((void *)_insBuff, _insBuffHandle);
        _insBuff = NULL;
        return XTASKS_ERROR;
    }
    _insBuffPhy = (xtasks_ins_times *)phyAddr;
    _insTimerAddr = (uint64_t)xdmaGetInstrumentationTimerAddr();
    return XDMA_SUCCESS;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    //Open libxdma
    if (xdmaOpen() != XDMA_SUCCESS) {
        err_0: __sync_sub_and_fetch(&_init_cnt, 1);
        return XTASKS_ERROR;
    }

    //Get the number of devices from libxdma
    int xdmaDevices;
    if (xdmaGetNumDevices(&xdmaDevices) != XDMA_SUCCESS) {
        err_1: _numAccs = 0;
        xdmaClose();
        goto err_0;
    }
    _numAccs = (size_t)xdmaDevices;

    //Preallocate accelerators array
    _accs = malloc(sizeof(acc_t)*_numAccs);

    //Generate the configuration file path
    char * buffer = malloc(sizeof(char)*STR_BUFFER_SIZE);
    const char * accMapPath = getenv("XTASKS_CONFIG_FILE"); //< 1st, environment var
    if (accMapPath == NULL) {
        accMapPath = (const char *)getauxval(AT_EXECFN); //< Get executable file path
        if ((strlen(accMapPath) + 14) > STR_BUFFER_SIZE) {
            free(buffer);
            buffer = malloc(sizeof(char)*(strlen(accMapPath) + 14));
        }
        strcpy(buffer, accMapPath);
        accMapPath = strcat(buffer, ".xtasks.config"); //< 2nd, exec file
        if (access(accMapPath, R_OK) == -1) {
            accMapPath = strrchr(accMapPath, '/'); //< 3rd, exec file in current dir
            if (accMapPath == NULL) {
                printErrorMsgCfgFile();
                err_2: free(buffer);
                free(_accs);
                goto err_1;
            }
            accMapPath++;
        }
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(accMapPath, "r");
    if (accMapFile == NULL) {
        fprintf(stderr, "ERROR: Cannot open file %s to read current FPGA configuration\n", accMapPath);
        goto err_2;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        goto err_2;
    }
    xtasks_acc_type t;
    size_t num, total;
    total = 0;
    while (fscanf(accMapFile, "%u %u %s", &t, &num, buffer) == 3) { //< Parse the file
    //while (fgets(buffer, STR_BUFFER_SIZE, accMapFile)) {
        total += num;
        for (size_t i = total - num; i < total && total <= _numAccs; ++i) {
            _accs[i].info.id = i;
            _accs[i].info.type = t;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
        }
    }
    fclose(accMapFile);
    free(buffer);

    //Update number of accelerators
    if (total > _numAccs) {
        fprintf(stderr, "WARN: xTasks configuration file contains %u ", total);
        fprintf(stderr, "accelerators, but only %u FPGA devices have been found\n", _numAccs);
        fprintf(stderr, "      Using only the first accelerators of configuration file\n");
    }
    _numAccs = (total < _numAccs) ? total : _numAccs;

    //Get device and channel handles from libxdma for each accelerator
    xdma_device devs[_numAccs];
    int numAccs = _numAccs;
    if (xdmaGetDevices(numAccs, &devs[0], &xdmaDevices) != XDMA_SUCCESS || xdmaDevices != numAccs) {
        goto err_1;
    }
    for (size_t i = 0; i < _numAccs; ++i) {
        _accs[i].xdmaDev = devs[i];
        xdmaGetDeviceChannel(devs[i], XDMA_TO_DEVICE, &_accs[i].inChannel);
        xdmaGetDeviceChannel(devs[i], XDMA_FROM_DEVICE, &_accs[i].outChannel);
    }

    //Init HW instrumentation
    if (initHWIns() != XTASKS_SUCCESS) {
        goto err_2;
    }

    //Allocate the tasks buffer
    xdma_status s;
    s = xdmaAllocateKernelBuffer((void**)&_tasksBuff, &_tasksBuffHandle, NUM_RUN_TASKS*sizeof(task_t));
    if (s != XDMA_SUCCESS) {
        err_3: _tasksBuff = NULL;
        _tasksBuffPhy = NULL;
        goto err_2;
    }
    unsigned long phyAddr;
    s = xdmaGetDMAAddress(_tasksBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        err_4: xdmaFreeKernelBuffer((void *)_tasksBuff, _tasksBuffHandle);
        goto err_3;
    }
    _tasksBuffPhy = (hw_task_t *)phyAddr;

    //Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        goto err_4;
    }
    memset(_tasks, 0, NUM_RUN_TASKS*sizeof(task_t));

    return XTASKS_SUCCESS;
}

static xtasks_stat finiHWIns()
{
    xdma_status s = xdmaFreeKernelBuffer((void *)_insBuff, _insBuffHandle);
    _insBuff = NULL;
    _insBuffPhy = NULL;
    return s == XDMA_SUCCESS ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksFini()
{
    //Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    //Free tasks array
    free(_tasks);

    //Free tasks buffer
    xdma_status s = xdmaFreeKernelBuffer((void *)_tasksBuff, _tasksBuffHandle);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }
    _tasksBuff = NULL;
    _tasksBuffPhy = NULL;

    //Finialize HW instrumentation
    if (finiHWIns() != XTASKS_SUCCESS) {
        return XTASKS_ERROR;
    }

    _numAccs = 0;
    free(_accs);
    if (xdmaClose() != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }

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
    xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
    acc_t * accel = (acc_t *)accId;
    int idx = getFreeTaskEntry(accel);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].hwTask = &_tasksBuff[idx];
    _tasks[idx].argsCnt = 0;
    //_tasks[idx].accel = accel; //NOTE: Done in getFreeTaskEntry()
    _tasks[idx].hwTask->profile.timer = _insTimerAddr;
    _tasks[idx].hwTask->profile.buffer = (uintptr_t)(&_insBuffPhy[idx]);
    _tasks[idx].hwTask->compute = compute;
    _tasks[idx].hwTask->destID = HW_TASK_DEST_ID_PS;

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
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
    if (1 > HW_TASK_NUM_ARGS - task->argsCnt) {
        //TODO: Allocate a new chunk for the hw_task_t struct
        return XTASKS_ENOMEM;
    }

    size_t idx = task->argsCnt++;
    task->hwTask->args[idx].cacheFlags = flags;
    task->hwTask->args[idx].paramId = idx;
    task->hwTask->args[idx].address = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    if (num > HW_TASK_NUM_ARGS - task->argsCnt) {
        //TODO: Allocate a new chunk for the hw_task_t struct
        return XTASKS_ENOMEM;
    }

    for (size_t i = 0, idx = task->argsCnt; i < num; ++i, ++idx) {
        task->hwTask->args[idx].cacheFlags = flags;
        task->hwTask->args[idx].paramId = idx;
        task->hwTask->args[idx].address = values[i];
    }
    task->argsCnt += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);

    size_t size = HW_TASK_HEAD_BYTES + task->argsCnt*sizeof(hw_arg_t);
    unsigned int descOffset = (uintptr_t)(task->hwTask) - (uintptr_t)(_tasksBuff);
    xdma_status retD, retS;
    retD = xdmaSubmitKBuffer(_tasksBuffHandle, size, descOffset,
           XDMA_ASYNC, task->accel->xdmaDev, task->accel->inChannel, &task->descriptorTx);
    retS = xdmaSubmitKBuffer(_tasksBuffHandle, sizeof(uint64_t),
           descOffset + offsetof(hw_task_t, compute),
           XDMA_ASYNC, task->accel->xdmaDev, task->accel->outChannel, &task->syncTx);

    return (retD == XDMA_SUCCESS && retS == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);

    xdma_status retD, retS;
    retD = xdmaWaitTransfer(task->descriptorTx);
    retS = xdmaWaitTransfer(task->syncTx);

    return (retD == XDMA_SUCCESS && retS == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_id * id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_task_handle const handle, xtasks_ins_times ** times)
{
    task_t * task = (task_t *)(handle);
    size_t idx = task - _tasks;

    if (times == NULL || idx >= NUM_RUN_TASKS) return XTASKS_EINVAL;

    *times = &_insBuff[idx];

    return XTASKS_SUCCESS;
}
