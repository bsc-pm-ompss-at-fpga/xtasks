/*
* Copyright (c) 2018, BSC (Barcelona Supercomputing Center)
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
#include "util/common.h"
#include "util/queue.h"

#include <libpicos.h>
#include <elf.h>
#include <stdio.h>
#include <stddef.h>

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array
#define MAX_CNT_FIN_TASKS       16              ///< Max. number of finished tasks processed for other accels before return
#define NUM_RUN_TASKS           256             ///< Maximum number of concurrently running tasks
#define NUM_DEPS_EXEC_MASK      0xFFFF          ///< Mask used to set numDeps field and mark task as executed

//! \brief HW accelerator representation
typedef struct {
    uint16_t        picosArchMask;
    char            descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
    queue_t *       finishedQueue;
} acc_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    acc_t *                 accel;         ///< Accelerator where the task will run
    __attribute__((aligned(8))) picos_task
                            picosTask;     ///< Picos task representation
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static task_t * _tasks;     ///< Array with internal task information

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    //Initialize libpicos
    if (picosInitializeDef() != PICOS_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("picosInitialize failed");
        return ret;
    }

    //Preallocate accelerators array
    _numAccs = DEF_ACCS_LEN;
    _accs = malloc(sizeof(acc_t)*_numAccs);
    if (_accs == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for accelerators info");
        INIT_ERR_0: picosShutdown();
        return ret;
    }

    //Generate the configuration file path
    char * buffer = getConfigFilePath();
    if (buffer == NULL) {
        ret = XTASKS_EFILE;
        printErrorMsgCfgFile();
        INIT_ERR_1: free(_accs);
        _numAccs = 0;
        goto INIT_ERR_0;
    }

    //Check if architecture mask should be forced
    uint16_t forcedMask = 0;
    const char * envMask = getenv("XTASKS_PICOS_ARCHMASK");
    if (envMask != NULL) {
        if (envMask[0] == '0' && envMask[1] != '\0' && (envMask[1] == 'x' || envMask[1] == 'X')) {
            //The mask is a hexadecimal number
            forcedMask = strtoul(envMask, NULL, 16);
        } else {
            //The mask seems to be a decimal number
            forcedMask = atoi(envMask);
        }
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(buffer, "r");
    if (accMapFile == NULL) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open FPGA configuration file");
        INIT_ERR_2: free(buffer);
        goto INIT_ERR_1;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("First line of FPGA configuration file is not valid");
        fclose(accMapFile);
        goto INIT_ERR_2;
    }
    xtasks_acc_type t;
    int retFscanf;
    float freq;
    size_t num, total;
    total = 0;
    while ((retFscanf = fscanf(accMapFile, "%u %zu %s %f", &t, &num, buffer, &freq)) == 4) { //< Parse the file
        //while (fgets(buffer, STR_BUFFER_SIZE, accMapFile)) {
        total += num;
        if (total > _numAccs) {
            //_accs array is not big enough -> double its capacity
            acc_t * oldAccs = _accs;
            _numAccs = 2*_numAccs;
            _accs = malloc(sizeof(acc_t)*_numAccs);
            memcpy(_accs, oldAccs, sizeof(acc_t)*(total - num));
            free(oldAccs);
        }
        uint16_t mask = forcedMask ? forcedMask : ((1 << num) - 1) << (15 - total);
        for (size_t i = total - num; i < total; ++i) {
            //NOTE: Using the same mask for all accels of same type to allow picos balance execution.
            //      This can be an issue if the queues to retrieve tasks are different for each accelerator
            //      because the library is breaking the request of executing a task in one accel.
            //      Therefore, the task may become finished in a different accel!
            //_accs[i].picosArchMask = 1 << (15 - i); //NOTE: ACC0 sets bit15, ACC1 sets bit14, etc.
            _accs[i].picosArchMask = mask;
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
    _numAccs = (total < _numAccs) ? total : _numAccs;

    if (retFscanf != EOF) {
        //Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    }

    //Init accelerators' structures
    for (size_t i = 0; i < _numAccs; ++i) {
        _accs[i].finishedQueue = queueInit();
    }

    //Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate kernel memory for task information");
        goto INIT_ERR_1;
    }
    memset(_tasks, 0, NUM_RUN_TASKS*sizeof(task_t));

    return ret;
}

xtasks_stat xtasksFini()
{
    //Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    //Free tasks array
    free(_tasks);

    //Finialize accelerators' structures
    for (size_t i = 0; i < _numAccs; ++i) {
        queueFini(_accs[i].finishedQueue);
    }

    //Free the accelerators array
    _numAccs = 0;
    free(_accs);

    //Finialize libpicos
    if (picosShutdown() != PICOS_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    return ret;
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

static int getFreeTaskEntry()
{
    int ini = rand()%NUM_RUN_TASKS;
    for (int i = (ini+1)%NUM_RUN_TASKS; i != ini; i = (i+1)%NUM_RUN_TASKS) {
        if (_tasks[i].picosTask.taskID == 0) {
            uint64_t taskID = (uintptr_t)&_tasks[i];
            if (__sync_bool_compare_and_swap(&_tasks[i].picosTask.taskID, 0, taskID)) {
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
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].accel = accel;
    //_tasks[idx].picosTask.taskID = (uintptr_t)&_tasks[i]; //NOTE: Done in getFreeTaskEntry()
    //NOTE: Set the upper bit of taskID as the 32 high bits cannot be 0x00000000
    //      We are assuming that in a 64bit architecture the highest bit will not be used
    _tasks[idx].picosTask.taskID |= 0x8000000000000000;
    _tasks[idx].picosTask.numDeps = 0;
    _tasks[idx].picosTask.archMask = accel->picosArchMask;

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    task_t * task = (task_t *)(*handle);

    *handle = NULL;
    task->picosTask.taskID = 0;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    size_t idx = task->picosTask.numDeps;
    if (task->picosTask.numDeps >= PICOS_MAX_DEPS_TASK) {
        //NOTE: Picos cannot handle the needed amount of dependencies
        return XTASKS_ERROR;
    }

    task->picosTask.numDeps++;
    task->picosTask.deps[idx].address = value;
    task->picosTask.deps[idx].direction = PICOS_INOUT; //NOTE: Just in case, mark argument as inout

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    size_t idx = task->picosTask.numDeps;
    if (num > PICOS_MAX_DEPS_TASK - idx) {
        //NOTE: Picos cannot handle the needed amount of dependencies
        return XTASKS_ERROR;
    }

    for (size_t i = 0; i < num; ++i, ++idx) {
        //NOTE: Picos expects the HIGH and LOW parts in the other way
        task->picosTask.deps[idx].address = values[i] << 32 | values[i] >> 32;
        task->picosTask.deps[idx].direction = PICOS_INOUT; //NOTE: Just in case, mark argument as inout
    }
    task->picosTask.numDeps += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    return picosSendExecTask(&task->picosTask) == PICOS_SUCCESS ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    size_t tries = 0;
    size_t const MAX_WAIT_TASKS_TRIES = 16;

    while (task->picosTask.numDeps != NUM_DEPS_EXEC_MASK && tries++ < MAX_WAIT_TASKS_TRIES) {
        xtasks_task_id id;
        xtasks_task_handle h;
        xtasks_stat s = xtasksTryGetFinishedTask(&h, &id);
        if (s != XTASKS_SUCCESS && s != XTASKS_PENDING) {
            return XTASKS_ERROR;
        }
    }
    return tries > MAX_WAIT_TASKS_TRIES ? XTASKS_PENDING : XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle * handle, xtasks_task_id * id)
{
    if (id == NULL || handle == NULL) {
        return XTASKS_EINVAL;
    }

    picos_task t;
    picos_status s = picosGetExecTask(&t);
    if (s == PICOS_SUCCESS) {
        uint64_t tId;
        picosGetTaskID(&t, &tId);
        //NOTE: Clear highest bit. See xtasksCreateTask()
        tId &= 0x7FFFFFFFFFFFFFFF;
        uintptr_t tPtr = (uintptr_t)tId;
        task_t * task = (task_t *)(tPtr);

        //Mark the task as executed (using the valid field as it is not used in the cached copy)
        task->picosTask.numDeps = NUM_DEPS_EXEC_MASK;

        *id = task->id;
        *handle = (xtasks_task_handle)task;
        return XTASKS_SUCCESS;
    } else if (s == PICOS_BUSY) {
        return XTASKS_PENDING;
    }

    return XTASKS_ERROR;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel,
    xtasks_task_handle * task, xtasks_task_id * id)
{
    acc_t * acc = (acc_t *)accel;

    //1st: Try get a finished task from the accel queue
    task_t * t = (task_t *)queueTryPop(acc->finishedQueue);
    if (t == NULL) {
        unsigned int cnt = 0;
        xtasks_task_handle xTask;
        xtasks_task_id xId;
        //2nd: Try to fetch a task from the finished queue
        while (cnt < MAX_CNT_FIN_TASKS && xtasksTryGetFinishedTask(&xTask, &xId) == XTASKS_SUCCESS) {
            ++cnt;
            t = (task_t *)xTask;
            if (t->accel == acc) {
                //The task was executed in the accelerator
                *task = xTask;
                *id = xId;
                break;
            } else {
                //The task was not executed in the accelerator
                queuePush(t->accel->finishedQueue, (void *)t);
                t = NULL;
            }
        }
    } else {
        *task = (xtasks_task_handle)t;
        *id = (xtasks_task_id)t->id;
    }

    return t == NULL ? XTASKS_PENDING : XTASKS_SUCCESS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_task_handle const handle, xtasks_ins_times ** times)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp * timestamp)
{
    return XTASKS_ENOSYS;
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

    xdma_status status = xdmaGetDeviceAddress(handle, addr);
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
