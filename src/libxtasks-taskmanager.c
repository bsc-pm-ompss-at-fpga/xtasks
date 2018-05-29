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

#include <libxdma.h>
#include <libxdma_version.h>
#include <elf.h>
#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <sys/mman.h>

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array

#define READY_QUEUE_PATH        "/dev/taskmanager/ready_queue"
#define FINI_QUEUE_PATH         "/dev/taskmanager/finished_queue"
#define ASYNC_RST_PATH          "/dev/taskmanager/ctrl"

#define READY_QUEUE_LEN         1024            ///< Total number of entries in the ready queue
#define READY_QUEUE_ACC_LEN     32              ///< Number of entries in the sub-queue of ready queue for one accelerator
#define FINI_QUEUE_LEN          1024            ///< Total number of entries in the finish queue
#define FINI_QUEUE_ACC_LEN      32              ///< Number of entries in the sub-queue of finish queue for one accelerator
#define VALID_ENTRY_MASK        0x80
#define FREE_ENTRY_MASK         0
#define HW_TASK_HEAD_BYTES      24              //NOTE: Actually 32 bytes, but taskID is not taken into account when computing taskSize
#define HW_TASK_NUM_ARGS        14              //NOTE: (256 - HW_TASK_HEAD_BYTES)/sizeof(task_info_arg_t)
//NOTE: The number of slots must be at least the number of slots in the ready queue (READY_QUEUE_LEN)
//      + the number of running tasks (1 foreach accelerator -> _numAccs)
#define NUM_RUN_TASKS           (READY_QUEUE_LEN + _numAccs) ///< Maximum number of concurrently running tasks
#define INS_BUFFER_SIZE         (NUM_RUN_TASKS*sizeof(xtasks_ins_times)) ///< Tasks in the readyQueue + tasks being executed in accelerators
#define HW_TASK_DEST_ID_PS      0x0000001F      ///< Processing System identifier for the destId field
#define HW_TASK_DEST_ID_TM      0x00000011      ///< Task manager identifier for the destId field

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 1
# error Installed libxdma is not supported (use >= 1.0)
#endif

//! \brief HW accelerator representation
typedef struct {
    char            descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
    unsigned short  readyQueueIdx;     ///< Writing index of the accelerator sub-queue in the ready queue
    unsigned short  finiQueueIdx;      ///< Reading index of the accelerator sub-queue in the finished queue
} acc_t;

//! \brief Ready task buffer element representation
typedef struct __attribute__ ((__packed__)) {
    uint64_t   taskInfoAddr;  //[0  :63 ] Physical address to the Task info structure
    uint16_t   argsBitmask;   //[64 :79 ] Bitmask defining if the arguments are ready or not
    uint16_t   _padding1;
    uint8_t    taskSize;      //[96 :103] Size of Task Info structure in words (of 64 bits)
    uint8_t    _padding2;
    uint8_t    accID;         //[112:119] Accelerator ID where the task will be sent
    uint8_t    valid;         //[120:127] Valid Entry
} ready_task_t;

//! \brief Finished task buffer representation
typedef struct __attribute__ ((__packed__)) {
    uint64_t  taskID;        //[0  :63 ] Task identifier sent to the ready queue
    uint32_t  accID;         //[64 :95 ] Accelerator ID that executed the task
    uint8_t   _padding[3];
    uint8_t   valid;         //[120:127] Valid Entry
} fini_task_t;

//! \brief Task argument representation in task_info
typedef struct __attribute__ ((__packed__)) {
    uint32_t argCached; //[0  :31 ] Flags
    uint32_t argID;     //[32 :63 ] Argument ID
    uint64_t argAddr;   //[64 :127] Address
} task_info_arg_t;

//! \brief Task information for the accelerator
typedef struct __attribute__ ((__packed__)) {
    uint64_t taskID;                         //[0  :63 ] Task identifier
    struct {
        uint64_t timerAddr;                  //[64 :127] Timer address for instrumentation
        uint64_t bufferAddr;                 //[128:191] Buffer address to store instrumentation info
    } profile;
    uint32_t compute;                        //[192:223] Compute flag
    uint32_t destID;                         //[224:255] Destination ID where the accelerator will send the 'complete' signal
    task_info_arg_t args[HW_TASK_NUM_ARGS];  //[   :   ] Task arguments info
} task_info_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    task_info_t *           hwTask;        ///< Pointer to the task_info_t struct
    size_t                  argsCnt;       ///< Number of arguments in the task
    acc_t *                 accel;         ///< Accelerator where the task will run
    ready_task_t            tmTask;        ///< Cached task information that will be sent to the TM
    xdma_buf_handle         taskHandle;    ///< Task buffer handle (only used if: argsCnt > HW_TASK_NUM_ARGS)
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint64_t             _insTimerAddr;      ///< Physical address of HW instrumentation timer
static xtasks_ins_times *   _insBuff;           ///< Buffer for the HW instrumentation
static xtasks_ins_times *   _insBuffPhy;        ///< Physical address of _insBuff
static xdma_buf_handle      _insBuffHandle;     ///< Handle of _insBuff in libxdma
static task_info_t *        _tasksBuff;         ///< Buffer to send the HW tasks
static task_info_t *        _tasksBuffPhy;      ///< Physical address of _tasksBuff
static xdma_buf_handle      _tasksBuffHandle;   ///< Handle of _tasksBuff in libxdma
static task_t *             _tasks;             ///< Array with internal task information
static int                  _readyQFd;          ///< File descriptior of gpioctrl device
static int                  _finiQFd;           ///< File descriptior of gpioctrl device
static int                  _ctrlFd;            ///< File descriptior of gpioctrl device
static ready_task_t        *_readyQueue;        ///< Buffer for the ready tasks
static fini_task_t         *_finiQueue;         ///< Buffer for the finished tasks
static uint32_t volatile   *_asyncRst;          ///< Register to reset indexes of _readyQueue and _finiQueue

static inline __attribute__((always_inline)) xtasks_stat resetQueueIdx()
{
    //Nudge one register
    *_asyncRst = 0x00;
    for ( int i = 0; i < 10; i++ ) i = i; // Lose some time
    *_asyncRst = 0x01;
    return XTASKS_SUCCESS;
}

static xtasks_stat initHWIns()
{
    //allocate instrumentation buffer & get its physical address
    xdma_status s;
    s = xdmaAllocateKernelBuffer((void**)&_insBuff, &_insBuffHandle, INS_BUFFER_SIZE);
    if (s != XDMA_SUCCESS) {
        PRINT_ERROR("Cannot allocate kernel buffer for instrumentation");
        return XTASKS_ERROR;
    }
    unsigned long phyAddr;
    s = xdmaGetDMAAddress(_insBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        PRINT_ERROR("Cannot get physical address of instrumentation buffer");
        xdmaFreeKernelBuffer((void *)_insBuff, _insBuffHandle);
        _insBuff = NULL;
        return XTASKS_ERROR;
    }
    _insBuffPhy = (xtasks_ins_times *)phyAddr;
    _insTimerAddr = 0;
    s = xdmaInitHWInstrumentation();
    if (s == XDMA_SUCCESS) {
        _insTimerAddr = (uint64_t)xdmaGetInstrumentationTimerAddr();
    }
    return XTASKS_SUCCESS;
}

static xtasks_stat finiHWIns()
{
    xdma_status s0 = XDMA_SUCCESS;
    if (_insTimerAddr != 0) {
        //xdmaInitHWInstrumentation was succesfully executed
        s0 = xdmaFiniHWInstrumentation();
    }
    xdma_status s1 = xdmaFreeKernelBuffer((void *)_insBuff, _insBuffHandle);
    _insBuff = NULL;
    _insBuffPhy = NULL;
    return (s0 == XDMA_SUCCESS && s1 == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    //Open libxdma
    if (xdmaOpen() != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("xdmaOpen failed");
        INIT_ERR_0: __sync_sub_and_fetch(&_init_cnt, 1);
        return ret;
    }

    //Preallocate accelerators array
    _numAccs = DEF_ACCS_LEN;
    _accs = malloc(sizeof(acc_t)*_numAccs);
    if (_accs == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for accelerators info");
        goto INIT_ERR_0;
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
        for (size_t i = total - num; i < total; ++i) {
            _accs[i].info.id = i;
            _accs[i].info.type = t;
            _accs[i].info.freq = freq;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
            _accs[i].readyQueueIdx = 0;
            _accs[i].finiQueueIdx = 0;
        }
    }
    fclose(accMapFile);
    free(buffer);
    _numAccs = (total < _numAccs) ? total : _numAccs;

    if (retFscanf != EOF) {
        //Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    } else if (_numAccs > READY_QUEUE_LEN/READY_QUEUE_ACC_LEN) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("The maximum number of accelerators supported by the library was reached");
        goto INIT_ERR_1;
    }

    //Open and map the Task Manager queues into library memory
    _readyQFd = open(READY_QUEUE_PATH, O_RDWR, (mode_t) 0600);
    _finiQFd = open(FINI_QUEUE_PATH, O_RDWR, (mode_t) 0600);
    _ctrlFd = open(ASYNC_RST_PATH, O_RDWR, (mode_t) 0600);
    if (_readyQFd < 0 || _finiQFd < 0 || _ctrlFd < 0) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open taskmanager device files");
        close(_readyQFd);
        close(_finiQFd);
        close(_ctrlFd);
        goto INIT_ERR_1;
    }

    _readyQueue = (ready_task_t *)mmap(NULL, sizeof(ready_task_t)*READY_QUEUE_LEN,
        PROT_READ | PROT_WRITE, MAP_SHARED, _readyQFd, 0);
    if (_readyQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map ready queue of Task Manager");
        INIT_ERR_3:
        close(_readyQFd);
        close(_finiQFd);
        close(_ctrlFd);
        goto INIT_ERR_1;
    }

    //Ensure that readyQueue is empty
    for (size_t idx = 0; idx < READY_QUEUE_LEN; ++idx) {
        _readyQueue[idx].valid = FREE_ENTRY_MASK;
    }

    _finiQueue = (fini_task_t *)mmap(NULL, sizeof(fini_task_t)*FINI_QUEUE_LEN,
        PROT_READ | PROT_WRITE, MAP_SHARED, _finiQFd, 0);
    if (_finiQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map finish queue of Task Manager");
        INIT_ERR_4: munmap(_readyQueue, sizeof(ready_task_t)*READY_QUEUE_LEN);
        goto INIT_ERR_3;
    }

    //If any, invalidate finished tasks in finiQueue
    for (size_t idx = 0; idx < FINI_QUEUE_LEN; ++idx) {
        _finiQueue[idx].valid = FREE_ENTRY_MASK;
    }

    _asyncRst = (uint32_t *)mmap(NULL, sizeof(uint32_t), PROT_READ | PROT_WRITE,
        MAP_SHARED, _ctrlFd, 0);
    if (_asyncRst == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map control registers of Task Manager");
        INIT_ERR_5: munmap(_finiQueue, sizeof(fini_task_t)*FINI_QUEUE_LEN);
        goto INIT_ERR_4;
    }

    //Reset _readyQueue and _finiQueue indexes
    if (resetQueueIdx() != XTASKS_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot reset the Task Manager");
        INIT_ERR_6: munmap((void *)_asyncRst, sizeof(uint32_t));
        goto INIT_ERR_5;
    }

    //Init the HW instrumentation
    if (initHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_EFILE;
        //NOTE: PRINT_ERROR done inside the function
        goto INIT_ERR_6;
    }

    //Allocate the task_info buffer
    xdma_status s;
    s = xdmaAllocateKernelBuffer((void**)&_tasksBuff, &_tasksBuffHandle,
        NUM_RUN_TASKS*sizeof(task_info_t));
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate kernel memory for task information");
        INIT_ERR_7: finiHWIns();
        _tasksBuff = NULL;
        _tasksBuffPhy = NULL;
        goto INIT_ERR_6;
    }
    unsigned long phyAddr;
    s = xdmaGetDMAAddress(_tasksBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot get physical address of task info. region");
        INIT_ERR_8: xdmaFreeKernelBuffer((void *)_tasksBuff, _tasksBuffHandle);
        goto INIT_ERR_7;
    }
    _tasksBuffPhy = (task_info_t *)phyAddr;

    //Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto INIT_ERR_8;
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

    //Free tasks buffer
    xdma_status s = xdmaFreeKernelBuffer((void *)_tasksBuff, _tasksBuffHandle);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }
    _tasksBuff = NULL;
    _tasksBuffPhy = NULL;

    //Finialize the HW instrumentation
    if (finiHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    //Unmap the Task Manager queues
    int status = munmap(_finiQueue, sizeof(fini_task_t)*FINI_QUEUE_LEN);
    if (status < 0) {
        ret = XTASKS_EFILE;
    }
    status = munmap(_readyQueue, sizeof(ready_task_t)*READY_QUEUE_LEN);
    if (status < 0) {
        ret = XTASKS_EFILE;
    }
    status = munmap((void *)_asyncRst, sizeof(uint32_t));
    if (status < 0) {
        ret = XTASKS_EFILE;
    }
    int statusRd, statusFi, statusCtrl;
    statusRd = close(_readyQFd);
    statusFi = close(_finiQFd);
    statusCtrl = close(_ctrlFd);

    if (statusRd == -1 || statusFi == -1 || statusCtrl == -1) {
        ret = XTASKS_EFILE;
    }

    //Free the accelerators array
    _numAccs = 0;
    free(_accs);

    //Close xdma library
    if (xdmaClose() != XDMA_SUCCESS) {
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
    for (int i = 0; i < NUM_RUN_TASKS; ++i) {
        if (_tasks[i].hwTask == NULL) {
            if (__sync_bool_compare_and_swap(&_tasks[i].hwTask, 0, &_tasksBuff[i])) {
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
    //_tasks[idx].hwTask = &_tasksBuff[idx]; //NOTE: Done in getFreeTaskEntry()
    _tasks[idx].argsCnt = 0;
    _tasks[idx].accel = accel;
    _tasks[idx].hwTask->taskID = (uintptr_t)(&_tasks[idx]);
    _tasks[idx].hwTask->profile.timerAddr = _insTimerAddr;
    _tasks[idx].hwTask->profile.bufferAddr = (uintptr_t)(&_insBuffPhy[idx]);
    _tasks[idx].hwTask->compute = compute;
    _tasks[idx].hwTask->destID = HW_TASK_DEST_ID_TM;
    _tasks[idx].tmTask.taskInfoAddr = (uintptr_t)(&_tasksBuffPhy[idx]);
    _tasks[idx].tmTask.argsBitmask = 0xFFFF; //All will be ready
    //_tasks[idx].tmTask.size = ?¿; //Computed at submit
    _tasks[idx].tmTask.accID = accel->info.id;
    _tasks[idx].tmTask.valid = FREE_ENTRY_MASK; //Not ready yet

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    task_t * task = (task_t *)(*handle);
    *handle = NULL;
    //__sync_synchronize(); //Execute previous operations before the next instruction
    task->hwTask = NULL;

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
    task->hwTask->args[idx].argCached = flags;
    task->hwTask->args[idx].argID = idx;
    task->hwTask->args[idx].argAddr = value;

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
        task->hwTask->args[idx].argCached = flags;
        task->hwTask->args[idx].argID = idx;
        task->hwTask->args[idx].argAddr = values[i];
    }
    task->argsCnt += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    acc_t * acc = task->accel;

    if (task == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }

    // Get an empty slot into the manager ready queue
    size_t idx, next, offset;
    offset = acc->info.id*READY_QUEUE_ACC_LEN;
    do {
        idx = acc->readyQueueIdx;
        if (_readyQueue[offset + idx].valid == VALID_ENTRY_MASK) {
            return XTASKS_ENOENTRY;
        }
        next = (idx+1)%READY_QUEUE_ACC_LEN;
    } while ( !__sync_bool_compare_and_swap(&acc->readyQueueIdx, idx, next) );

    // Copy the task info structure into kernel memory
    // NOTE: For now, already there

    // Copy the ready task structure into the BRAM
    task->tmTask.taskSize = (HW_TASK_HEAD_BYTES + task->argsCnt*sizeof(task_info_arg_t)) /
        sizeof(uint64_t);
    memcpy(&_readyQueue[offset + idx], &task->tmTask, sizeof(ready_task_t));
    __sync_synchronize();
    _readyQueue[offset + idx].valid = VALID_ENTRY_MASK;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    acc_t * acc = task->accel;
    size_t tries = 0;
    size_t const MAX_WAIT_TASKS_TRIES = 0xFFFFFFFF;

    //NOTE: This implementation loses some tasks if waitTask and tryGetFinishedTask are combined.
    //      Force waiting for the first submited task?
    while (task->tmTask.valid == 0 && tries++ < MAX_WAIT_TASKS_TRIES) {
        xtasks_task_id id;
        xtasks_task_handle h;
        xtasksTryGetFinishedTaskAccel(acc, &h, &id);
    }
    return tries > MAX_WAIT_TASKS_TRIES ? XTASKS_PENDING : XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle * handle, xtasks_task_id * id)
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

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel,
    xtasks_task_handle * handle, xtasks_task_id * id)
{
    acc_t * acc = (acc_t *)accel;

    if (handle == NULL || id == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }

    // Get a non-empty slot into the manager finished queue
    size_t idx, next, offset;
    offset = acc->info.id*READY_QUEUE_ACC_LEN;
    do {
        idx = acc->finiQueueIdx;
        if (_finiQueue[offset + idx].valid != VALID_ENTRY_MASK) {
            return XTASKS_PENDING;
        }
        next = (idx+1)%FINI_QUEUE_ACC_LEN;
    } while ( !__sync_bool_compare_and_swap(&acc->finiQueueIdx, idx, next) );

    //Extract the information from the finished buffer
    uintptr_t tmp = _finiQueue[offset + idx].taskID;

    //Free the buffer slot
    __sync_synchronize();
    _finiQueue[offset + idx].valid = FREE_ENTRY_MASK;

    task_t * task = (task_t *)tmp;
    *handle = (xtasks_task_handle)task;
    *id = task->id;

    //Mark the task as executed (using the valid field as it is not used in the cached copy)
    task->tmTask.valid = 1;

    return XDMA_SUCCESS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_task_handle const handle, xtasks_ins_times ** times)
{
    task_t * task = (task_t *)(handle);
    size_t idx = task - _tasks;

    if (times == NULL || idx >= NUM_RUN_TASKS) return XTASKS_EINVAL;

    *times = &_insBuff[idx];

    return XTASKS_SUCCESS;
}

#ifdef XTASKS_DEBUG
ready_task_t readyQueue(size_t const idx)
{
    return _readyQueue[idx];
}

fini_task_t finiQueue(size_t const idx)
{
    return _finiQueue[idx];
}

task_info_t tasksBuffer(size_t const idx)
{
    return _tasksBuff[idx];
}
#endif
