/*
* Copyright (c) 2018-2019, BSC (Barcelona Supercomputing Center)
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
#include "util/ticket-lock.h"

#include <libxdma.h>
#include <libxdma_version.h>
#include <elf.h>
#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>

#include <assert.h>

#include <admxrc3.h>

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array

#define CMD_IN_QUEUE_LEN        1920            ///< Total number of entries in the cmd_in queue
#define CMD_IN_SUBQUEUE_LEN     128             ///< Number of entries in the sub-queue of cmd_in queue for one accelerator
#define CMD_OUT_QUEUE_LEN       480             ///< Total number of entries in the finish queue
#define CMD_OUT_SUBQUEUE_LEN    32              ///< Number of entries in the sub-queue of finish queue for one accelerator
#define QUEUE_VALID             0x80
#define QUEUE_RESERVED          0x40
#define QUEUE_INVALID           0x00
#define DEF_EXEC_TASK_SIZE      256             ///< Size of hw task when using the defult num. of args.
#define DEF_EXEC_TASK_ARGS_LEN  14              //NOTE: (DEF_EXEC_TASK_SIZE - sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
//NOTE: A task in extended mode MUST fit into the a sub-queue of cmd_in queue
#define EXT_HW_TASK_SIZE        512             ///< Size of hw task when using the extended num. of args.
#define EXT_HW_TASK_ARGS_LEN    30              //NOTE: (EXT_HW_TASK_SIZE - sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
//NOTE: The value NUM_RUN_TASKS may be changed to increase the number of concurrent tasks submited into the accelerators
#define NUM_RUN_TASKS                        1024 ///< Maximum number of concurrently running tasks
#define NEW_TASK_COPY_FLAGS_WORDOFFSET       0  ///< Offset of new_task_copy_t->flags field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_SIZE_WORDOFFSET        32 ///< Offset of new_task_copy_t->size field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_OFFSET_WORDOFFSET      0  ///< Offset of new_task_copy_t->offset field in the 3rd word forming new_task_copy_t
#define NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET 32 ///< Offset of new_task_copy_t->accessedLen field in the 3rd word forming new_task_copy_t

#define READY_QUEUE_ADDRESS 0
#define FINI_QUEUE_ADDRESS 0x30000
#define PICOS_RESET0_ADDRESS 0x10000
#define PICOS_RESET1_ADDRESS 0x20000
#define TASKMANAGER_RESET_ADDRESS 0x40000

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
# error Installed libxdma is not supported (use >= 3.0)
#endif

//! \brief Response out command for execute task commands
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     taskID;     //[64 :127] Executed task identifier
} cmd_out_exec_task_t;

//! \brief Internal library HW accelerator information
typedef struct {
    char                     descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info          info;
    unsigned short volatile  cmdInWrIdx;        ///< Writing index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile  cmdInRdIdx;        ///< Reading index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile  cmdInAvSlots;      ///< Counter for available slots in cmd_in sub-queue
    unsigned short volatile  cmdOutIdx;         ///< Reading index of the accelerator sub-queue in the cmd_out queue
} acc_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    cmd_exec_task_header_t *cmdHeader;     ///< Pointer to the cmd_exec_task_header_t struct
    cmd_exec_task_arg_t *   cmdExecArgs;   ///< Pointer to the array of cmd_exec_task_arg_t structs
    acc_t *                 accel;         ///< Accelerator where the task will run
    uint8_t                 extSize;       ///< Whether the space available in args is extended or not
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint8_t *            _cmdExecTaskBuff;   ///< Buffer to send the HW tasks
static task_t *             _tasks;             ///< Array with internal task information
static uint64_t            *_cmdInQueue;        ///< Command IN queue
static uint64_t            *_cmdOutQueue;       ///< Command OUT queue
static uint32_t volatile   *_taskmanagerRst;    ///< Register to reset Task Manager

static ticketLock_t         _bufferTicket;      ///< Lock to atomically access PCI direct slave
static ADMXRC3_HANDLE       _hDevice;           ///< Alphadata device handle

static inline __attribute__((always_inline)) void resetTaskManager()
{
    //Nudge one register
    *_taskmanagerRst = 0x00;
    for ( int i = 0; i < 10; i++ ) i = i; // Lose some time
    *_taskmanagerRst = 0x01;
}

static xtasks_stat xtasksSubmitCommand(acc_t * acc, uint64_t * command, size_t const length)
{
    size_t idx;
    uint64_t cmdHeader;
    size_t const offset = acc->info.id*CMD_IN_SUBQUEUE_LEN;
    cmd_header_t * const cmdHeaderPtr = (cmd_header_t * const)&cmdHeader;

    // While there is not enough space in the queue, look for already read commands
    while (acc->cmdInAvSlots < length) {
        ticketLockAcquire(&_bufferTicket);
        if (acc->cmdInAvSlots < length) {
SUB_CMD_CHECK_RD:
            idx = acc->cmdInRdIdx;
            cmdHeader = _cmdInQueue[offset + idx];
            if (cmdHeaderPtr->valid == QUEUE_INVALID) {
                uint8_t const cmdNumArgs = cmdHeaderPtr->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
                size_t const cmdNumWords = (sizeof(cmd_exec_task_header_t) +
                    sizeof(cmd_exec_task_arg_t)*cmdNumArgs)/sizeof(uint64_t);
                acc->cmdInRdIdx = (idx + cmdNumWords)%CMD_IN_SUBQUEUE_LEN;
                acc->cmdInAvSlots += cmdNumWords;
            }
        } else {
            //NOTE: At this point the thread has the lock acquired, so directly bypass it into the queue idx update.
            //      The lock will be released in the code after the atomic operations
            goto SUB_CMD_UPDATE_IDX;
        }
        ticketLockRelease(&_bufferTicket);
    }

    ticketLockAcquire(&_bufferTicket);
    if (acc->cmdInAvSlots >= length) {
SUB_CMD_UPDATE_IDX:
        idx = acc->cmdInWrIdx;
        acc->cmdInWrIdx = (idx + length)%CMD_IN_SUBQUEUE_LEN;
        acc->cmdInAvSlots -= length;
        cmdHeader = _cmdInQueue[offset + idx];
        cmdHeaderPtr->valid = QUEUE_RESERVED;
        _cmdInQueue[offset + idx] = cmdHeader;

        // Do no write the header (1st word pointer by command ptr) until all payload is write
        // Check if 2 writes have to be done because there is not enough space at the end of subqueue
        const size_t count = min(CMD_IN_SUBQUEUE_LEN - idx - 1, length - 1);
        memcpy(&_cmdInQueue[offset + idx + 1], command + 1, count*sizeof(uint64_t));
        if ((length - 1) > count) {
            memcpy(&_cmdInQueue[offset], command + 1 + count, (length - count)*sizeof(uint64_t));
        }
        cmdHeader = *command;
        cmdHeaderPtr->valid = QUEUE_VALID;

        __sync_synchronize();
        // Write the header now
        _cmdInQueue[offset + idx] = cmdHeader;
    } else {
        //NOTE: At this point the thread has the lock acquired, so directly bypass it into the check.
        //      The lock will be released in the code after the check
        goto SUB_CMD_CHECK_RD;
    }

    ticketLockRelease(&_bufferTicket);

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksInitHWIns(size_t const nEvents)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksFiniHWIns()
{
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    //Initialize xdma memory subsystem
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
        goto INIT_ERR_1;
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(buffer, "r");
    if (accMapFile == NULL) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open FPGA configuration file");
        free(buffer);
        goto INIT_ERR_1;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("First line of FPGA configuration file is not valid");
        free(buffer);
        fclose(accMapFile);
        goto INIT_ERR_1;
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
            _accs[i].info.maxTasks = -1;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
            _accs[i].cmdInWrIdx = 0;
            _accs[i].cmdInAvSlots = CMD_IN_SUBQUEUE_LEN;
            _accs[i].cmdInRdIdx = 0;
            _accs[i].cmdOutIdx = 0;
        }
    }
    fclose(accMapFile);
    free(buffer);
    _numAccs = (total < _numAccs) ? total : _numAccs;

    if (retFscanf != EOF && retFscanf != 0) {
        //Looks like the configuration file doesn't match the expected format
        fprintf(stderr, "WARN: xTasks configuration file may be not well formated.\n");
    } else if (_numAccs > CMD_IN_QUEUE_LEN/CMD_IN_SUBQUEUE_LEN) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("The maximum number of accelerators supported by the library was reached");
        goto INIT_ERR_1;
    }

    ADMXRC3_STATUS admxrc3_status = ADMXRC3_Open(0, &_hDevice);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
        perror("Error opening alphadata device");
        goto INIT_ERR_1;
    }

    ticketLockInit(&_bufferTicket);

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, READY_QUEUE_ADDRESS, sizeof(uint64_t)*CMD_IN_QUEUE_LEN, (void**)&_cmdInQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map ready queue of Task Manager");
        goto INIT_ERR_MMAP_READY;
    }

    //If any, invalidate commands in cmd_in queue
    _memset(_cmdInQueue, 0, CMD_IN_QUEUE_LEN*sizeof(uint64_t));

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, FINI_QUEUE_ADDRESS, sizeof(uint64_t)*CMD_OUT_QUEUE_LEN, (void**)&_cmdOutQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map finish queue of Task Manager");
        goto INIT_ERR_MMAP_FINI;
    }

    //If any, invalidate commands in cmd_out queue
    _memset(_cmdOutQueue, 0, CMD_OUT_QUEUE_LEN*sizeof(uint64_t));

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, TASKMANAGER_RESET_ADDRESS, sizeof(uint32_t), (void**)&_taskmanagerRst);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map control registers of Task Manager");
        goto INIT_ERR_MMAP_RST;
    }

    //Reset _cmdInQueue and _finiQueue indexes
    resetTaskManager();

    //Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto INIT_ERR_ALLOC_TASKS;
    }
    _cmdExecTaskBuff = (uint8_t *)malloc(NUM_RUN_TASKS*DEF_EXEC_TASK_SIZE);
    if (_cmdExecTaskBuff == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for exec. tasks buffer");
        goto INIT_ERR_ALLOC_EXEC_TASKS_BUFF;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].cmdHeader = (cmd_exec_task_header_t *)&_cmdExecTaskBuff[idx*DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)
            &_cmdExecTaskBuff[idx*DEF_EXEC_TASK_SIZE + sizeof(cmd_exec_task_header_t)];
        _tasks[idx].extSize = 0;
    }

    return ret;

    //Error handling code
        free(_cmdExecTaskBuff);
    INIT_ERR_ALLOC_EXEC_TASKS_BUFF:
        free(_tasks);
    INIT_ERR_ALLOC_TASKS:
        ADMXRC3_UnmapWindow(_hDevice, (void*)_taskmanagerRst);
    INIT_ERR_MMAP_RST:
        ADMXRC3_UnmapWindow(_hDevice, _cmdOutQueue);
    INIT_ERR_MMAP_FINI:
        ADMXRC3_UnmapWindow(_hDevice, _cmdInQueue);
    INIT_ERR_MMAP_READY:
        ADMXRC3_Close(_hDevice);
        ticketLockFini(&_bufferTicket);
    INIT_ERR_1:
        free(_accs);
        _numAccs = 0;
    INIT_ERR_0:
        __sync_sub_and_fetch(&_init_cnt, 1);
    return ret;
}

xtasks_stat xtasksFini()
{
    //Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    //Free tasks array
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        if (_tasks[idx].extSize) {
            //Free tasks in extended mode
            free((void *)_tasks[idx].cmdHeader);
        }
    }
    free(_cmdExecTaskBuff);
    free(_tasks);
    _tasks = NULL;

    *_taskmanagerRst = 0;

    //Finialize the HW instrumentation if needed
    /*if (xtasksFiniHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_ERROR;
    }*/

    //Unmap the Task Manager queues
    ADMXRC3_STATUS statusRd, statusFi, statusCtrl;
    statusCtrl = ADMXRC3_UnmapWindow(_hDevice, (void*)_taskmanagerRst);
    statusFi = ADMXRC3_UnmapWindow(_hDevice, _cmdOutQueue);
    statusRd = ADMXRC3_UnmapWindow(_hDevice, _cmdInQueue);
    if (statusRd != ADMXRC3_SUCCESS || statusFi != ADMXRC3_SUCCESS || statusCtrl != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    ticketLockFini(&_bufferTicket);
    ADMXRC3_STATUS statusDev = ADMXRC3_Close(_hDevice);
    if (statusDev != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
    }
    free(_accs);
    _accs = NULL;
    _numAccs = 0;

    //Close xdma memory management
    if (xdmaFiniMem() != XDMA_SUCCESS) {
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
        if (_tasks[i].id == 0) {
            if (__sync_bool_compare_and_swap(&_tasks[i].id, 0, 1)) {
                return i;
            }
        }
    }
    return -1;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_handle const parent, xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
    acc_t * accel = (acc_t *)accId;
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].accel = accel;
    _tasks[idx].cmdHeader->header.commandCode = CMD_EXEC_TASK_CODE;
    _tasks[idx].cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    _tasks[idx].cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    _tasks[idx].cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_TM;
    _tasks[idx].cmdHeader->parentID = (uintptr_t)(parent);
    _tasks[idx].cmdHeader->taskID = (uintptr_t)(&_tasks[idx]);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    task_t * task = (task_t *)(*handle);
    *handle = NULL;
    task->id = 0;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (argsCnt == DEF_EXEC_TASK_ARGS_LEN) {
        //Entering in extended mode
        cmd_exec_task_header_t * prevHeader = task->cmdHeader;
        task->cmdHeader = (cmd_exec_task_header_t *)malloc(EXT_HW_TASK_SIZE);
        if (task->cmdHeader == NULL) {
            task->cmdHeader = prevHeader;
            return XTASKS_ENOMEM;
        }
        task->extSize = 1;
        task->cmdExecArgs = (cmd_exec_task_arg_t *)(task->cmdHeader + 1);
        memcpy(task->cmdHeader, prevHeader, DEF_EXEC_TASK_SIZE); //< Move the hw task header and args
    }

    argsCnt = task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET]++;
    task->cmdExecArgs[argsCnt].flags = flags;
    task->cmdExecArgs[argsCnt].id = id;
    task->cmdExecArgs[argsCnt].value = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (num > EXT_HW_TASK_ARGS_LEN - argsCnt) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (num > (DEF_EXEC_TASK_ARGS_LEN - argsCnt) && argsCnt <= DEF_EXEC_TASK_ARGS_LEN) {
        //Entering in extended mode because:
        // 1) The number of args to add does not fit in current allocated space
        // 2) We are not in extended mode
        // 3) The number of args will fit in extended mode
        cmd_exec_task_header_t * prevHeader = task->cmdHeader;
        task->cmdHeader = (cmd_exec_task_header_t *)malloc(EXT_HW_TASK_SIZE);
        if (task->cmdHeader == NULL) {
            task->cmdHeader = prevHeader;
            return XTASKS_ENOMEM;
        }
        task->extSize = 1;
        task->cmdExecArgs = (cmd_exec_task_arg_t *)(task->cmdHeader + 1);
        memcpy(task->cmdHeader, prevHeader, DEF_EXEC_TASK_SIZE); //< Move the hw task header and args
    }

    for (size_t i = 0, idx = argsCnt; i < num; ++i, ++idx) {
        task->cmdExecArgs[idx].flags = flags;
        task->cmdExecArgs[idx].id = idx;
        task->cmdExecArgs[idx].value = values[i];
    }
    task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    acc_t * acc = task->accel;

    if (task == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }
    // Update the task/command information
    task->cmdHeader->header.valid = QUEUE_VALID;

    //NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numCmdWords = (sizeof(cmd_exec_task_header_t) + sizeof(cmd_exec_task_arg_t)*argsCnt)/sizeof(uint64_t);
    return xtasksSubmitCommand(acc, (uint64_t *)task->cmdHeader, numCmdWords);
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    acc_t * acc = task->accel;
    size_t tries = 0;
    size_t const MAX_WAIT_TASKS_TRIES = 0xFFFFFFFF;

    //NOTE: This implementation loses some tasks if waitTask and tryGetFinishedTask are combined.
    //      Force waiting for the first submited task?
    while (task->cmdHeader->header.valid == QUEUE_VALID && tries++ < MAX_WAIT_TASKS_TRIES) {
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
    uint64_t * const subqueue = _cmdOutQueue + acc->info.id*CMD_OUT_SUBQUEUE_LEN;

    if (handle == NULL || id == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }

    ticketLockAcquire(&_bufferTicket);

    size_t idx = acc->cmdOutIdx;
    uint64_t cmdBuffer = subqueue[idx];
    cmd_header_t * cmd = (cmd_header_t *)&cmdBuffer;

    if (cmd->valid == QUEUE_VALID) {
        //Read the command header
        idx = acc->cmdOutIdx;
        cmdBuffer = subqueue[idx];

#ifdef XTASKS_DEBUG
        if (cmd->commandCode != CMD_FINI_EXEC_CODE || cmd->commandArgs[CMD_FINI_EXEC_ARGS_ACCID_OFFSET] != acc->info.id) {
            PRINT_ERROR("Found unexpected data when executing xtasksTryGetFinishedTaskAccel");
            __sync_lock_release(&acc->cmdOutLock);
            return XTASKS_ERROR;
        }
#endif /* XTASKS_DEBUG */

        //Read the command payload (task identifier)
        size_t const dataIdx = (idx+1)%CMD_OUT_SUBQUEUE_LEN;
        uint64_t const taskID = subqueue[dataIdx];
        subqueue[dataIdx] = 0; //< Clean the buffer slot

        //Invalidate the buffer entry
        cmd = (cmd_header_t *)&subqueue[idx];
        cmd->valid = QUEUE_INVALID;

        //Update the read index and release the lock
        acc->cmdOutIdx = (idx + sizeof(cmd_out_exec_task_t)/sizeof(uint64_t))%CMD_OUT_SUBQUEUE_LEN;
        ticketLockRelease(&_bufferTicket);

#ifdef XTASKS_DEBUG
        if (taskID < (uintptr_t)_tasks || taskID >= (uintptr_t)(_tasks + NUM_RUN_TASKS)) {
            PRINT_ERROR("Found an invalid task identifier when executing xtasksTryGetFinishedTaskAccel");
            return XTASKS_ERROR;
        }
#endif /* XTASKS_DEBUG */

        uintptr_t taskPtr = (uintptr_t)taskID;
        task_t * task = (task_t *)taskPtr;
        *handle = (xtasks_task_handle)task;
        *id = task->id;

        //Mark the task as executed (using the valid field as it is not used in the cached copy)<
        task->cmdHeader->header.valid = QUEUE_INVALID;

        return XTASKS_SUCCESS;

    }

    ticketLockRelease(&_bufferTicket);
    return XTASKS_PENDING;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event * events, size_t const maxCount)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask ** task)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_handle const parent, size_t count)
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
    ticketLockAcquire(&_bufferTicket);
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpyAsync(usr, handle, len, offset, mode, cpyHandle);
    ticketLockRelease(&_bufferTicket);
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
uint64_t cmdInQueue(size_t const accID, size_t const idx)
{
    return _cmdInQueue[accID*CMD_IN_SUBQUEUE_LEN + idx];
}

fini_task_t finiQueue(size_t const idx)
{
    return _finiQueue[idx];
}

uint64_t newQueue(size_t const idx)
{
    return _newQueue[idx];
}

rem_fini_task_t remFiniQueue(size_t const idx)
{
    return _remFiniQueue[idx];
}

xtasks_ins_event accelGetEvent(size_t const aIdx, size_t const evIdx) {
    return _instrBuff[aIdx*_numInstrEvents + evIdx];
}

void accelPrintInstrBuffer(size_t const aIdx) {
    xtasks_ins_event *event = _instrBuff + aIdx*_numInstrEvents;
    fprintf(stderr, "timestamp, accid, eventid, value\n");
    while ( event->eventId != XTASKS_EVENT_TYPE_INVALID ) {
        fprintf(stderr, "%llu,\t%u,\t%u,\t%llu\n", (unsigned long long int)event->timestamp,
                event->eventType, event->eventId, (unsigned long long int)event->value);
        event++;
    }
}

#endif
