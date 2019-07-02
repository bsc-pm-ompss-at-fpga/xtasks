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

//#define PICOS

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array

#define CMD_IN_QUEUE_LEN        2048            ///< Total number of entries in the cmd_in queue
#define CMD_IN_SUBQUEUE_LEN     64              ///< Number of entries in the sub-queue of cmd_in queue for one accelerator
#define CMD_OUT_QUEUE_LEN       1024            ///< Total number of entries in the cmd_out queue
#define CMD_OUT_SUBQUEUE_LEN    32              ///< Number of entries in the sub-queue of cmd_out queue for one accelerator
#define NEW_QUEUE_LEN           1024            //NOTE: Each element is a uint64_t (the number of arguments for a task is unknown)
#define REMFINI_QUEUE_LEN       1024            ///< Total number of entries in the remote finished queue
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

#ifdef PICOS
#define PICOS_RESET0_ADDRESS 0x14000
#define PICOS_RESET1_ADDRESS 0x18000
#define PICOS_DEBUG_ADDRESS 0x50000
#endif

#define READY_QUEUE_ADDRESS 0x00000000
#define FINI_QUEUE_ADDRESS 0x00004000
#define TASKMANAGER_RESET_ADDRESS 0x00010000
#define NEW_QUEUE_ADDRESS 0x00008000
#define REMFINI_QUEUE_ADDRESS 0x0000C000

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
# error Installed libxdma is not supported (use >= 3.0)
#endif

//! \brief Response out command for execute task commands
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     taskID;     //[64 :127] Executed task identifier
} cmd_out_exec_task_t;

//! \brief New task buffer representation  (Only the header part, N arguments follow the header)
typedef struct __attribute__ ((__packed__)) {
    uint8_t   _padding;      //[0  :7  ]
    uint8_t   numArgs;       //[8  :15 ] Number of arguments after this header
    uint8_t   numDeps;       //[16 :23 ] Number of dependencies after the task arguments
    uint8_t   numCopies;     //[24 :31 ] Number of copies after the task dependencies
    uint32_t  archMask:24;   //[32 :55 ] Architecture mask in Picos style
    uint8_t   valid;         //[56 :63 ] Valid Entry
    uint64_t  parentID;      //[64 :127] Parent task identifier that is creating the task
    uint64_t  typeInfo;      //[128:191] Information of task type
} new_task_header_t;

//! \brief New task buffer representation (Only the dependence part, repeated N times)
typedef struct __attribute__ ((__packed__)) {
    uint64_t  address:56; //[0  :55 ] Dependence value
    uint8_t   flags;      //[56 :63 ] Dependence flags
} new_task_dep_t;

//! \brief New task buffer representation (Only the argument part, repeated N times)
typedef struct __attribute__ ((__packed__)) {
    uint64_t address;     //[0  :63 ] Copy address
    uint32_t flags;       //[64 :95 ] Copy flags
    uint32_t size;        //[96 :127] Size of the copy
    uint32_t offset;      //[128:159] Offset of accessed region in the copy
    uint32_t accessedLen; //[160:191] Length of accessed region in the copy
} new_task_copy_t;

//! \brief Remote finished task buffer representation
typedef struct __attribute__ ((__packed__)) {
    uint8_t  valid;       //[0  :7  ] Valid Entry
    uint8_t _padding[3];
    uint32_t components;  //[32 :63 ] Number of tasks that have finished
    uint64_t taskID;      //[64 :127] Parent task identifier that created the tasks
} rem_fini_task_t;

//! \brief Internal library HW accelerator information
typedef struct {
    char                     descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info          info;
    unsigned short volatile  cmdInWrIdx;        ///< Writing index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile  cmdInRdIdx;        ///< Reading index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile  cmdInAvSlots;      ///< Counter for available slots in cmd_in sub-queue
    unsigned short volatile  cmdOutIdx;         ///< Reading index of the accelerator sub-queue in the cmd_out queue
    unsigned short volatile  instrIdx;          ///< Reading index of the accelerator instrumentation buffer
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
static uint64_t            *_newQueue;          ///< Buffer for the new tasks created in the HW
static size_t               _newQueueIdx;       ///< Reading index of the _newQueue
static rem_fini_task_t     *_remFiniQueue;      ///< Buffer for the remote finished tasks
static size_t               _remFiniQueueIdx;   ///< Writing index of the _remFiniQueue
static uint32_t volatile   *_taskmanagerRst;    ///< Register to reset Task Manager
#ifdef PICOS
static uint32_t            *_picosRst0;         ///< Register to reset Picos (Picos clock domain)
static uint32_t            *_picosRst1;         ///< Register to reset Picos (hwacc clock domain)
static uint32_t            *_picosDebug;        ///< Picos debug registers
static uint64_t             _copyTime;
#endif

static ticketLock_t         _bufferTicket;      ///< Lock to atomically access PCI direct slave
static ADMXRC3_HANDLE       _hDevice;           ///< Alphadata device handle

static inline __attribute__((always_inline)) void resetTaskManager()
{
    //Nudge one register
    *_taskmanagerRst = 0x01;
    for ( int i = 0; i < 10; i++ ) i = i; // Lose some time
    *_taskmanagerRst = 0x00;

#ifdef PICOS
    //Nudge one register
    *_picosRst0 = 1;
    __sync_synchronize();
    *_picosRst0 = 0;

    //Nudge one register
    *_picosRst1 = 1;
    __sync_synchronize();
    *_picosRst1 = 0;
#endif
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
#ifdef PICOS
    _copyTime = 0;
#endif

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
    //_memset(_cmdInQueue, 0, CMD_IN_QUEUE_LEN*sizeof(uint64_t));
    for (int i = 0; i < CMD_IN_QUEUE_LEN; ++i) {
        _cmdInQueue[i] = 0;
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, FINI_QUEUE_ADDRESS, sizeof(uint64_t)*CMD_OUT_QUEUE_LEN, (void**)&_cmdOutQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map finish queue of Task Manager");
        goto INIT_ERR_MMAP_FINI;
    }

    //If any, invalidate commands in cmd_out queue
    //_memset(_cmdOutQueue, 0, CMD_OUT_QUEUE_LEN*sizeof(uint64_t));
    for (int i = 0; i < CMD_OUT_QUEUE_LEN; ++i) {
        _cmdOutQueue[i] = 0;
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, NEW_QUEUE_ADDRESS, sizeof(uint64_t)*NEW_QUEUE_LEN, (void**)&_newQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map new queue of Task Manager");
        goto INIT_ERR_MAP_NEW;
    }

    //If any, invalidate tasks in newQueue
    //_memset(_newQueue, 0, NEW_QUEUE_LEN*sizeof(uint64_t));
    /*for (int i = 0; i < NEW_QUEUE_LEN; ++i) {
        _newQueue[i] = 0;
    }*/

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, REMFINI_QUEUE_ADDRESS, sizeof(uint64_t)*REMFINI_QUEUE_LEN, (void**)&_remFiniQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map remote finished queue of Task Manager");
        goto INIT_ERR_MAP_REMFINI;
    }

    //If any, invalidate tasks in remFiniQueue
    //_memset(_remFiniQueue, 0, REMFINI_QUEUE_LEN*sizeof(uint64_t));
    /*for (int i = 0; i < REMFINI_QUEUE_LEN; ++i) {
        _remFiniQueue[i] = 0;
    }*/

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, TASKMANAGER_RESET_ADDRESS, sizeof(uint32_t), (void**)&_taskmanagerRst);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map control registers of Task Manager");
        goto INIT_ERR_MMAP_RST;
    }

#ifdef PICOS
    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, PICOS_DEBUG_ADDRESS, sizeof(uint32_t)*36, (void**)&_picosDebug);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot map Picos debug registers");
        goto INIT_ERR_MMAP_PICOSDEBUG;
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, PICOS_RESET0_ADDRESS, sizeof(uint32_t), (void**)&_picosRst0);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot map Picos reset register 0");
        goto INIT_ERR_MMAP_PICOSRST0;
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, PICOS_RESET1_ADDRESS, sizeof(uint32_t), (void**)&_picosRst1);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot map Picos reset register 1");
        goto INIT_ERR_MMAP_PICOSRST1;
    }
#endif

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
#ifdef PICOS
        ADMXRC3_UnmapWindow(_hDevice, (void*)_picosRst1);
    INIT_ERR_MMAP_PICOSRST1:
        ADMXRC3_UnmapWindow(_hDevice, (void*)_picosRst0);
    INIT_ERR_MMAP_PICOSRST0:
        ADMXRC3_UnmapWindow(_hDevice, (void*)_picosDebug);
    INIT_ERR_MMAP_PICOSDEBUG:
#endif
        ADMXRC3_UnmapWindow(_hDevice, (void*)_taskmanagerRst);
    INIT_ERR_MMAP_RST:
        ADMXRC3_UnmapWindow(_hDevice, _remFiniQueue);
    INIT_ERR_MAP_REMFINI:
        ADMXRC3_UnmapWindow(_hDevice, _newQueue);
    INIT_ERR_MAP_NEW:
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

#ifdef PICOS
    /*static const char* regNames[] = {
        "new",
        "ready",
        "finish",
        "remote ready",
        "remote finish",
        "smp finish"
    };

    //Picos-only code:
    printf("----------------------------\n");
    printf("Picos debug registers:\n");
    for (int i = 0; i < 6; ++i) {
        printf("%s:\t%u\n", regNames[i], _picosDebug[i]);
    }
    for (int i = 6; i < 6+15; ++i) {
        printf("hwacc %d finish:\t%u\n", i-6, _picosDebug[i]);
    }
    for (int i = 6+15; i < 6+15+15; ++i) {
        printf("hwacc %d time:\t%u\n", i-6-15, _picosDebug[i]);
    }
    printf("----------------------------\n");*/
    printf("compute %d\n", _picosDebug[6+15]);
    printf("copy %lu\n", _copyTime);
#endif

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

#ifdef PICOS
    *_taskmanagerRst = 0;
#endif

    //Unmap the Task Manager queues
    ADMXRC3_STATUS statusRd, statusFi, statusNw, statusRFi, statusCtrl;
    statusCtrl = ADMXRC3_UnmapWindow(_hDevice, (void*)_taskmanagerRst);
    statusFi = ADMXRC3_UnmapWindow(_hDevice, _cmdOutQueue);
    statusNw = ADMXRC3_UnmapWindow(_hDevice, _newQueue);
    statusRFi = ADMXRC3_UnmapWindow(_hDevice, _remFiniQueue);
    statusRd = ADMXRC3_UnmapWindow(_hDevice, _cmdInQueue);
    ADMXRC3_STATUS statusDbg = ADMXRC3_SUCCESS;
    ADMXRC3_STATUS statusRst0 = ADMXRC3_SUCCESS;
    ADMXRC3_STATUS statusRst1 = ADMXRC3_SUCCESS;
#ifdef PICOS
    statusDbg = ADMXRC3_UnmapWindow(_hDevice, _picosDebug);
    statusRst0 = ADMXRC3_UnmapWindow(_hDevice, _picosRst0);
    statusRst1 = ADMXRC3_UnmapWindow(_hDevice, _picosRst1);
#endif
    if (statusRd != ADMXRC3_SUCCESS || statusFi != ADMXRC3_SUCCESS || statusCtrl != ADMXRC3_SUCCESS ||
            statusDbg != ADMXRC3_SUCCESS || statusRst0 != ADMXRC3_SUCCESS || statusRst1 != ADMXRC3_SUCCESS ||
            statusRFi != ADMXRC3_SUCCESS || statusNw != ADMXRC3_SUCCESS) {
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
    if (_newQueue == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the manager new queue
    size_t idx, taskSize;
    new_task_header_t * hwTaskHeader;
    ticketLockAcquire(&_bufferTicket);

    idx = _newQueueIdx;
    hwTaskHeader = (new_task_header_t *)(&_newQueue[idx]);
    if (hwTaskHeader->valid != QUEUE_VALID) {
        ticketLockRelease(&_bufferTicket);
        return XTASKS_PENDING;
    }
    taskSize = (
                sizeof(new_task_header_t) +
                sizeof(uint64_t)*hwTaskHeader->numArgs +
                sizeof(new_task_dep_t)*hwTaskHeader->numDeps +
                sizeof(new_task_copy_t)*hwTaskHeader->numCopies)/
            sizeof(uint64_t);
    _newQueueIdx = (idx+taskSize)%NEW_QUEUE_LEN;

    //Extract the information from the new buffer
    *task = realloc(*task,
        sizeof(xtasks_newtask) +
        sizeof(xtasks_newtask_arg)*hwTaskHeader->numArgs +
        sizeof(xtasks_newtask_dep)*hwTaskHeader->numDeps +
        sizeof(xtasks_newtask_copy)*hwTaskHeader->numCopies);
    (*task)->args = (xtasks_newtask_arg *)(*task + 1);
    (*task)->numArgs = hwTaskHeader->numArgs;
    (*task)->deps = (xtasks_newtask_dep *)((*task)->args + (*task)->numArgs);
    (*task)->numDeps = hwTaskHeader->numDeps;
    (*task)->copies = (xtasks_newtask_copy *)((*task)->deps + (*task)->numDeps);
    (*task)->numCopies = hwTaskHeader->numCopies;
    (*task)->architecture = hwTaskHeader->archMask;

    idx = (idx+1)%NEW_QUEUE_LEN; //NOTE: new_task_header_t->parentID field is the 2nd word
    task_t * parentTask = (task_t *)((uintptr_t)(_newQueue[idx]));
    (*task)->parentId = parentTask->id; //< The external parent identifier must be returned (not the xtasks internal one)
    _newQueue[idx] = 0; //< Cleanup the memory position

    idx = (idx+1)%NEW_QUEUE_LEN; //NOTE: new_task_header_t->typeInfo field is the 3rd word
    (*task)->typeInfo = _newQueue[idx];
    _newQueue[idx] = 0; //< Cleanup the memory position

    for (size_t i = 0; i < (*task)->numCopies; ++i) {
        //NOTE: Each copy uses 3 uint64_t elements in the newQueue
        //      After using each memory position, we have to clean it

        //NOTE: new_task_copy_t->address field is the 1st word
        idx = (idx+1)%NEW_QUEUE_LEN;
        (*task)->copies[i].address = (void *)((uintptr_t)_newQueue[idx]);
        _newQueue[idx] = 0;

         //NOTE: new_task_copy_t->flags and new_task_copy_t->size fields are the 2nd word
        idx = (idx+1)%NEW_QUEUE_LEN;
        uint32_t copyFlags = _newQueue[idx] >> NEW_TASK_COPY_FLAGS_WORDOFFSET;
        (*task)->copies[i].flags = copyFlags;
        uint32_t copySize = _newQueue[idx] >> NEW_TASK_COPY_SIZE_WORDOFFSET;
        (*task)->copies[i].size = copySize;
        _newQueue[idx] = 0;

         //NOTE: new_task_copy_t->offset and new_task_copy_t->accessedLen fields are the 2nd word
        idx = (idx+1)%NEW_QUEUE_LEN;
        uint32_t copyOffset = _newQueue[idx] >> NEW_TASK_COPY_OFFSET_WORDOFFSET;
        (*task)->copies[i].offset = copyOffset;
        uint32_t copyAccessedLen = _newQueue[idx] >> NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET;
        (*task)->copies[i].accessedLen = copyAccessedLen;
        _newQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numDeps; ++i) {
        idx = (idx+1)%NEW_QUEUE_LEN;
        new_task_dep_t * hwTaskDep = (new_task_dep_t *)(&_newQueue[idx]);

        //Parse the dependence information
        (*task)->deps[i].address = hwTaskDep->address;
        (*task)->deps[i].flags = hwTaskDep->flags;

        //Cleanup the memory position
        _newQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numArgs; ++i) {
        //Check that arg pointer is not out of bounds
        idx = (idx+1)%NEW_QUEUE_LEN;
        (*task)->args[i] = _newQueue[idx];

        //Cleanup the memory position
        _newQueue[idx] = 0;
    }

    //Free the buffer slot
    //NOTE: This word cannot be set to 0 as the task size information must be keept
    __sync_synchronize();
    hwTaskHeader->valid = QUEUE_INVALID;
    ticketLockFini(&_bufferTicket);

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_handle const parent, size_t count)
{
    task_t * task = (task_t *)(parent);

    if (task == NULL) {
        return XTASKS_EINVAL;
    }

    ticketLockAcquire(&_bufferTicket);

    // Get an empty slot into the TM remote finished queue
    size_t idx;
    idx = _remFiniQueueIdx;
    if (_remFiniQueue[idx].valid == QUEUE_VALID) {
        ticketLockRelease(&_bufferTicket);
        return XTASKS_ENOENTRY;
    }
    _remFiniQueueIdx = (idx+1)%REMFINI_QUEUE_LEN;

    // Copy the information into the queue
    _remFiniQueue[idx].taskID = (uintptr_t)(task);
    _remFiniQueue[idx].components = count;
    __sync_synchronize();
    _remFiniQueue[idx].valid = QUEUE_VALID;
    ticketLockRelease(&_bufferTicket);

    return XTASKS_SUCCESS;
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
#ifdef PICOS
    struct timeval time1, time2;
    gettimeofday(&time1, NULL);
#endif
    xdma_status status = xdmaMemcpy(usr, handle, len, offset, mode);
#ifdef PICOS
    gettimeofday(&time2, NULL);
    uint64_t elapsed = (time2.tv_sec*1000000 + time2.tv_usec) - (time1.tv_sec*1000000 + time1.tv_usec);
    if (elapsed == 0)
        printf("Warning, lost precision\n");
    _copyTime += elapsed;
#endif
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
