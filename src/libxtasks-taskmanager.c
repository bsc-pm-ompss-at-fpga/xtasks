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
#include <errno.h>

#include <assert.h>

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array

#define READY_QUEUE_PATH        "/dev/ompss_fpga/task_manager/ready_queue"
#define FINI_QUEUE_PATH         "/dev/ompss_fpga/task_manager/finished_queue"
#define NEW_QUEUE_PATH          "/dev/ompss_fpga/task_manager/new_queue"
#define REMFINI_QUEUE_PATH      "/dev/ompss_fpga/task_manager/remote_finished_queue"
#define TASKMANAGER_RST_PATH    "/dev/ompss_fpga/task_manager/ctrl"

#define READY_QUEUE_LEN         1024            ///< Total number of entries in the ready queue
#define READY_QUEUE_ACC_LEN     32              ///< Number of entries in the sub-queue of ready queue for one accelerator
#define FINI_QUEUE_LEN          1024            ///< Total number of entries in the finish queue
#define FINI_QUEUE_ACC_LEN      32              ///< Number of entries in the sub-queue of finish queue for one accelerator
#define NEW_QUEUE_LEN           1024            //NOTE: Each element is a uint64_t (the number of arguments for a task is unknown)
#define REMFINI_QUEUE_LEN       1024            ///< Total number of entries in the remote finished queue
#define VALID_ENTRY_MASK        0x80
#define FREE_ENTRY_MASK         0
#define DEF_HW_TASK_SIZE        256             ///< Size of hw task when using the defult num. of args.
#define DEF_HW_TASK_ARGS_LEN    13              //NOTE: (DEF_HW_TASK_SIZE - sizeof(hw_task_header_t))/sizeof(hw_task_arg_t)
#define EXT_HW_TASK_SIZE        1024            ///< Size of hw task when using the extended num. of args.
#define EXT_HW_TASK_ARGS_LEN    61              //NOTE: (EXT_HW_TASK_SIZE - sizeof(hw_task_header_t))/sizeof(hw_task_arg_t)
//NOTE: The number of slots must be at least the number of slots in the ready queue (READY_QUEUE_LEN)
//      + the number of running tasks (1 foreach accelerator -> _numAccs)
#define NUM_RUN_TASKS           (READY_QUEUE_LEN + _numAccs) ///< Maximum number of concurrently running tasks
#define HW_TASK_DEST_ID_PS      0x1F            ///< Processing System identifier for the destId field
#define HW_TASK_DEST_ID_TM      0x11            ///< Task manager identifier for the destId field
#define NEW_TASK_COPY_FLAGS_WORDOFFSET       0  ///< Offset of new_task_copy_t->flags field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_SIZE_WORDOFFSET        32 ///< Offset of new_task_copy_t->size field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_OFFSET_WORDOFFSET      0  ///< Offset of new_task_copy_t->offset field in the 3rd word forming new_task_copy_t
#define NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET 32 ///< Offset of new_task_copy_t->accessedLen field in the 3rd word forming new_task_copy_t

#define READY_TASK_FLAG_AVAIL   0x1             ///< Flag to do not set the accelerator unavailable when running
#define READY_TASK_FLAG_BUSY    0x0             ///< Flag to set the accelerator unavailable when running
#define HW_TASK_CMD_EXEC_TASK   0x01            ///< Command code for execute task commands
#define HW_TASK_CMD_SETUP_INS   0x02            ///< Command code for setup instrumentation info.
#define HW_TASK_CMD_ARGS_COMPUTE_OFFSET      0  ///< Offset of Compute flag in the commandArgs array
#define HW_TASK_CMD_ARGS_DEST_ID_OFFSET      1  ///< Offset of Destination id (where accel will send finish signal) in the commandArgs array

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 2
# error Installed libxdma is not supported (use >= 2.0)
#endif

//! \brief HW accelerator representation
typedef struct {
    char            descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
    unsigned short  readyQueueIdx;     ///< Writing index of the accelerator sub-queue in the ready queue
    unsigned short  finiQueueIdx;      ///< Reading index of the accelerator sub-queue in the finished queue
    unsigned short  instrIdx;          ///< Reading index of the accelerator instrumentation buffer
    unsigned short  lock;              ///< Lock for atomic operations over accelerator buffers
} acc_t;

//! \brief Ready task buffer element representation
typedef struct __attribute__ ((__packed__)) {
    uint64_t   taskInfoAddr;  //[0  :63 ] Physical address to the Task info structure
    uint16_t   argsBitmask;   //[64 :79 ] Bitmask defining if the arguments are ready or not
    uint16_t   _padding1;
    uint8_t    taskSize;      //[96 :103] Size of Task Info structure in words (of 64 bits)
    uint8_t    flags;         //[104:111] Flags
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

//! \brief New task buffer representation  (Only the header part, N arguments follow the header)
typedef struct __attribute__ ((__packed__)) {
    uint16_t  numCopies;     //[0  :15 ] Number of copies after the task arguments
    uint16_t  numArgs;       //[16 :31 ] Number of arguments after this header
    uint32_t  archMask:24;   //[32 :55 ] Architecture mask in Picos style
    uint8_t   valid;         //[56 :63 ] Valid Entry
    uint64_t  parentID;      //[64 :127] Parent task identifier that is creating the task
    uint64_t  typeInfo;      //[128:191] Information of task type
} new_task_header_t;

//! \brief New task buffer representation (Only the argument part, repeated N times)
typedef struct __attribute__ ((__packed__)) {
    uint64_t  value:56;   //[0  :55 ] Argument value
    uint8_t   flags;      //[56 :63 ] Argument flags
} new_task_arg_t;

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

//! \brief Task argument representation in task_info
typedef struct __attribute__ ((__packed__)) {
    uint32_t argCached; //[0  :31 ] Flags
    uint32_t argID;     //[32 :63 ] Argument ID
    uint64_t argAddr;   //[64 :127] Address
} hw_task_arg_t;

//! \brief Task information for the accelerator
typedef struct __attribute__ ((__packed__)) {
    uint64_t parentTaskID;                   //[0  :63 ] Task identifier
    uint8_t commandCode;                     //[64 :71 ] Command code
    uint8_t commandArgs[7];                  //[72 :127] Command arguments
    uint64_t taskID;                         //[128:191] Task identifier
    // struct {
    //     uint64_t timerAddr;                  //[192:255] Timer address for instrumentation
    //     uint64_t bufferAddr;                 //[256:319] Buffer address to store instrumentation info
    // } profile;
} hw_task_header_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    hw_task_header_t *      hwTaskHeader;  ///< Pointer to the hw_task_header_t struct
    hw_task_arg_t *         hwTaskArgs;    ///< Pointer to the array of hw_task_arg_t structs
    size_t                  argsCnt;       ///< Number of arguments in the task
    acc_t *                 accel;         ///< Accelerator where the task will run
    ready_task_t            tmTask;        ///< Cached task information that will be sent to the TM
    xdma_buf_handle         taskHandle;    ///< Task buffer handle (only used if: argsCnt > DEF_HW_TASK_ARGS_LEN)
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint8_t *            _tasksBuff;         ///< Buffer to send the HW tasks
static uint8_t *            _tasksBuffPhy;      ///< Physical address of _tasksBuff
static xdma_buf_handle      _tasksBuffHandle;   ///< Handle of _tasksBuff in libxdma
static task_t *             _tasks;             ///< Array with internal task information
static int                  _readyQFd;          ///< File descriptior of ready queue
static int                  _finiQFd;           ///< File descriptior of finished queue
static int                  _newQFd;            ///< File descriptior of new queue
static int                  _remFiniQFd;        ///< File descriptior of remote finished queue
static int                  _ctrlFd;            ///< File descriptior of gpioctrl device
static ready_task_t        *_readyQueue;        ///< Buffer for the ready tasks
static fini_task_t         *_finiQueue;         ///< Buffer for the finished tasks
static uint64_t            *_newQueue;          ///< Buffer for the new tasks created in the HW
static size_t               _newQueueIdx;       ///< Reading index of the _newQueue
static rem_fini_task_t     *_remFiniQueue;      ///< Buffer for the remote finished tasks
static size_t               _remFiniQueueIdx;   ///< Writing index of the _remFiniQueue
static uint32_t volatile   *_taskmanagerRst;    ///< Register to reset Task Manager

static uint64_t             _insTimerAddr;      ///< Physical address of HW instrumentation timer
static size_t               _numInstrEvents;    ///< Number of instrumentation events for each accelerator buffer
static xtasks_ins_event    *_instrBuff;          ///< Buffer of instrumentation events
static xtasks_ins_event    *_instrBuffPhy;      ///< Physical address of _instrBuff
static xdma_buf_handle      _instrBuffHandle;   ///< Handle of _instrBuff in libxdma

static inline __attribute__((always_inline)) void resetTaskManager()
{
    //Nudge one register
    *_taskmanagerRst = 0x00;
    for ( int i = 0; i < 10; i++ ) i = i; // Lose some time
    *_taskmanagerRst = 0x01;
}

//FIXME: Refactor the command struct name/information
static xtasks_stat xtasksSubmitCommand(acc_t * acc, ready_task_t * command)
{
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

    // Copy the ready task structure into the BRAM
    memcpy(&_readyQueue[offset + idx], command, sizeof(ready_task_t));
    __sync_synchronize();
    _readyQueue[offset + idx].valid = VALID_ENTRY_MASK;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksInitHWIns(size_t const nEvents)
{
    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;
    size_t insBufferSize;
    unsigned long phyAddr;

    //At least we need 1 event + last event mark
    if (nEvents <= 1) return XTASKS_EINVAL;

    //Check if bitstrem has the HW instrumentation feature
    if (checkBitstremFeature("hw_instrumentation") == BIT_FEATURE_NO_AVAIL) {
        return XTASKS_ENOAV;
    }

    s = xdmaInitHWInstrumentation();
    if (s == XDMA_SUCCESS) {
        _insTimerAddr = (uint64_t)xdmaGetInstrumentationTimerAddr();
    } else {
        //Return as there's nothing to undo
        return XTASKS_ENOAV;
        //goto intrInitErr;
    }

    _numInstrEvents = nEvents;
    insBufferSize = nEvents*_numAccs*sizeof(xtasks_ins_event);
    s = xdmaAllocateHost((void **)&_instrBuff, &_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        goto instrAllocErr;
    }

    //get phy address
    s = xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        goto instrGetAddrErr;
    }
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    //Invalidate all entries
    for (size_t i = 0; i < insBufferSize; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    //Send the instrumentation buffer to each accelerator
    for (size_t i = 0; i < _numAccs; ++i) {
        //NOTE: Use the same instrmentation buffer to send the command
        uint64_t * data = (uint64_t *)(_instrBuff + _numInstrEvents*i);
        //Comand code + arguments
        data[0] = (_numInstrEvents << 8) | HW_TASK_CMD_SETUP_INS;
        //Instrumentation timer address
        data[1] = _insTimerAddr;
        //Instrumentation buffer phy address + Maximum event count (upper 16 bits)
        data[2] = (uintptr_t)(_instrBuffPhy + _numInstrEvents*i);

        ready_task_t cmd;
        cmd.taskInfoAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents*i);
        cmd.taskSize = 3; //< Command header + Timer address + Buffer address
        cmd.flags = READY_TASK_FLAG_AVAIL;
        cmd.accID = _accs[i].info.id;
        cmd.valid = FREE_ENTRY_MASK; //Not ready yet

        _accs[i].instrIdx = 0;
        ret = xtasksSubmitCommand(_accs + i, &cmd);
        if (ret != XTASKS_SUCCESS) {
            goto instrSendInit;
        }
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
        //Instrumentation is not initialized or has been already finished
        return XDMA_SUCCESS;
    }

    s0 = xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuff = NULL;
    _instrBuffPhy = NULL;
    s1 = xdmaFiniHWInstrumentation();

    return (s0 == XDMA_SUCCESS && s1 == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    //Always initialize instrumentation as disabled
    _instrBuff = NULL;
    _numInstrEvents = 0;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    //Check if bitstrem has the task manager feature
    if (checkBitstremFeature("task_manager") == BIT_FEATURE_NO_AVAIL) {
        PRINT_ERROR("OmpSs@FPGA Task Manager not available in the loaded FPGA bitstrem");
        return XTASKS_ENOAV;
    }

    //Open libxdma
    s = xdmaOpen();
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        if (s == XDMA_ENOENT) {
            PRINT_ERROR("xdmaOpen failed. Check if xdma device exist in the system");
        } else if (s == XDMA_EACCES) {
            PRINT_ERROR("xdmaOpen failed. Current user cannot access xdma device");
        } else {
            PRINT_ERROR("xdmaOpen failed");
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
            _accs[i].info.maxTasks = READY_QUEUE_ACC_LEN;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
            _accs[i].readyQueueIdx = 0;
            _accs[i].finiQueueIdx = 0;
            _accs[i].lock = 0;
        }
    }
    fclose(accMapFile);
    free(buffer);
    _numAccs = (total < _numAccs) ? total : _numAccs;

    if (retFscanf != EOF && retFscanf != 0) {
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
    _ctrlFd = open(TASKMANAGER_RST_PATH, O_RDWR, (mode_t) 0600);
    if (_readyQFd < 0 || _finiQFd < 0 || _ctrlFd < 0) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open taskmanager device files");
        goto INIT_ERR_OPEN_COMM;
    }

    if (checkBitstremFeature("ext_task_manager") == BIT_FEATURE_NO_AVAIL) {
	//Do not try to open the extended TM queues as we know that are not available
        _newQFd = -1;
	_remFiniQFd = -1;
    } else {
        _newQFd = open(NEW_QUEUE_PATH, O_RDWR, (mode_t) 0600);
        if (_newQFd < 0 && errno != ENOENT) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open taskmanager new queue device file");
            goto INIT_ERR_OPEN_NEW;
        }
        _remFiniQFd = open(REMFINI_QUEUE_PATH, O_RDWR, (mode_t) 0600);
        if (_remFiniQFd < 0 && errno != ENOENT) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open taskmanager remote finished queue device file");
            goto INIT_ERR_OPEN_REMFINI;
        }
    }

    _readyQueue = (ready_task_t *)mmap(NULL, sizeof(ready_task_t)*READY_QUEUE_LEN,
        PROT_READ | PROT_WRITE, MAP_SHARED, _readyQFd, 0);
    if (_readyQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map ready queue of Task Manager");
        goto INIT_ERR_MMAP_READY;
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
        goto INIT_ERR_MMAP_FINI;
    }

    //If any, invalidate finished tasks in finiQueue
    for (size_t idx = 0; idx < FINI_QUEUE_LEN; ++idx) {
        _finiQueue[idx].valid = FREE_ENTRY_MASK;
    }

    _newQueueIdx = 0;
    if (_newQFd == -1) {
       _newQueue = NULL;
    } else {
        _newQueue = (uint64_t *)mmap(NULL, sizeof(uint64_t)*NEW_QUEUE_LEN,
            PROT_READ | PROT_WRITE, MAP_SHARED, _newQFd, 0);
        if (_newQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map new queue of Task Manager");
            goto INIT_ERR_MAP_NEW;
        }

        //If any, invalidate tasks in newQueue
        _memset(_newQueue, 0, NEW_QUEUE_LEN*sizeof(uint64_t));
    }

    _remFiniQueueIdx = 0;
    if (_remFiniQFd == -1) {
       _remFiniQueue = NULL;
    } else {
        _remFiniQueue = (rem_fini_task_t *)mmap(NULL, sizeof(rem_fini_task_t)*REMFINI_QUEUE_LEN,
            PROT_READ | PROT_WRITE, MAP_SHARED, _remFiniQFd, 0);
        if (_remFiniQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map remote finished queue of Task Manager");
            goto INIT_ERR_MAP_REMFINI;
        }

        //If any, invalidate tasks in remFiniQueue
        _memset(_remFiniQueue, 0, NEW_QUEUE_LEN*sizeof(uint64_t));
    }

    _taskmanagerRst = (uint32_t *)mmap(NULL, sizeof(uint32_t), PROT_READ | PROT_WRITE,
        MAP_SHARED, _ctrlFd, 0);
    if (_taskmanagerRst == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map control registers of Task Manager");
        goto INIT_ERR_MMAP_RST;
    }

    //Reset _readyQueue and _finiQueue indexes
    resetTaskManager();

    //Allocate the task_info buffer
    s = xdmaAllocateHost((void**)&_tasksBuff, &_tasksBuffHandle,
        NUM_RUN_TASKS*DEF_HW_TASK_SIZE);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate kernel memory for task information");
        goto INIT_ERR_ALLOC_INFO;
    }
    unsigned long phyAddr;
    s = xdmaGetDeviceAddress(_tasksBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("Cannot get physical address of task info. region");
        goto INIT_ERR_GETADDR_INFO;
    }
    _tasksBuffPhy = (uint8_t *)phyAddr;

    //Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS*sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto INIT_ERR_ALLOC_TASKS;
    }
    memset(_tasks, 0, NUM_RUN_TASKS*sizeof(task_t));

    return ret;

    //Error handling code
    INIT_ERR_ALLOC_TASKS:
    INIT_ERR_GETADDR_INFO:
        xdmaFree(_tasksBuffHandle);
    INIT_ERR_ALLOC_INFO:
        munmap((void *)_taskmanagerRst, sizeof(uint32_t));
    INIT_ERR_MMAP_RST:
        if (_remFiniQueue != NULL)
            munmap(_remFiniQueue, sizeof(rem_fini_task_t)*REMFINI_QUEUE_LEN);
    INIT_ERR_MAP_REMFINI:
        if (_newQueue != NULL)
            munmap(_newQueue, sizeof(uint64_t)*NEW_QUEUE_LEN);
    INIT_ERR_MAP_NEW:
        munmap(_finiQueue, sizeof(fini_task_t)*FINI_QUEUE_LEN);
    INIT_ERR_MMAP_FINI:
        munmap(_readyQueue, sizeof(ready_task_t)*READY_QUEUE_LEN);
    INIT_ERR_MMAP_READY:
        if (_remFiniQFd != -1)
            close(_remFiniQFd);
    INIT_ERR_OPEN_REMFINI:
        if (_newQFd != -1)
            close(_newQFd);
    INIT_ERR_OPEN_NEW:
    INIT_ERR_OPEN_COMM:
        close(_ctrlFd);
        close(_finiQFd);
        close(_readyQFd);
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
    free(_tasks);

    //Free tasks buffer
    xdma_status s = xdmaFree(_tasksBuffHandle);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }
    _tasksBuff = NULL;
    _tasksBuffPhy = NULL;

    //Finialize the HW instrumentation if needed
    if (xtasksFiniHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    //Unmap the Task Manager queues
    int statusRd, statusFi, statusNw, statusRFi, statusCtrl;
    statusCtrl = munmap((void *)_taskmanagerRst, sizeof(uint32_t));
    statusRFi = _remFiniQueue != NULL ? munmap(_remFiniQueue, sizeof(rem_fini_task_t)*REMFINI_QUEUE_LEN) : 0;
    statusNw = _newQueue != NULL ? munmap(_newQueue, sizeof(uint64_t)*NEW_QUEUE_LEN) : 0;
    statusFi = munmap(_finiQueue, sizeof(fini_task_t)*FINI_QUEUE_LEN);
    statusRd = munmap(_readyQueue, sizeof(ready_task_t)*READY_QUEUE_LEN);
    if (statusRd == -1 || statusFi == -1 || statusNw == -1 || statusRFi == -1 || statusCtrl == -1) {
        ret = XTASKS_EFILE;
    }

    statusCtrl = close(_ctrlFd);
    statusRFi = _remFiniQFd != -1 ? close(_remFiniQFd) : 0;
    statusNw = _newQFd != -1 ? close(_newQFd) : 0;
    statusFi = close(_finiQFd);
    statusRd = close(_readyQFd);
    if (statusRd == -1 || statusFi == -1 || statusNw == -1 || statusRFi == -1 || statusCtrl == -1) {
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
        if (_tasks[i].hwTaskHeader == NULL) {
            hw_task_header_t * header = (hw_task_header_t *)(&_tasksBuff[i*DEF_HW_TASK_SIZE]);
            if (__sync_bool_compare_and_swap(&_tasks[i].hwTaskHeader, 0, header)) {
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
    //_tasks[idx].hwTaskHeader = &_tasksBuff[idx*DEF_HW_TASK_SIZE]; //NOTE: Done in getFreeTaskEntry()
    _tasks[idx].hwTaskArgs = (hw_task_arg_t *)
        (&_tasksBuff[idx*DEF_HW_TASK_SIZE + sizeof(hw_task_header_t)]);
    _tasks[idx].argsCnt = 0;
    _tasks[idx].accel = accel;
    _tasks[idx].tmTask.taskInfoAddr = (uintptr_t)(&_tasksBuffPhy[idx*DEF_HW_TASK_SIZE]);
    _tasks[idx].tmTask.argsBitmask = 0xFFFF; //All will be ready
    //_tasks[idx].tmTask.size = ?¿; //Computed at submit
    _tasks[idx].tmTask.accID = accel->info.id;
    _tasks[idx].tmTask.valid = FREE_ENTRY_MASK; //Not ready yet
    _tasks[idx].tmTask.flags = READY_TASK_FLAG_BUSY;

    hw_task_header_t taskHeader;
    taskHeader.parentTaskID = (uintptr_t)(parent);
    taskHeader.commandCode = HW_TASK_CMD_EXEC_TASK;
    taskHeader.commandArgs[HW_TASK_CMD_ARGS_COMPUTE_OFFSET] = compute;
    taskHeader.commandArgs[HW_TASK_CMD_ARGS_DEST_ID_OFFSET] = HW_TASK_DEST_ID_TM;
    taskHeader.taskID = (uintptr_t)(&_tasks[idx]);
    // taskHeader.profile.timerAddr = _insTimerAddr;
    // taskHeader.profile.bufferAddr = instrumentData;
    memcpy(_tasks[idx].hwTaskHeader, &taskHeader, sizeof(hw_task_header_t));

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    task_t * task = (task_t *)(*handle);
    *handle = NULL;
    if (task->argsCnt > DEF_HW_TASK_ARGS_LEN) {
        //Task is in extended mode
        xdmaFree(task->taskHandle);
        task->argsCnt = 0;
    }
    __sync_synchronize(); //Execute previous operations before the next instruction
    task->hwTaskHeader = NULL;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    if (task->argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (task->argsCnt == DEF_HW_TASK_ARGS_LEN) {
        //Entering in extended mode
        hw_task_header_t * prevHeader = task->hwTaskHeader;
        xdma_status s;
        s = xdmaAllocateHost((void**)(&task->hwTaskHeader), &task->taskHandle,
            EXT_HW_TASK_SIZE);
        if (s != XDMA_SUCCESS) {
            return XTASKS_ENOMEM;
        }
        memcpy(task->hwTaskHeader, prevHeader, DEF_HW_TASK_SIZE); //< Move the hw task header and args
        //Update the task info
        task->hwTaskArgs = (hw_task_arg_t *)(task->hwTaskHeader + 1);
        unsigned long dmaAddr;
        s = xdmaGetDeviceAddress(task->taskHandle, &dmaAddr);
        if (s != XDMA_SUCCESS) {
            xdmaFree(task->taskHandle);
            return XTASKS_ERROR;
        }
        task->tmTask.taskInfoAddr = (uint64_t)dmaAddr;
    }

    size_t idx = task->argsCnt++;
    task->hwTaskArgs[idx].argCached = flags;
    task->hwTaskArgs[idx].argID = id;
    task->hwTaskArgs[idx].argAddr = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    if (num > EXT_HW_TASK_ARGS_LEN - task->argsCnt) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (num > (DEF_HW_TASK_ARGS_LEN - task->argsCnt) && task->argsCnt <= DEF_HW_TASK_ARGS_LEN) {
        //Entering in extended mode because:
        // 1) The number of args to add does not fit in current allocated space
        // 2) We are not in extended mode
        // 3) The number of args will fit in extended mode
        hw_task_header_t * prevHeader = task->hwTaskHeader;
        xdma_status s;
        s = xdmaAllocateHost((void**)(&task->hwTaskHeader), &task->taskHandle,
            EXT_HW_TASK_SIZE);
        if (s != XDMA_SUCCESS) {
            return XTASKS_ENOMEM;
        }
        memcpy(task->hwTaskHeader, prevHeader, DEF_HW_TASK_SIZE); //< Move the hw task header and args
        //Update the task info
        task->hwTaskArgs = (hw_task_arg_t *)(task->hwTaskHeader + 1);
        unsigned long dmaAddr;
        s = xdmaGetDeviceAddress(task->taskHandle, &dmaAddr);
        if (s != XDMA_SUCCESS) {
            xdmaFree(task->taskHandle);
            return XTASKS_ERROR;
        }
        task->tmTask.taskInfoAddr = (uint64_t)dmaAddr;
    }

    for (size_t i = 0, idx = task->argsCnt; i < num; ++i, ++idx) {
        task->hwTaskArgs[idx].argCached = flags;
        task->hwTaskArgs[idx].argID = idx;
        task->hwTaskArgs[idx].argAddr = values[i];
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

    // Copy the task info structure into kernel memory
    // NOTE: For now, already there

    // Update the task/command size
    task->tmTask.taskSize = (sizeof(hw_task_header_t) - sizeof(uint64_t) /*taskID not considered*/ +
        task->argsCnt*sizeof(hw_task_arg_t)) / sizeof(uint64_t);

    return xtasksSubmitCommand(acc, &task->tmTask);
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

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event * events, size_t const maxCount)
{
    acc_t * acc = (acc_t *)(accel);
    xtasks_ins_event * accBuffer = _instrBuff + acc->info.id*_numInstrEvents;
    size_t count, i;

    if (events == NULL || (acc - _accs) >= _numAccs || maxCount <= 0) return XTASKS_EINVAL;
    else if (_instrBuff == NULL) return XTASKS_ENOAV;

    if (__sync_lock_test_and_set(&acc->lock, 1)) {
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
        __sync_lock_release(&acc->lock);

        if (i == count && count < maxCount) {
            //Put the last event identifier
            events[i].eventType = XTASKS_EVENT_TYPE_INVALID;
        }
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask ** task)
{
    if (_newQueue == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the manager finished queue
    size_t idx, next, taskSize;
    new_task_header_t * hwTaskHeader;
    do {
        idx = _newQueueIdx;
        hwTaskHeader = (new_task_header_t *)(&_newQueue[idx]);
        if (hwTaskHeader->valid != VALID_ENTRY_MASK) {
            return XTASKS_PENDING;
        }
        taskSize = (
          sizeof(new_task_header_t) +
          sizeof(new_task_arg_t)*hwTaskHeader->numArgs +
          sizeof(new_task_copy_t)*hwTaskHeader->numCopies)/
          sizeof(uint64_t);
        next = (idx+taskSize)%NEW_QUEUE_LEN;
    } while ( !__sync_bool_compare_and_swap(&_newQueueIdx, idx, next) );

    //Extract the information from the new buffer
    *task = realloc(*task,
        sizeof(xtasks_newtask) +
        sizeof(xtasks_newtask_arg)*hwTaskHeader->numArgs +
        sizeof(xtasks_newtask_copy)*hwTaskHeader->numCopies);
    (*task)->args = (xtasks_newtask_arg *)(*task + 1);
    (*task)->numArgs = hwTaskHeader->numArgs;
    (*task)->copies = (xtasks_newtask_copy *)((*task)->args + (*task)->numArgs);
    (*task)->numCopies = hwTaskHeader->numCopies;
    (*task)->architecture = hwTaskHeader->archMask;

    idx = (idx+1)%NEW_QUEUE_LEN; //NOTE: new_task_header_t->parentID field is the 2nd word
    task_t * parentTask = (task_t *)((uintptr_t)(_newQueue[idx]));
    (*task)->parentId = parentTask->id; //< The external parent identifier must be returned (not the xtasks internal one)
    _newQueue[idx] = 0; //< Cleanup the memory position

    idx = (idx+1)%NEW_QUEUE_LEN; //NOTE: new_task_header_t->typeInfo field is the 3rd word
    (*task)->typeInfo = _newQueue[idx];
    _newQueue[idx] = 0; //< Cleanup the memory position

    for (size_t i = 0; i < (*task)->numArgs; ++i) {
        //Check that arg pointer is not out of bounds
        idx = (idx+1)%NEW_QUEUE_LEN;
        new_task_arg_t * hwTaskArg = (new_task_arg_t *)(&_newQueue[idx]);

        //Parse the arg information
        (*task)->args[i].value = hwTaskArg->value;
        (*task)->args[i].flags = hwTaskArg->flags;

        //Cleanup the memory position
        _newQueue[idx] = 0;
    }

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

    //Free the buffer slot
    //NOTE: This word cannot be set to 0 as the task size information must be keept
    __sync_synchronize();
    hwTaskHeader->valid = FREE_ENTRY_MASK;

    return XDMA_SUCCESS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_handle const parent, size_t count)
{
    task_t * task = (task_t *)(parent);

    if (task == NULL) {
        return XTASKS_EINVAL;
    }

    // Get an empty slot into the TM remote finished queue
    size_t idx, next;
    do {
        idx = _remFiniQueueIdx;
        if (_remFiniQueue[idx].valid == VALID_ENTRY_MASK) {
            return XTASKS_ENOENTRY;
        }
        next = (idx+1)%REMFINI_QUEUE_LEN;
    } while ( !__sync_bool_compare_and_swap(&_remFiniQueueIdx, idx, next) );

    // Copy the information into the queue
    _remFiniQueue[idx].taskID = (uintptr_t)(task);
    _remFiniQueue[idx].components = count;
    __sync_synchronize();
    _remFiniQueue[idx].valid = VALID_ENTRY_MASK;

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
ready_task_t readyQueue(size_t const idx)
{
    return _readyQueue[idx];
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

hw_task_header_t tasksBufferHeader(size_t const idx)
{
    hw_task_header_t * tmp = (hw_task_header_t *)(&_tasksBuff[idx*DEF_HW_TASK_SIZE]);
    return *tmp;
}

hw_task_arg_t tasksBufferArg(size_t const tIdx, size_t const aIdx)
{
    hw_task_arg_t * tmp = (hw_task_arg_t *)(&_tasksBuff[tIdx*DEF_HW_TASK_SIZE + sizeof(hw_task_header_t)]);
    return tmp[aIdx];
}

xtasks_ins_event accelGetEvent(size_t const aIdx, size_t const evIdx) {
    return _instrBuff[aIdx*_numInstrEvents + evIdx];
}

void accelPrintInstrBuffer(size_t const aIdx) {
    xtasks_ins_event *event = _instrBuff + aIdx*_numInstrEvents;
    fprintf(stderr, "timestamp, accid, eventid, value\n");
    while ( event->eventId != XTASKS_EVENT_TYPE_INVALID ) {
        fprintf(stderr, "%lu,\t%u,\t%u,\t%lu\n", event->timestamp,
                event->eventType, event->eventId, event->value);
        event++;
    }
}

#endif
