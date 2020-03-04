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

#include "libxtasks.h"
#include "util/common.h"
#include "util/ticket-lock.h"
#include "platform.h"

#include <libxdma.h>
#include <libxdma_version.h>
#include <stdio.h>
#include <stddef.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>

#include <assert.h>

#include <admxrc3.h>

#define DEF_ACCS_LEN            8               ///< Def. allocated slots in the accs array

#define CMD_IN_QUEUE_LEN        1024            ///< Total number of entries in the cmd_in_queue
#define CMD_IN_SUBQUEUE_LEN     64              ///< Number of entries in the sub-queue of cmd_in_queue for one accelerator
#define CMD_OUT_QUEUE_LEN       1024            ///< Total number of entries in the cmd_out_queue
#define CMD_OUT_SUBQUEUE_LEN    64              ///< Number of entries in the sub-queue of cmd_out_queue for one accelerator
#define SPWN_OUT_QUEUE_LEN      1024            //NOTE: Each element is a uint64_t (the number of arguments for a task is unknown)
#define SPWN_IN_QUEUE_LEN       1024            ///< Total number of entries in the spawn_out_in_queue
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

#define CMD_IN_QUEUE_ADDR 0x00004000
#define CMD_OUT_QUEUE_ADDR 0x00008000
#define HWRUNTIME_RESET_ADDRESS 0x0000C000
#define SPAWN_OUT_QUEUE_ADDRESS 0x00014000
#define SPAWN_IN_QUEUE_ADDRESS 0x000018000

#define HWCOUNTER_ADDRESS 0x00010000

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
# error Installed libxdma is not supported (use >= 3.0)
#endif

//! \brief Platform and Backend strings
const char _platformName[] = "alpha_data";
const char _backendName[] = "hwruntime";

//! \brief New task buffer representation  (Only the header part, N arguments follow the header)
typedef struct __attribute__ ((__packed__)) {
    uint8_t   _padding;      //[0  :7  ]
    uint8_t   numArgs;       //[8  :15 ] Number of arguments after this header
    uint8_t   numDeps;       //[16 :23 ] Number of dependencies after the task arguments
    uint8_t   numCopies;     //[24 :31 ] Number of copies after the task dependencies
    uint32_t  archMask:24;   //[32 :55 ] Architecture mask in Picos style
    uint8_t   valid;         //[56 :63 ] Valid Entry
    uint64_t  taskID;        //[64 :127] Task identifier
    uint64_t  parentID;      //[128:191] Parent task identifier that is creating the task
    uint64_t  typeInfo;      //[192:255] Information of task type
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
    uint8_t  _padding[7];
    uint8_t  valid;       //[56 :63 ] Valid Entry
    uint64_t taskID;      //[64 :127] Task identifier
    uint64_t parentID;    //[128:191] Parent task identifier that created the tasks
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
    unsigned short volatile  instrLock;         ///< Lock for atomic operations over instrumentation buffers
} acc_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id          id;            ///< External task identifier
    cmd_header_t *          cmdHeader;     ///< Pointer to the cmd_header_t struct
    cmd_exec_task_arg_t *   cmdExecArgs;   ///< Pointer to the array of cmd_exec_task_arg_t structs
    acc_t *                 accel;         ///< Accelerator where the task will run
    uint8_t                 extSize:1;     ///< Whether the space available in args is extended or not
    uint8_t                 periTask:1;    ///< Whether the tasks is a periodic task or not
} task_t;

static int _init_cnt = 0;   ///< Counter of calls to init/fini
static size_t   _numAccs;   ///< Number of accelerators in the system
static acc_t *  _accs;      ///< Accelerators data
static uint8_t *            _cmdExecTaskBuff;   ///< Buffer to send the HW tasks
static task_t *             _tasks;             ///< Array with internal task information
static uint64_t            *_cmdInQueue;        ///< Command IN queue
static uint64_t            *_cmdOutQueue;       ///< Command OUT queue
static uint64_t            *_spawnOutQueue;     ///< Spawn OUT queue (buffer of FPGA spawned tasks)
static size_t               _spawnOutQueueIdx;  ///< Reading index of the _spawnOutQueue
static uint64_t            *_spawnInQueue;      ///< Spawn IN queue (buffer of finished FPGA spawned tasks)
static size_t               _spawnInQueueIdx;   ///< Writing index of the _spawnInQueue
static uint32_t volatile   *_hwruntimeRst;      ///< Register to reset HW runtime

static ticketLock_t         _bufferTicket;      ///< Lock to atomically access PCI direct slave
static ADMXRC3_HANDLE       _hDevice;           ///< Alphadata device handle

static size_t               _numInstrEvents;    ///< Number of instrumentation events for each accelerator buffer
static xtasks_ins_event    *_instrBuffPhy;      ///< Physical address of the FPGA side instrumentation buffer
static xtasks_ins_event    *_instrBuff;         ///< Buffer of instrumentation events (host side)
static xdma_buf_handle      _instrBuffHandle;   ///< Handle of _instrBuff in libxdma
static uint64_t*            _instrCounter;      ///< Hardware counter

static inline __attribute__((always_inline)) void resetHWRuntime()
{
    //Nudge one register
    *_hwruntimeRst = 0x01;
    for ( int i = 0; i < 10; i++ ) i = i; // Lose some time
    *_hwruntimeRst = 0x00;
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
                size_t const cmdNumWords = getCmdLength(cmdHeaderPtr);
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
        //memcpy(&_cmdInQueue[offset + idx + 1], command + 1, count*sizeof(uint64_t));
        if (ADMXRC3_Write(_hDevice, 1, 0, CMD_IN_QUEUE_ADDR + (offset + idx + 1)*sizeof(uint64_t), count*sizeof(uint64_t), command + 1) != ADMXRC3_SUCCESS) {
            perror("Error submitting command\n");
        }
        if ((length - 1) > count) {
            //memcpy(&_cmdInQueue[offset], command + 1 + count, (length - count)*sizeof(uint64_t));
            if (ADMXRC3_Write(_hDevice, 1, 0, CMD_IN_QUEUE_ADDR + offset*sizeof(uint64_t), (length - count)*sizeof(uint64_t), command + 1 + count) != ADMXRC3_SUCCESS) {
                perror("Error submitting command\n");
            }
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
    size_t insBufferSize;
    if (nEvents <= 1) return XTASKS_EINVAL;
    //Check if bitstream has the HW instrumentation feature
    bit_feature_t feature = checkbitstreamFeature("hwcounter");
    if (feature == BIT_FEATURE_NO_AVAIL || feature == BIT_FEATURE_UNKNOWN) {
        return XTASKS_ENOAV;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents*_numAccs*sizeof(xtasks_ins_event);

    xdma_status s = xdmaAllocate(&_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ENOAV;
    }
    unsigned long phyAddr;
    xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    //Invalidate all entries
    _instrBuff = (xtasks_ins_event*)malloc(insBufferSize);
    for (int i = 0; i < _numInstrEvents*_numAccs; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    s = xdmaMemcpy(_instrBuff, _instrBuffHandle, insBufferSize, 0, XDMA_TO_DEVICE);

    if (s != XDMA_SUCCESS) {
        free(_instrBuff);
        xdmaFree(_instrBuffHandle);
        _numInstrEvents = 0;
        _instrBuffPhy = NULL;
        _instrBuff = NULL;
        return XTASKS_ERROR;
    }

    //Send the instrumentation buffer to each accelerator
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    memcpy((void *)cmd.header.commandArgs, (void *)&_numInstrEvents, CMD_SETUP_INS_ARGS_NUMEVS_BYTES);
    for (size_t i = 0; i < _numAccs; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents*i);

        xtasks_stat ret = xtasksSubmitCommand(_accs + i, (uint64_t *)&cmd, sizeof(cmd_setup_hw_ins_t)/sizeof(uint64_t));
        if (ret != XTASKS_SUCCESS) {
            free(_instrBuff);
            xdmaFree(_instrBuffHandle);
            _numInstrEvents = 0;
            _instrBuffPhy = NULL;
            _instrBuff = NULL;
            return ret;
        }

        _accs[i].instrIdx = 0;
        _accs[i].instrLock = 0;
    }

    ADMXRC3_STATUS stat = ADMXRC3_MapWindow(_hDevice, 1, HWCOUNTER_ADDRESS, sizeof(uint64_t), (void**)&_instrCounter);
    if (stat != ADMXRC3_SUCCESS) {
        free(_instrBuff);
        xdmaFree(_instrBuffHandle);
        _numInstrEvents = 0;
        _instrBuffPhy = NULL;
        _instrBuff = NULL;
        return XTASKS_ERROR;
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksFiniHWIns()
{
    if (_instrBuff == NULL)
        return XTASKS_ERROR;
    free(_instrBuff);
    ADMXRC3_STATUS stat = ADMXRC3_UnmapWindow(_hDevice, _instrCounter);
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuffPhy = NULL;
    _instrBuff = NULL;
    return stat == ADMXRC3_SUCCESS ? XTASKS_SUCCESS:XTASKS_ERROR;
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    ADMXRC3_STATUS admxrc3_status = ADMXRC3_Open(0, &_hDevice);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_ERROR;
        perror("Error opening alphadata device");
        goto INIT_ERR_0;
    }

    //Check if bitstream is compatible
    bit_compatibility_t compat = checkbitstreamCompatibility(_hDevice);
    if (compat == BIT_NO_COMPAT || compat == BIT_COMPAT_UNKNOWN) {
        printErrorBitstreamCompatibility();
	ret = XTASKS_ENOAV;
        goto INIT_ERR_1;
    }

    //Check if bitstream has the hwruntime feature
    bit_feature_t feature = checkbitstreamFeature("hwruntime");
    if (feature == BIT_FEATURE_NO_AVAIL) {
        PRINT_ERROR("HW runtime not available in the loaded FPGA bitstream");
        ret = XTASKS_ENOAV;
	goto INIT_ERR_1;
    }

    //Initialize xdma memory subsystem
    s = xdmaInitMem();
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        goto INIT_ERR_1;
    }

    //Preallocate accelerators array
    _numAccs = DEF_ACCS_LEN;
    _accs = malloc(sizeof(acc_t)*_numAccs);
    if (_accs == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for accelerators info");
        goto INIT_ERR_2;
    }

    //Generate the configuration file path
    char * buffer = getConfigFilePath();
    if (buffer == NULL) {
        ret = XTASKS_EFILE;
        printErrorMsgCfgFile();
        goto INIT_ERR_3;
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(buffer, "r");
    if (accMapFile == NULL) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot open FPGA configuration file");
        free(buffer);
        goto INIT_ERR_3;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("First line of FPGA configuration file is not valid");
        free(buffer);
        fclose(accMapFile);
        goto INIT_ERR_3;
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
        goto INIT_ERR_3;
    }

    ticketLockInit(&_bufferTicket);

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, CMD_IN_QUEUE_ADDR, sizeof(uint64_t)*CMD_IN_QUEUE_LEN, (void**)&_cmdInQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_in_queue");
        goto INIT_ERR_MMAP_READY;
    }

    //If any, invalidate commands in cmd_in queue
    //_memset(_cmdInQueue, 0, CMD_IN_QUEUE_LEN*sizeof(uint64_t));
    for (int i = 0; i < CMD_IN_QUEUE_LEN; ++i) {
        _cmdInQueue[i] = 0;
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, CMD_OUT_QUEUE_ADDR, sizeof(uint64_t)*CMD_OUT_QUEUE_LEN, (void**)&_cmdOutQueue);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_out_queue");
        goto INIT_ERR_MMAP_FINI;
    }

    //If any, invalidate commands in cmd_out queue
    //_memset(_cmdOutQueue, 0, CMD_OUT_QUEUE_LEN*sizeof(uint64_t));
    for (int i = 0; i < CMD_OUT_QUEUE_LEN; ++i) {
        _cmdOutQueue[i] = 0;
    }

    feature = checkbitstreamFeature("hwruntime_ext");

    if (feature == BIT_FEATURE_NO_AVAIL)
        _spawnOutQueue = NULL;
    else {
        admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, SPAWN_OUT_QUEUE_ADDRESS, sizeof(uint64_t)*SPWN_OUT_QUEUE_LEN, (void**)&_spawnOutQueue);
        if (admxrc3_status != ADMXRC3_SUCCESS) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_out_queue");
            goto INIT_ERR_MAP_NEW;
        }

        //If any, invalidate tasks in newQueue
        //_memset(_spawnOutQueue, 0, SPWN_OUT_QUEUE_LEN*sizeof(uint64_t));
        for (int i = 0; i < SPWN_OUT_QUEUE_LEN; ++i) {
            _spawnOutQueue[i] = 0;
        }
    }

    if (feature == BIT_FEATURE_NO_AVAIL)
        _spawnInQueue = NULL;
    else {
        admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, SPAWN_IN_QUEUE_ADDRESS, sizeof(uint64_t)*SPWN_IN_QUEUE_LEN, (void**)&_spawnInQueue);
        if (admxrc3_status != ADMXRC3_SUCCESS) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_in_queue");
            goto INIT_ERR_MAP_REMFINI;
        }

        //If any, invalidate tasks in remFiniQueue
        //_memset(_spawnInQueue, 0, SPWN_IN_QUEUE_LEN*sizeof(uint64_t));
        for (int i = 0; i < SPWN_IN_QUEUE_LEN; ++i) {
            _spawnInQueue[i] = 0;
        }
    }

    admxrc3_status = ADMXRC3_MapWindow(_hDevice, 1, HWRUNTIME_RESET_ADDRESS, sizeof(uint32_t), (void**)&_hwruntimeRst);
    if (admxrc3_status != ADMXRC3_SUCCESS) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map control registers of Task Manager");
        goto INIT_ERR_MMAP_RST;
    }

    resetHWRuntime();

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
        _tasks[idx].cmdHeader = (cmd_header_t *)&_cmdExecTaskBuff[idx*DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)0;
        _tasks[idx].extSize = 0;
        _tasks[idx].periTask = 0;
    }

    return ret;

    //Error handling code
        free(_cmdExecTaskBuff);
    INIT_ERR_ALLOC_EXEC_TASKS_BUFF:
        free(_tasks);
    INIT_ERR_ALLOC_TASKS:
        ADMXRC3_UnmapWindow(_hDevice, (void*)_hwruntimeRst);
    INIT_ERR_MMAP_RST:
        if (_spawnInQueue != NULL)
            ADMXRC3_UnmapWindow(_hDevice, _spawnInQueue);
    INIT_ERR_MAP_REMFINI:
        if (_spawnOutQueue != NULL)
            ADMXRC3_UnmapWindow(_hDevice, _spawnOutQueue);
    INIT_ERR_MAP_NEW:
        ADMXRC3_UnmapWindow(_hDevice, _cmdOutQueue);
    INIT_ERR_MMAP_FINI:
        ADMXRC3_UnmapWindow(_hDevice, _cmdInQueue);
    INIT_ERR_MMAP_READY:
        ticketLockFini(&_bufferTicket);
    INIT_ERR_3:
        free(_accs);
        _numAccs = 0;
    INIT_ERR_2:
        xdmaFiniMem();
    INIT_ERR_1:
        ADMXRC3_Close(_hDevice);
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

    //Unmap the hwruntime queues
    ADMXRC3_STATUS statusRd, statusFi, statusNw, statusRFi, statusCtrl;
    statusCtrl = ADMXRC3_UnmapWindow(_hDevice, (void*)_hwruntimeRst);
    statusFi = ADMXRC3_UnmapWindow(_hDevice, _cmdOutQueue);
    statusNw = statusRFi = ADMXRC3_SUCCESS;
    if (_spawnOutQueue != NULL)
        statusNw = ADMXRC3_UnmapWindow(_hDevice, _spawnOutQueue);
    if (_spawnInQueue != NULL)
        statusRFi = ADMXRC3_UnmapWindow(_hDevice, _spawnInQueue);
    statusRd = ADMXRC3_UnmapWindow(_hDevice, _cmdInQueue);
    if (statusRd != ADMXRC3_SUCCESS || statusFi != ADMXRC3_SUCCESS || statusCtrl != ADMXRC3_SUCCESS ||
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
    xtasks_task_id const parent, xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
    acc_t * accel = (acc_t *)accId;
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].accel = accel;
    _tasks[idx].periTask = 0;
    cmd_exec_task_header_t * cmdHeader = (cmd_exec_task_header_t *)_tasks[idx].cmdHeader;
    _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_EXEC_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_TM;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(&_tasks[idx]);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps,
    unsigned int const period, xtasks_task_handle * handle)
{
    acc_t * accel = (acc_t *)accId;
    int idx = getFreeTaskEntry();
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    _tasks[idx].id = id;
    _tasks[idx].accel = accel;
    _tasks[idx].periTask = 1;
    cmd_peri_task_header_t * cmdHeader = (cmd_peri_task_header_t *)_tasks[idx].cmdHeader;
    _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_PERI_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_TM;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(&_tasks[idx]);
    cmdHeader->numReps = numReps;
    cmdHeader->period = period;

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
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (argsCnt == DEF_EXEC_TASK_ARGS_LEN) {
        //Entering in extended mode
        cmd_header_t * prevHeader = task->cmdHeader;
        task->cmdHeader = (cmd_header_t *)malloc(EXT_HW_TASK_SIZE);
        if (task->cmdHeader == NULL) {
            task->cmdHeader = prevHeader;
            return XTASKS_ENOMEM;
        }
        task->extSize = 1;
        task->cmdExecArgs = (cmd_exec_task_arg_t *)(((unsigned char *)task->cmdHeader) +
          (task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t)));
        memcpy(task->cmdHeader, prevHeader, DEF_EXEC_TASK_SIZE); //< Move the hw task header and args
    }

    argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET]++;
    task->cmdExecArgs[argsCnt].flags = flags;
    task->cmdExecArgs[argsCnt].id = id;
    task->cmdExecArgs[argsCnt].value = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
    task_t * task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (num > EXT_HW_TASK_ARGS_LEN - argsCnt) {
        //Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (num > (DEF_EXEC_TASK_ARGS_LEN - argsCnt) && argsCnt <= DEF_EXEC_TASK_ARGS_LEN) {
        //Entering in extended mode because:
        // 1) The number of args to add does not fit in current allocated space
        // 2) We are not in extended mode
        // 3) The number of args will fit in extended mode
        cmd_header_t * prevHeader = task->cmdHeader;
        task->cmdHeader = (cmd_header_t *)malloc(EXT_HW_TASK_SIZE);
        if (task->cmdHeader == NULL) {
            task->cmdHeader = prevHeader;
            return XTASKS_ENOMEM;
        }
        task->extSize = 1;
        task->cmdExecArgs = (cmd_exec_task_arg_t *)(((unsigned char *)task->cmdHeader) +
          (task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t)));
        memcpy(task->cmdHeader, prevHeader, DEF_EXEC_TASK_SIZE); //< Move the hw task header and args
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
    task_t * task = (task_t *)(handle);
    acc_t * acc = task->accel;

    if (task == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }
    // Update the task/command information
    task->cmdHeader->valid = QUEUE_VALID;

    //NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numHeaderBytes = task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t);
    size_t const numCmdWords = (numHeaderBytes + sizeof(cmd_exec_task_arg_t)*argsCnt)/sizeof(uint64_t);
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
    while (task->cmdHeader->valid == QUEUE_VALID && tries++ < MAX_WAIT_TASKS_TRIES) {
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
        task->cmdHeader->valid = QUEUE_INVALID;

        return XTASKS_SUCCESS;

    }

    ticketLockRelease(&_bufferTicket);
    return XTASKS_PENDING;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event * events, size_t const maxCount)
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
        xdma_status stat = xdmaMemcpy(events, _instrBuffHandle, count*sizeof(xtasks_ins_event), (accBuffer - _instrBuff)*sizeof(xtasks_ins_event), XDMA_FROM_DEVICE);
        if (stat != XDMA_SUCCESS) {
            __sync_lock_release(&acc->instrLock);
            return XTASKS_ERROR;
        }
        i = 0;
        while (i < count && events[i].eventType != XTASKS_EVENT_TYPE_INVALID)
        {
            //Invalidate all read entries in the accelerator buffer
            accBuffer[i].eventType = XTASKS_EVENT_TYPE_INVALID;
            i++;
        }
        if (i > 0) {
            stat = xdmaMemcpy(accBuffer, _instrBuffHandle, i*sizeof(xtasks_ins_event), (accBuffer - _instrBuff)*sizeof(xtasks_ins_event), XDMA_TO_DEVICE);
            if (stat != XDMA_SUCCESS) {
                __sync_lock_release(&acc->instrLock);
                return XTASKS_ERROR;
            }
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
    if (_spawnOutQueue == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the spawn out queue
    size_t idx, taskSize;
    //new_task_header_t * hwTaskHeader;
    ticketLockAcquire(&_bufferTicket);

    idx = _spawnOutQueueIdx;
    //hwTaskHeader = (new_task_header_t *)(&_spawnOutQueue[idx]);
    uint64_t* headerP = &_spawnOutQueue[idx];
    uint64_t header = _spawnOutQueue[idx];

    uint8_t valid = header >> 56;
    uint8_t numArgs = (header >> 8) & 0x00000000000000FF;
    uint8_t numDeps = 0;
    uint8_t numCopies = (header >> 24) & 0x00000000000000FF;
    uint32_t archMask = (header >> 32) & 0x0000000000FFFFFF;
    //new_task_header_t headerCopy = *hwTaskHeader;
    if (valid != QUEUE_VALID) {
        ticketLockRelease(&_bufferTicket);
        return XTASKS_PENDING;
    }

    taskSize = (
                sizeof(new_task_header_t) +
                sizeof(uint64_t)*numArgs +
                sizeof(new_task_dep_t)*numDeps +
                sizeof(new_task_copy_t)*numCopies)/
            sizeof(uint64_t);
    _spawnOutQueueIdx = (idx+taskSize)%SPWN_OUT_QUEUE_LEN;

    //Extract the information from the new buffer
    *task = realloc(*task,
                    sizeof(xtasks_newtask) +
                    sizeof(xtasks_newtask_arg)*numArgs +
                    sizeof(xtasks_newtask_dep)*numDeps +
                    sizeof(xtasks_newtask_copy)*numCopies);
    (*task)->args = (xtasks_newtask_arg *)(*task + 1);
    (*task)->numArgs = numArgs;
    (*task)->deps = (xtasks_newtask_dep *)((*task)->args + (*task)->numArgs);
    (*task)->numDeps = numDeps;
    (*task)->copies = (xtasks_newtask_copy *)((*task)->deps + (*task)->numDeps);
    (*task)->numCopies = numCopies;
    (*task)->architecture = archMask;

    idx = (idx+1)%SPWN_OUT_QUEUE_LEN; //NOTE: new_task_header_t->taskID field is the 2nd word
    (*task)->taskId = _spawnOutQueue[idx];
    _spawnOutQueue[idx] = 0; //< Cleanup the memory position

    idx = (idx+1)%SPWN_OUT_QUEUE_LEN; //NOTE: new_task_header_t->parentID field is the 3th word
    (*task)->parentId = _spawnOutQueue[idx]; //< NOTE: We don't know what is that ID (SW or HW)
    _spawnOutQueue[idx] = 0; //< Cleanup the memory position

    idx = (idx+1)%SPWN_OUT_QUEUE_LEN; //NOTE: new_task_header_t->typeInfo field is the 4th word
    (*task)->typeInfo = _spawnOutQueue[idx];
    _spawnOutQueue[idx] = 0; //< Cleanup the memory position

    for (size_t i = 0; i < (*task)->numCopies; ++i) {
        //NOTE: Each copy uses 3 uint64_t elements in the newQueue
        //      After using each memory position, we have to clean it
        uint64_t tmp;

        //NOTE: new_task_copy_t->address field is the 1st word
        idx = (idx+1)%SPWN_OUT_QUEUE_LEN;
        (*task)->copies[i].address = (void *)((uintptr_t)_spawnOutQueue[idx]);
        _spawnOutQueue[idx] = 0;

        //NOTE: new_task_copy_t->flags and new_task_copy_t->size fields are the 2nd word
        idx = (idx+1)%SPWN_OUT_QUEUE_LEN;
        tmp = _spawnOutQueue[idx];
        uint8_t copyFlags = tmp >> NEW_TASK_COPY_FLAGS_WORDOFFSET;
        (*task)->copies[i].flags = copyFlags;
        uint32_t copySize = tmp >> NEW_TASK_COPY_SIZE_WORDOFFSET;
        (*task)->copies[i].size = copySize;
        _spawnOutQueue[idx] = 0;

        //NOTE: new_task_copy_t->offset and new_task_copy_t->accessedLen fields are the 2nd word
        idx = (idx+1)%SPWN_OUT_QUEUE_LEN;
        tmp = _spawnOutQueue[idx];
        uint32_t copyOffset = tmp >> NEW_TASK_COPY_OFFSET_WORDOFFSET;
        (*task)->copies[i].offset = copyOffset;
        uint32_t copyAccessedLen = tmp >> NEW_TASK_COPY_ACCESSEDLEN_WORDOFFSET;
        (*task)->copies[i].accessedLen = copyAccessedLen;
        _spawnOutQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numDeps; ++i) {
        idx = (idx+1)%SPWN_OUT_QUEUE_LEN;
        new_task_dep_t * hwTaskDep = (new_task_dep_t *)(&_spawnOutQueue[idx]);

        //Parse the dependence information
        (*task)->deps[i].address = hwTaskDep->address;
        (*task)->deps[i].flags = hwTaskDep->flags;

        //Cleanup the memory position
        _spawnOutQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numArgs; ++i) {
        //Check that arg pointer is not out of bounds
        idx = (idx+1)%SPWN_OUT_QUEUE_LEN;
        (*task)->args[i] = _spawnOutQueue[idx];

        //Cleanup the memory position
        _spawnOutQueue[idx] = 0;
    }

    //Free the buffer slot
    //NOTE: This word cannot be set to 0 as the task size information must be keept
    __sync_synchronize();
    //hwTaskHeader->valid = QUEUE_INVALID;
    *headerP = header & 0x00FFFFFFFFFFFFFF;
    ticketLockRelease(&_bufferTicket);

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id)
{
    rem_fini_task_t * entryHeader;

    ticketLockAcquire(&_bufferTicket);

    // Get an empty slot into the TM remote finished queue
    size_t idx;
    idx = _spawnInQueueIdx;
    entryHeader = (rem_fini_task_t *)(&_spawnInQueue[idx]);
    if (entryHeader->valid == QUEUE_VALID) {
        ticketLockRelease(&_bufferTicket);
        return XTASKS_ENOENTRY;
    }

    //NOTE: rem_fini_task_t->taskId is the 1st word
    idx = (idx+1)%SPWN_IN_QUEUE_LEN;
    _spawnInQueue[idx] = id;

    //NOTE: rem_fini_task_t->parentId is the 2nd word
    idx = (idx+1)%SPWN_IN_QUEUE_LEN;
    _spawnInQueue[idx] = parent;

    __sync_synchronize();
    entryHeader->valid = QUEUE_VALID;

    _spawnInQueueIdx = (idx+1)%SPWN_IN_QUEUE_LEN;

    ticketLockRelease(&_bufferTicket);

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp * timestamp)
{
    if (timestamp == NULL) return XTASKS_EINVAL;

    *timestamp = *_instrCounter;
    return XTASKS_SUCCESS;
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

uint64_t cmdOutQueue(size_t const accID, size_t const idx)
{
    return _cmdOutQueue[accID*CMD_OUT_SUBQUEUE_LEN + idx];
}

uint64_t spawnOutQueue(size_t const idx)
{
    return _spawnOutQueue[idx];
}

uint64_t spawnInQueue(size_t const idx)
{
    return _spawnInQueue[idx];
}

xtasks_ins_event accelGetEvent(size_t const aIdx, size_t const evIdx) {
    return _instrBuff[aIdx*_numInstrEvents + evIdx];
}

void accelPrintInstrBuffer(size_t const aIdx) {
    xtasks_ins_event *event = _instrBuff + aIdx*_numInstrEvents;
    xdma_status stat = xdmaMemcpy(event, _instrBuffHandle, (_numAccs*_numInstrEvents - aIdx*_numInstrEvents)*sizeof(xtasks_ins_event), (aIdx*_numInstrEvents)*sizeof(xtasks_ins_event), XDMA_FROM_DEVICE);
    if (stat != XDMA_SUCCESS)
        return;
    fprintf(stderr, "timestamp, accid, eventid, value\n");
    while ( event->eventId != XTASKS_EVENT_TYPE_INVALID ) {
        fprintf(stderr, "%llu,\t%u,\t%u,\t%llu\n", (unsigned long long int)event->timestamp,
                event->eventType, event->eventId, (unsigned long long int)event->value);
        event++;
    }
}

#endif
