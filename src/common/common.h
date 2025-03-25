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

#ifndef __LIBXTASKS_COMMON_H__
#define __LIBXTASKS_COMMON_H__

#include "../libxtasks.h"
#include "libxdma.h"
#include "ticket-lock.h"

#include <libxdma.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>  //include index for old compilers
//__USE_BSD is one of the defines needed to enable the usleep function, it seems
// to not be enabled with old compilers
#ifdef __USE_BSD
#define BSD_USED
#endif
#define __USE_BSD
#include <unistd.h>
#ifndef BSD_USED
#undef __USE_BSD
#endif
#undef BSD_USED
#include "bitinfo.h"

#define PRINT_ERROR(_str_) fprintf(stderr, "[xTasks ERROR]: %s\n", _str_)
#define PRINT_ERROR_ARGS(_str_, ...) fprintf(stderr, "[xTasks ERROR]: " _str_ "\n", __VA_ARGS__)
#define MIN_WRAPPER_VER 13
#define MAX_WRAPPER_VER 13

#define QUEUE_VALID 0x80
#define QUEUE_RESERVED 0x40
#define QUEUE_INVALID 0x00
#define DEF_EXEC_TASK_SIZE 256  ///< Size of hw task when using the default num. of args.
#define DEF_EXEC_TASK_ARGS_LEN \
    14  // NOTE: (DEF_EXEC_TASK_SIZE - sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)
// NOTE: A task in extended mode MUST fit into the a sub-queue of cmd_in queue
#define EXT_HW_TASK_SIZE 512  ///< Size of hw task when using the extended num. of args.
#define EXT_HW_TASK_ARGS_LEN \
    30  // NOTE: (EXT_HW_TASK_SIZE -
        // sizeof(cmd_exec_task_header_t))/sizeof(cmd_exec_task_arg_t)

#define CMD_EXEC_TASK_CODE 0x01              ///< Command code for execute task commands
#define CMD_SETUP_INS_CODE 0x02              ///< Command code for setup instrumentation info
#define CMD_FINI_EXEC_CODE 0x03              ///< Command code for finished execute task commands
#define CMD_PERI_TASK_CODE 0x05              ///< Command code for execute periodic task commands
#define CMD_SETUP_INS_ARGS_NUMEVS_BYTES 4    ///< Number of bytes used by Num. Events argument in the commandArgs array
#define CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET 0  ///< Offset of Num. Args. field in the commandArgs array
#define CMD_EXEC_TASK_ARGS_COMP_OFFSET 3     ///< Offset of Compute flag in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_OFFSET \
    4  ///< Offset of Destination id (where accel will send finish signal) in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_HWR 0x0  ///< Hardware runtime identifier for the destId field

// NOTE: The value NUM_RUN_TASKS may be changed to increase the number of concurrent tasks submitted into the
// accelerators
#define NUM_RUN_TASKS 1024  ///< Maximum number of concurrently running tasks
#define NEW_TASK_COPY_FLAGS_WORDOFFSET \
    0  ///< Offset of new_task_copy_t->flags field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_ARGIDX_WORDOFFSET \
    8  ///< Offset of new_task_copy_t->argIdx field in the 2nd word forming new_task_copy_t
#define NEW_TASK_COPY_SIZE_WORDOFFSET \
    32  ///< Offset of new_task_copy_t->size field in the 2nd word forming new_task_copy_t
#define STR_BUFFER_SIZE 129

#define max(a, b) \
    ({ \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b; \
    })

#define min(a, b) \
    ({ \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a < _b ? _a : _b; \
    })

//! \brief Internal library HW accelerator information
typedef struct {
    int devId;
    xtasks_acc_info info;
    unsigned short volatile cmdInWrIdx;    ///< Writing index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile cmdInRdIdx;    ///< Reading index of the accelerator sub-queue in the cmd_in queue
    unsigned short volatile cmdInAvSlots;  ///< Counter for available slots in cmd_in sub-queue
    ticketLock_t cmdInLock;                ///< Lock for atomic operations over cmd_in sub-queue
    unsigned short volatile cmdOutIdx;     ///< Reading index of the accelerator sub-queue in the cmd_out queue
    unsigned short volatile cmdOutLock;    ///< Lock for atomic operations over cmd_out sub-queue
    unsigned short volatile instrIdx;      ///< Reading index of the accelerator instrumentation buffer
    unsigned short volatile instrLock;     ///< Lock for atomic operations over instrumentation buffers
} acc_t;

//! \brief Command header type
typedef struct __attribute__((__packed__, aligned(8))) {
    uint8_t commandCode;     //[0  :7  ] Command code
    uint8_t commandArgs[6];  //[8  :55 ] Command arguments
    uint8_t valid;           //[56 :63 ] Valid entry
} cmd_header_t;

//! \brief Header of execute task command
typedef struct __attribute__((__packed__, aligned(8))) {
    cmd_header_t header;  //[0  :63 ] Command header
    uint64_t taskID;      //[64 :127] Task identifier
    uint64_t parentID;    //[128:191] Parent task identifier (may be null)
} cmd_exec_task_header_t;

//! \brief Argument entry of execute task command
typedef struct __attribute__((__packed__, aligned(8))) {
    uint8_t flags;        //[0  :7  ] Flags
    uint8_t _padding[3];  //[8  :32 ]
    uint32_t id;          //[32 :63 ] Argument ID
    uint64_t value;       //[64 :127] Address
} cmd_exec_task_arg_t;

//! \brief Internal library task information
typedef struct {
    xtasks_task_id id;                 ///< External task identifier
    cmd_header_t *cmdHeader;           ///< Pointer to the cmd_header_t struct
    cmd_exec_task_arg_t *cmdExecArgs;  ///< Pointer to the array of cmd_exec_task_arg_t structs
    xtasks_acc_handle accel;           ///< Accelerator where the task will run
    uint8_t extSize : 1;               ///< Whether the space available in args is extended or not
    uint8_t periTask : 1;              ///< Whether the tasks is a periodic task or not
} task_t;

typedef enum { BIT_NO_COMPAT, BIT_COMPAT, BIT_COMPAT_SKIP } bit_compatibility_t;

//! \brief Header of execute periodic task command
typedef struct __attribute__((__packed__, aligned(8))) {
    cmd_header_t header;  //[0  :63 ] Command header
    uint64_t taskID;      //[64 :127] Task identifier
    uint64_t parentID;    //[128:191] Parent task identifier (may be null)
    uint32_t numReps;     //[192:223] Number of task body repetitions
    uint32_t period;      //[224:255] Minimum number of cycles between repetitions
} cmd_peri_task_header_t;

//! \brief Setup hw instrumentation command
typedef struct __attribute__((__packed__, aligned(8))) {
    cmd_header_t header;  //[0  :63 ] Command header
    uint64_t bufferAddr;  //[64 :127] Instrumentation buffer address
} cmd_setup_hw_ins_t;

//! \brief Response out command for execute task commands
typedef struct __attribute__((__packed__, aligned(8))) {
    cmd_header_t header;  //[0  :63 ] Command header
    uint64_t taskID;      //[64 :127] Executed task identifier
} cmd_out_exec_task_t;

//! \brief New task buffer representation  (Only the header part, N arguments follow the header)
typedef struct __attribute__((__packed__)) {
    uint8_t _padding;      //[0  :7  ]
    uint8_t numArgs;       //[8  :15 ] Number of arguments after this header
    uint8_t numDeps;       //[16 :23 ] Number of dependencies after the task arguments
    uint8_t numCopies;     //[24 :31 ] Number of copies after the task dependencies
    uint8_t _padding1[3];  //[32 :55 ]
    uint8_t valid;         //[56 :63 ] Valid Entry
    uint64_t taskID;       //[64 :127] Task identifier
    uint64_t parentID;     //[128:191] Parent task identifier that is creating the task
    uint64_t typeInfo;     //[192:255] Information of task type
} new_task_header_t;

//! \brief New task buffer representation (Only the dependence part, repeated N times)
typedef struct __attribute__((__packed__)) {
    uint64_t address : 56;  //[0  :55 ] Dependence value
    uint8_t flags;          //[56 :63 ] Dependence flags
} new_task_dep_t;

//! \brief New task buffer representation (Only the argument part, repeated N times)
typedef struct __attribute__((__packed__)) {
    uint64_t address;  //[0  :63 ] Copy address
    uint32_t flags;    //[64 :95 ] Copy flags
    uint32_t size;     //[96 :127] Size of the copy
} new_task_copy_t;

//! \brief Remote finished task buffer representation
typedef struct __attribute__((__packed__)) {
    uint8_t _padding[7];  //[0  :55 ]
    uint8_t valid;        //[56 :63 ] Valid Entry
    uint64_t taskID;      //[64 :127] Task identifier
    uint64_t parentID;    //[128:191] Parent task identifier that created the tasks
} rem_fini_task_t;

/*!
 * \brief Returns the length of command based on its header
 */
uint8_t getCmdLength(cmd_header_t const *header)
{
    uint8_t length = 0;
    uint8_t const cmdCode = header->commandCode;
    if (cmdCode == CMD_EXEC_TASK_CODE) {
        uint8_t const numArgs = header->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
        uint8_t const argsLength = sizeof(cmd_exec_task_arg_t) / sizeof(uint64_t) * numArgs;
        length = sizeof(cmd_exec_task_header_t) / sizeof(uint64_t) + argsLength;
    } else if (cmdCode == CMD_SETUP_INS_CODE) {
        length = sizeof(cmd_setup_hw_ins_t) / sizeof(uint64_t);
    } else if (cmdCode == CMD_FINI_EXEC_CODE) {
        length = sizeof(cmd_out_exec_task_t) / sizeof(uint64_t);
    } else if (cmdCode == CMD_PERI_TASK_CODE) {
        uint8_t const numArgs = header->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
        uint8_t const argsLength = sizeof(cmd_exec_task_arg_t) / sizeof(uint64_t) * numArgs;
        length = sizeof(cmd_peri_task_header_t) / sizeof(uint64_t) + argsLength;
    }
    return length;
}

/*!
 * \brief Returns a xtasks_stat based on a xdma_status
 */
xtasks_stat toXtasksStat(xdma_status const status)
{
    xtasks_stat ret = XTASKS_ERROR;
    if (status == XDMA_SUCCESS) {
        ret = XTASKS_SUCCESS;
    } else if (status == XDMA_ENOMEM) {
        ret = XTASKS_ENOMEM;
    } else if (status == XDMA_ENOSYS) {
        ret = XTASKS_ENOSYS;
    }
    return ret;
}

/*!
 * \brief Prints an error message in STDERR about bitstream compatibility
 */
void printErrorBitstreamVersionCompatibility(unsigned int bitinfoVersion)
{
    fprintf(stderr, "ERROR: Loaded FPGA bitstream may not be compatible with this version of libxtasks.\n");
    fprintf(stderr, "       The compatible bitinfo version is: %d\n", BITINFO_MIN_REV);
    fprintf(stderr, "       Loaded FPGA bitstream bitinfo version is: %u\n", bitinfoVersion);
    fprintf(stderr, "       Alternatively, you may disable the compatibility check setting");
    fprintf(stderr, " XTASKS_COMPATIBILITY_CHECK environment variable to 0.\n");
}

/*!
 * \brief Prints an error message in STDERR about bitstream compatibility
 */
void printErrorBitstreamWrapperCompatibility(unsigned int wrapperVersion)
{
    fprintf(stderr, "ERROR: Loaded FPGA bitstream may not be compatible with this version of libxtasks.\n");
    fprintf(stderr, "       The compatible wrapper versions are: [%d, %d]\n", MIN_WRAPPER_VER, MAX_WRAPPER_VER);
    fprintf(stderr, "       Loaded FPGA bitstream wrapper version is: %u\n", wrapperVersion);
    fprintf(stderr, "       Alternatively, you may disable the compatibility check setting");
    fprintf(stderr, " XTASKS_COMPATIBILITY_CHECK environment variable to 0.\n");
}

/*!
 * \brief Checks whether the current fpga bitstream is compatible with the libxtasks version or not.
 * \return  BIT_NO_COMPAT if the bitstream is not compatible
 *          BIT_COMPAT if the bitstream is compatible
 *          BIT_COMPAT_SKIP if the check was skipped due to user requirements
 */
bit_compatibility_t checkbitstreamCompatibility(const uint32_t *bitinfo)
{
    const char *compatCheck = getenv("XTASKS_COMPATIBILITY_CHECK");
    if (compatCheck != NULL && compatCheck[0] == '0') {
        return BIT_COMPAT_SKIP;
    } else if (compatCheck != NULL && compatCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_COMPATIBILITY_CHECK, must be 0 or 1. Ignoring it");
    }

    const unsigned int bitinfoRev = bitinfo_get_version(bitinfo);
    // The bitstream info BRAM version is old
    if (bitinfoRev < BITINFO_MIN_REV) {
        printErrorBitstreamVersionCompatibility(bitinfoRev);
        return BIT_NO_COMPAT;
    }

    const unsigned int version = bitinfo_get_wrapper_version(bitinfo);
    if (version < MIN_WRAPPER_VER || version > MAX_WRAPPER_VER) {
        printErrorBitstreamWrapperCompatibility(version);
        return BIT_NO_COMPAT;
    }

    return BIT_COMPAT;
}

void initAccList(int devId, acc_t *accs, const bit_acc_type_t *acc_types, int n_acc_types, uint32_t cmdInSubqueueLen)
{
    int accid = 0;
    for (int i = 0; i < n_acc_types; ++i) {
        for (int j = 0; j < acc_types[i].count; ++j) {
            accs[accid].devId = devId;
            accs[accid].info.id = accid;
            accs[accid].info.type = acc_types[i].type;
            accs[accid].info.freq = acc_types[i].freq;
            accs[accid].info.description = acc_types[i].description;
            accs[accid].cmdInWrIdx = 0;
            accs[accid].cmdInAvSlots = cmdInSubqueueLen;
            accs[accid].cmdInRdIdx = 0;
            ticketLockInit(&accs[accid].cmdInLock);
            accs[accid].cmdOutIdx = 0;
            accs[accid].cmdOutLock = 0;
            ++accid;
        }
    }
}

inline __attribute__((always_inline)) void resetHWRuntime(volatile uint32_t *resetReg)
{
    // Nudge one register
    *resetReg = 0x00;
    usleep(1);  // Wait for the reset to propagate
    *resetReg = 0x01;
}

int getFreeTaskEntry(task_t *tasks)
{
    for (int i = 0; i < NUM_RUN_TASKS; ++i) {
        if (tasks[i].id == 0) {
            if (__sync_bool_compare_and_swap(&tasks[i].id, 0, 1)) {
                return i;
            }
        }
    }
    return -1;
}

void initializeTask(task_t *task, const xtasks_task_id id, xtasks_acc_handle const accel, xtasks_task_id const parent,
    xtasks_comp_flags const compute)
{
    task->id = id;
    task->accel = accel;
    task->periTask = 0;
    cmd_exec_task_header_t *cmdHeader = (cmd_exec_task_header_t *)task->cmdHeader;
    task->cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_EXEC_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_HWR;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(task);
}

void initializePeriodicTask(task_t *task, const xtasks_task_id id, xtasks_acc_handle const accel,
    xtasks_task_id const parent, xtasks_comp_flags const compute, int numReps, int period)
{
    task->id = id;
    task->accel = accel;
    task->periTask = 1;
    cmd_peri_task_header_t *cmdHeader = (cmd_peri_task_header_t *)task->cmdHeader;
    task->cmdExecArgs = (cmd_exec_task_arg_t *)(cmdHeader + 1);
    cmdHeader->header.commandCode = CMD_PERI_TASK_CODE;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] = 0;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_COMP_OFFSET] = compute;
    cmdHeader->header.commandArgs[CMD_EXEC_TASK_ARGS_DESTID_OFFSET] = CMD_EXEC_TASK_ARGS_DESTID_HWR;
    cmdHeader->parentID = (uintptr_t)(parent);
    cmdHeader->taskID = (uintptr_t)(task);
    cmdHeader->numReps = numReps;
    cmdHeader->period = period;
}

xtasks_stat setExtendedModeTask(task_t *task)
{
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
    return XTASKS_SUCCESS;
}

xtasks_stat submitCommand(
    acc_t *acc, uint64_t *command, const size_t length, volatile uint64_t *queue, uint32_t cmdInSubqueueLen)
{
    size_t idx;
    uint64_t cmdHeader;
    size_t const offset = acc->info.id * cmdInSubqueueLen;
    cmd_header_t *const cmdHeaderPtr = (cmd_header_t *const)&cmdHeader;

    // While there is not enough space in the queue, look for already read commands
    while (acc->cmdInAvSlots < length) {
        ticketLockAcquire(&acc->cmdInLock);
        if (acc->cmdInAvSlots < length) {
        SUB_CMD_CHECK_RD:
            idx = acc->cmdInRdIdx;
            cmdHeader = queue[offset + idx];
            if (cmdHeaderPtr->valid == QUEUE_INVALID) {
#ifdef XTASKS_DEBUG
                uint8_t const cmdNumArgs = cmdHeaderPtr->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
                if (cmdHeaderPtr->commandCode == CMD_EXEC_TASK_CODE && cmdNumArgs > EXT_HW_TASK_ARGS_LEN) {
                    PRINT_ERROR("Found unexpected data when executing xtasksSubmitCommand");
                    ticketLockRelease(&acc->cmdInLock);
                    return XTASKS_ERROR;
                }
#endif /* XTASKS_DEBUG */
                size_t const cmdNumWords = getCmdLength(cmdHeaderPtr);
                acc->cmdInRdIdx = (idx + cmdNumWords) % cmdInSubqueueLen;
                acc->cmdInAvSlots += cmdNumWords;
            }
        } else {
            // NOTE: At this point the thread has the lock acquired, so directly bypass it into the queue idx update.
            //      The lock will be released in the code after the atomic operations
            goto SUB_CMD_UPDATE_IDX;
        }
        ticketLockRelease(&acc->cmdInLock);
    }

    ticketLockAcquire(&acc->cmdInLock);
    if (acc->cmdInAvSlots >= length) {
    SUB_CMD_UPDATE_IDX:
        idx = acc->cmdInWrIdx;
        acc->cmdInWrIdx = (idx + length) % cmdInSubqueueLen;
        acc->cmdInAvSlots -= length;
        cmdHeader = queue[offset + idx];
        cmdHeaderPtr->valid = QUEUE_RESERVED;
        queue[offset + idx] = cmdHeader;

        // Release the lock as it is not needed anymore
        __sync_synchronize();
        ticketLockRelease(&acc->cmdInLock);

        // Do no write the header (1st word pointer by command ptr) until all payload is written
        for (size_t i = 1, idxMod = idx + 1 == cmdInSubqueueLen ? 0 : idx + 1; i < length;
             ++i, idxMod = idxMod + 1 == cmdInSubqueueLen ? 0 : idxMod + 1) {
            queue[offset + idxMod] = command[i];
        }
        cmdHeader = *command;
        cmdHeaderPtr->valid = QUEUE_VALID;

        __sync_synchronize();
        // Write the header now
        queue[offset + idx] = cmdHeader;
    } else {
        // NOTE: At this point the thread has the lock acquired, so directly bypass it into the check.
        //      The lock will be released in the code after the check
        goto SUB_CMD_CHECK_RD;
    }

    return XTASKS_SUCCESS;
}

int getAccEvents(
    acc_t *acc, xtasks_ins_event *events, size_t count, size_t numInstrEvents, xdma_buf_handle instrBuffHandle)
{
    size_t devInstroff;
    devInstroff = (acc->info.id * numInstrEvents + acc->instrIdx) * sizeof(xtasks_ins_event);

    xdma_status stat;
    int i;

    stat = xdmaMemcpy(events, instrBuffHandle, count * sizeof(xtasks_ins_event), devInstroff, XDMA_FROM_DEVICE);
    if (stat != XDMA_SUCCESS) {
        return -1;
    }
    // Count valid events
    for (i = 0; i < count && events[i].eventType != XTASKS_EVENT_TYPE_INVALID; i++)
        ;

    // Push event invalidation to the device
    if (i > 0) {
        uint32_t invEvent = XTASKS_EVENT_TYPE_INVALID;
        for (int e = 0; e < i; ++e) {
            stat = xdmaMemcpy(&invEvent, instrBuffHandle, sizeof(uint32_t),
                devInstroff + e * sizeof(xtasks_ins_event) + 1 * sizeof(uint32_t), XDMA_TO_DEVICE);
            if (stat != XDMA_SUCCESS) {
                return -1;
            }
        }
    }
    acc->instrIdx = (acc->instrIdx + i) % numInstrEvents;
    return i;
}

void getNewTaskFromQ(xtasks_newtask **task, volatile uint64_t *spawnQueue, int idx, uint32_t spawnOutQueueLen)
{
    volatile new_task_header_t *hwTaskHeader = (new_task_header_t *)&spawnQueue[idx];
    // Extract the information from the new buffer
    *task = realloc(*task, sizeof(xtasks_newtask) + sizeof(xtasks_newtask_arg) * hwTaskHeader->numArgs +
                               sizeof(xtasks_newtask_dep) * hwTaskHeader->numDeps +
                               sizeof(xtasks_newtask_copy) * hwTaskHeader->numCopies);
    (*task)->args = (xtasks_newtask_arg *)(*task + 1);
    (*task)->numArgs = hwTaskHeader->numArgs;
    (*task)->deps = (xtasks_newtask_dep *)((*task)->args + (*task)->numArgs);
    (*task)->numDeps = hwTaskHeader->numDeps;
    (*task)->copies = (xtasks_newtask_copy *)((*task)->deps + (*task)->numDeps);
    (*task)->numCopies = hwTaskHeader->numCopies;

    idx = (idx + 1) % spawnOutQueueLen;  // NOTE: new_task_header_t->taskID field is the 2nd word
    (*task)->taskId = spawnQueue[idx];
    spawnQueue[idx] = 0;  //< Cleanup the memory position

    idx = (idx + 1) % spawnOutQueueLen;   // NOTE: new_task_header_t->parentID field is the 3th word
    (*task)->parentId = spawnQueue[idx];  //< NOTE: We don't know what is that ID (SW or HW)
    spawnQueue[idx] = 0;                  //< Cleanup the memory position

    idx = (idx + 1) % spawnOutQueueLen;  // NOTE: new_task_header_t->typeInfo field is the 4th word
    (*task)->typeInfo = spawnQueue[idx];
    spawnQueue[idx] = 0;  //< Cleanup the memory position

    for (size_t i = 0; i < (*task)->numDeps; ++i) {
        idx = (idx + 1) % spawnOutQueueLen;
        new_task_dep_t *hwTaskDep = (new_task_dep_t *)(&spawnQueue[idx]);

        // Parse the dependence information
        (*task)->deps[i].address = hwTaskDep->address;
        (*task)->deps[i].flags = hwTaskDep->flags;

        // Cleanup the memory position
        spawnQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numCopies; ++i) {
        // NOTE: Each copy uses 2 uint64_t elements in the newQueue
        //      After using each memory position, we have to clean it
        uint64_t tmp;

        // NOTE: new_task_copy_t->address field is the 1st word
        idx = (idx + 1) % spawnOutQueueLen;
        (*task)->copies[i].address = (void *)((uintptr_t)spawnQueue[idx]);
        spawnQueue[idx] = 0;

        // NOTE: new_task_copy_t->flags and new_task_copy_t->size fields are the 2nd word
        idx = (idx + 1) % spawnOutQueueLen;
        tmp = spawnQueue[idx];
        uint8_t argIdx = tmp >> NEW_TASK_COPY_ARGIDX_WORDOFFSET;
        (*task)->copies[i].argIdx = argIdx;
        uint8_t copyFlags = tmp >> NEW_TASK_COPY_FLAGS_WORDOFFSET;
        (*task)->copies[i].flags = copyFlags;
        uint32_t copySize = tmp >> NEW_TASK_COPY_SIZE_WORDOFFSET;
        (*task)->copies[i].size = copySize;
        spawnQueue[idx] = 0;
    }

    for (size_t i = 0; i < (*task)->numArgs; ++i) {
        // Check that arg pointer is not out of bounds
        idx = (idx + 1) % spawnOutQueueLen;
        (*task)->args[i] = spawnQueue[idx];

        // Cleanup the memory position
        spawnQueue[idx] = 0;
    }

    // Free the buffer slot
    // NOTE: This word cannot be set to 0 as the task size information must be kept
    __sync_synchronize();
    hwTaskHeader->valid = QUEUE_INVALID;
}

#endif /* __LIBXTASKS_COMMON_H__ */
