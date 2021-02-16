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

#ifndef __LIBXTASKS_H__
#define __LIBXTASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum {
    XTASKS_SUCCESS = 0,  ///< Operation finished successfully
    XTASKS_ENOSYS,       ///< Function not implemented
    XTASKS_EINVAL,       ///< Invalid operation arguments
    XTASKS_ENOMEM,       ///< Not enough memory to execute the operation
    XTASKS_EFILE,        ///< Operation finished after fail a file operation
    XTASKS_ENOENTRY,     ///< Operation failed because no entry could be reserved
    XTASKS_PENDING,      ///< Operation not finished yet
    XTASKS_ENOAV,        ///< Function/operation not available
    XTASKS_ERROR         ///< Operation finished with some error
} xtasks_stat;

typedef enum { XTASKS_COMPUTE_DISABLE = 0, XTASKS_COMPUTE_ENABLE = 1 } xtasks_comp_flags;

typedef enum {
    XTASKS_HOST_TO_ACC,  ///< From host memory to accelerator memory
    XTASKS_ACC_TO_HOST   ///< From accelerator memory to host memory
} xtasks_memcpy_kind;

#define XTASKS_ARG_FLAG_BRAM 0x00
#define XTASKS_ARG_FLAG_PRIVATE 0x01
#define XTASKS_ARG_FLAG_GLOBAL 0x02
#define XTASKS_ARG_FLAG_COPY_IN 0x10
#define XTASKS_ARG_FLAG_COPY_OUT 0x20

typedef void *xtasks_task_handle;
typedef uint64_t xtasks_task_id;
typedef uint64_t xtasks_arg_val;
typedef uint32_t xtasks_arg_id;
typedef uint8_t xtasks_arg_flags;
typedef void *xtasks_acc_handle;
typedef uint32_t xtasks_acc_id;
typedef uint64_t xtasks_acc_type;
typedef const char *xtasks_acc_desc;
typedef uint64_t xtasks_ins_timestamp;
typedef void *xtasks_mem_handle;
typedef long unsigned int xtasks_memcpy_handle;
typedef uint64_t xtasks_newtask_arg;

typedef struct {
    xtasks_acc_id id;             ///< Accelerator identifier
    float freq;                   ///< Accelerator frequency (in MHz)
    xtasks_acc_type type;         ///< Accelerator type identifier
    xtasks_acc_desc description;  ///< Accelerator description
    short maxTasks;               ///< Max. number of concurrent tasks that the accelerator can manage (-1: undefined)
} xtasks_acc_info;

typedef struct {
    uint64_t value;      ///< Event value
    uint64_t timestamp;  ///< Event timestamp
    uint32_t eventId;    ///< Event id
    uint32_t eventType;  ///< Event type (one of xtasks_event_type)
} xtasks_ins_event;

typedef enum {
    XTASKS_EVENT_TYPE_BURST_OPEN = 0,
    XTASKS_EVENT_TYPE_BURST_CLOSE = 1,
    XTASKS_EVENT_TYPE_POINT = 2,
    XTASKS_EVENT_TYPE_INVALID = 0xFFFFFFFF
} xtasks_event_type;

typedef struct {
    uint64_t address;  ///< Dependence address
    uint8_t flags;     ///< Dependence flags
} xtasks_newtask_dep;

typedef struct {
    uint8_t flags;       ///< Copy flags
    void *address;       ///< Copy address
    size_t size;         ///< Size of the region to allocate (in bytes)
    size_t offset;       ///< Offset at the region beginning not accessed (in bytes)
    size_t accessedLen;  ///< Length of the accessed data in the region (in bytes)
} xtasks_newtask_copy;

typedef struct {
    xtasks_task_id taskId;        ///< Task identifier inside runtime HW
    xtasks_task_id parentId;      ///< Parent task identifier that is creating the task (may be SW or HW identifier)
    uint64_t typeInfo;            ///< Identifier of the task type
    size_t numArgs;               ///< Number of arguments
    xtasks_newtask_arg *args;     ///< Arguments array
    size_t numDeps;               ///< Number of dependences
    xtasks_newtask_dep *deps;     ///< Dependences array
    size_t numCopies;             ///< Number of copies
    xtasks_newtask_copy *copies;  ///< Copies array
} xtasks_newtask;

/*!
 * \brief Initialize the library
 */
xtasks_stat xtasksInit();

/*!
 * \brief Cleanup the library
 */
xtasks_stat xtasksFini();

/*!
 * \brief Get the platform name that libxtasks targets
 * \param[out] name       Pointer that will be set with the platform string address
 */
xtasks_stat xtasksGetPlatform(const char **name);

/*!
 * \brief Get the communication backend name that libxtasks uses
 * \param[out] name       Pointer that will be set with the backend string address
 */
xtasks_stat xtasksGetBackend(const char **name);

/*!
 * \brief Get the number of available accelerators in the system
 */
xtasks_stat xtasksGetNumAccs(size_t *count);

/*!
 * \brief Get the accelerator handles for the accelerators in the system
 * \param[in]  maxCount   Number of accelerator handles that will be retrieved
 * \param[out] array      Array (with >= maxCount entries) that will hold the accelerator handles
 * \param[out] count      Number of handles copied to the array
 */
xtasks_stat xtasksGetAccs(size_t const maxCount, xtasks_acc_handle *array, size_t *count);

/*!
 * \brief Get information of an accelerator
 * \param[in]  handle     Accelerator handle which information will be returned
 * \param[out] info       Pointer to a valid xtasks_acc_info struct where the accelerator info will be placed
 */
xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info *info);

/*!
 * \brief Create a new task
 * \param[in]  id         Task identifier that will be associated to the task
 * \param[in]  accel      Accelerator handle of the accelerator where the task will be sent
 * \param[in]  parent     Parent task identifier (it can be NULL if not knwon or not needed)
 * \param[in]  compute    Compute flags. Defines whether the accelerator should execute the task core or not
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle will be set
 */
xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId, xtasks_task_id const parent,
    xtasks_comp_flags const compute, xtasks_task_handle *handle);

/*!
 * \brief Create a new periodic task
 * \param[in]  id         Task identifier that will be associated to the task
 * \param[in]  accel      Accelerator handle of the accelerator where the task will be sent
 * \param[in]  parent     Parent task identifier (it can be NULL if not knwon or not needed)
 * \param[in]  compute    Compute flags. Defines whether the accelerator should execute the task core or not
 * \param[in]  numReps    Number of task core repetitions
 * \param[in]  period     Minimum number of accelerator cycles between task core executions
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle will be set
 */
xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps, unsigned int const period,
    xtasks_task_handle *handle);

/*!
 * \brief Delete a task
 * \param[in]  handle     Task handle of the task to delete
 */
xtasks_stat xtasksDeleteTask(xtasks_task_handle *handle);

/*!
 * \brief Add a task argument to a task with the given information
 * \param[in]  id         Argument identifier (they may be sent out of order)
 * \param[in]  flags      Argument flags. Defines some properties of the argument related to placing and caching
 * \param[in]  value      Argument value
 * \param[in]  handle     Task handle where the argument will be added
 */
xtasks_stat xtasksAddArg(
    xtasks_arg_id const id, xtasks_arg_flags const flags, xtasks_arg_val const value, xtasks_task_handle const handle);

/*!
 * \brief Append an array of arguments to a task all with the same flags
 *        The argument id will be set based on the number of existing arguments and the array ordering
 * \param[in]  num        Number of argument pointer by values
 * \param[in]  flags      Arguments flags (common for all arguments)
 * \param[in]  values     Array of argument values
 * \param[in]  handle     Task handle where the arguments will be added
 */
xtasks_stat xtasksAddArgs(
    size_t const num, xtasks_arg_flags const flags, xtasks_arg_val *const values, xtasks_task_handle const handle);

/*!
 * \brief Submit the task to the FPGA
 * \param[in]  handle     Task handle to be sent
 */
xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle);

/*!
 * \brief Synchronously wait until the task execution finishes
 * \param[in]  handle     Task handle to synchronize
 */
xtasks_stat xtasksWaitTask(xtasks_task_handle const handle);

/*!
 * \brief Try to get a task which execution finished
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle may be set
 * \param[out] id         Pointer to a valid xtasks_task_id where the task identifier may be set
 * \returns    XTASKS_PENDING if no task has been synchronized,
 *             XTASKS_SUCCESS if a task has been synchronized and output parameter have been set
 */
xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle *handle, xtasks_task_id *id);

/*!
 * \brief Try to get a task which execution finished for an accelerator
 * \param[in]  accel      Accelerator handle that will be used to retrieve a task from
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle may be set
 * \param[out] id         Pointer to a valid xtasks_task_id where the task identifier may be set
 * \returns    XTASKS_PENDING if no task has been synchronized,
 *             XTASKS_SUCCESS if a task has been synchronized and output parameter have been set
 */
xtasks_stat xtasksTryGetFinishedTaskAccel(
    xtasks_acc_handle const accel, xtasks_task_handle *handle, xtasks_task_id *id);

/*!
 * \brief Get instrumentation events buffer for an accelerator
 *        Events will be set in the events array until an XTASKS_EVENT_TYPE_INVALID is reached or maxCount
 *        events are wrote into the buffer.
 * \param[in]  accel      Accelerator handle of the accelerator which data will be retrieved
 * \param[out] events     Pointer to an xtasks_ins_event array that can fit at least maxCount elements
 * \param[in]  maxCount   Number of events the event array can hold
 * \returns    XTASKS_ENOAV if instrumentation is not available,
 *             XTASKS_SUCCESS if a some events have been wrote into events array
 */
xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event *events, size_t const maxCount);

/*!
 * \brief Initialize hardware instrumentation
 * \param[in] nEvents   Number of elements in the circular buffers of each accelerator
 */
xtasks_stat xtasksInitHWIns(size_t const nEvents);

/*!
 * \brief Finalize hardware instrumentation
 */
xtasks_stat xtasksFiniHWIns();
/*!
 * \brief Try to get a new task generated by an accelerator
 * \param[out] task       Pointer to a valid xtasks_newtask pointer that may be set if a new task is found.
 *                        NOTE: Caller will be responsible of freeing the allocated space for task information
 * \returns    XTASKS_PENDING if no task has been found,
 *             XTASKS_SUCCESS if a task has been found and the pointer has been set
 */
xtasks_stat xtasksTryGetNewTask(xtasks_newtask **task);

/*!
 * \brief Notify the finalization of a task generated by an accelerator
 * \param[in]  parent     Parent task identifier obtained at xtasksTryGetNewTask
 * \param[in]  id         Task identifier obtained at xtasksTryGetNewTask
 * \returns    XTASKS_SUCCESS if the action has been succesfully completed
 */
xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id);

/*!
 * \brief Get the current time for an accelerator
 * \param[in]  accel      Accelerator handle of the accelerator which time will be retrieved
 * \param[out] timestamp  Timestamp of current accelerator time
 */
xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp *timestamp);

/*!
 * \brief Allocate memory in the accelerators address space
 * \param[in]  len        Amount of bytes that will be allocated
 * \param[out] handle     Pointer to a valid xtasks_mem_handle where the allocation handle will be set
 */
xtasks_stat xtasksMalloc(size_t len, xtasks_mem_handle *handle);

/*!
 * \brief Unallocate memory
 * \param[in]  handle     Allocation handle to be unallocated
 */
xtasks_stat xtasksFree(xtasks_mem_handle handle);

/*!
 * \brief Get the accelerator address of an allocation
 * \param[in]  handle     Allocation handle
 * \param[out] addr       Pointer to a valid variable that will be set with the accelerator address
 */
xtasks_stat xtasksGetAccAddress(xtasks_mem_handle const handle, uint64_t *addr);

/*!
 * \brief Synchronously copy data to/from an allocation
 * \param[in]  handle     Allocation handle
 * \param[in]  offset     Offset (in bytes) before starting the copy in the allocation
 * \param[in]  len        Amount of bytes that will be copied
 * \param[in]  usr        Pointer to the user space memory that will be copied from/to
 * \param[in]  kind       Kind of copy that will be done
 */
xtasks_stat xtasksMemcpy(
    xtasks_mem_handle const handle, size_t offset, size_t len, void *usr, xtasks_memcpy_kind const kind);

/*!
 * \brief Asynchronously copy data to/from an allocation
 * \param[in]  handle     Allocation handle
 * \param[in]  offset     Offset (in bytes) before starting the copy in the allocation
 * \param[in]  len        Amount of bytes that will be copied
 * \param[in]  usr        Pointer to the user space memory that will be copied from/to
 * \param[in]  kind       Kind of copy that will be done
 * \param[out] cpyHandle  Pointer to a valid xtasks_memcpy_handle where the copy operation handle
 *                        will be set
 */
xtasks_stat xtasksMemcpyAsync(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
    xtasks_memcpy_kind const kind, xtasks_memcpy_handle *cpyHandle);

/*!
 * \brief Test the status of a copy operation
 * \param[in]  handle     Copy operation handle that will be checked
 * \returns    XTASKS_PENDING if the copy operation has not finished yet,
 *             XTASKS_SUCCESS if the copy operation has sucessfully finalized
 */
xtasks_stat xtasksTestCopy(xtasks_memcpy_handle *handle);

/*!
 * \brief Synchronously wait for a copy operation
 * \param[in]  handle     Copy operation handle that will be synchronized
 */
xtasks_stat xtasksSyncCopy(xtasks_memcpy_handle *handle);

#ifdef __cplusplus
}
#endif

#endif /* __LIBXTASKS_H__ */
