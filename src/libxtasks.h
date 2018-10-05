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

#ifndef __LIBXTASKS_H__
#define __LIBXTASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

typedef enum {
    XTASKS_SUCCESS = 0,       ///< Operation finised sucessfully
    XTASKS_ENOSYS,            ///< Function not implemented
    XTASKS_EINVAL,            ///< Invalid operation arguments
    XTASKS_ENOMEM,            ///< Not enough memory to execute the operation
    XTASKS_EFILE,             ///< Operation finished after fail a file operation
    XTASKS_ENOENTRY,          ///< Operation failed because no entry could be reserved
    XTASKS_PENDING,           ///< Operation not finished yet
    XTASKS_ERROR              ///< Operation finished with sone error
} xtasks_stat;

typedef enum {
    XTASKS_COMPUTE_DISABLE = 0,
    XTASKS_COMPUTE_ENABLE  = 1
} xtasks_comp_flags;

#define XTASKS_ARG_FLAG_BRAM     0x00
#define XTASKS_ARG_FLAG_PRIVATE  0x01
#define XTASKS_ARG_FLAG_GLOBAL   0x02
#define XTASKS_ARG_FLAG_COPY_IN  0x10
#define XTASKS_ARG_FLAG_COPY_OUT 0x20

typedef void *       xtasks_task_handle;
typedef uint64_t     xtasks_task_id;
typedef uint64_t     xtasks_arg_val;
typedef uint32_t     xtasks_arg_id;
typedef uint32_t     xtasks_arg_flags;
typedef void *       xtasks_acc_handle;
typedef uint32_t     xtasks_acc_id;
typedef uint32_t     xtasks_acc_type;
typedef const char * xtasks_acc_desc;
typedef uint64_t     xtasks_ins_timestamp;

typedef struct {
    xtasks_acc_id   id;               ///< Accelerator identifier
    xtasks_acc_type type;             ///< Accelerator type identifier
    xtasks_acc_desc description;      ///< Accelerator description
    float           freq;             ///< Accelerator frequency (in MHz)
    short           maxTasks;         ///< Max. number of concurrent tasks that the accelerator can manage (-1: undefined)
} xtasks_acc_info;

typedef struct {
    xtasks_ins_timestamp start;        ///< Timestamp start
    xtasks_ins_timestamp inTransfer;   ///< Timestamp after in transfers have finished
    xtasks_ins_timestamp computation;  ///< Timestamp after computation have finished
    xtasks_ins_timestamp outTransfer;  ///< Timestamp after output transfers have finished/acc end
} xtasks_ins_times;

/*!
 * \brief Initialize the library
 */
xtasks_stat xtasksInit();

/*!
 * \brief Cleanup the library
 */
xtasks_stat xtasksFini();

/*!
 * \brief Get the number of available accelerators in the system
 */
xtasks_stat xtasksGetNumAccs(size_t * count);

/*!
 * \brief Get the accelerator handles for the accelerators in the system
 * \param[in]  maxCount   Number of accelerator handles that will be retrieved
 * \param[out] array      Array (with >= maxCount entries) that will hold the accelerator handles
 * \param[out] count      Number of handles copied to the array
 */
xtasks_stat xtasksGetAccs(size_t const maxCount, xtasks_acc_handle * array, size_t * count);

/*!
 * \brief Get information of an accelerator
 * \param[in]  handle     Accelerator handle which information will be returned
 * \param[out] info       Pointer to a valid xtasks_acc_info struct where the accelerator info will be placed
 */
xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info * info);

/*!
 * \brief Create a new task
 * \param[in]  id         Task identifier that will be associated to the task
 * \param[in]  accel      Accelerator handle of the accelerator where the task will be sent
 * \param[in]  compute    Compute flags. Defines whether the accelerator should execute the task core or not
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle will be set
 */
xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accel,
    xtasks_comp_flags const compute, xtasks_task_handle * handle);

/*!
 * \brief Delete a task
 * \param[in]  handle     Task handle of the task to delete
 */
xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle);

/*!
 * \brief Add a task argument to a task with the given information
 * \param[in]  id         Argument identifier (they may be sent out of order)
 * \param[in]  flags      Argument flags. Defines some properties of the argument related to placing and caching
 * \param[in]  value      Argument value
 * \param[in]  handle     Task handle where the argument will be added
 */
xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle);

/*!
 * \brief Append an array of arguments to a task all with the same flags
 *        The argument id will be set based on the number of existing arguments and the array ordering
 * \param[in]  num        Number of argument pointer by values
 * \param[in]  flags      Arguments flags (common for all arguments)
 * \param[in]  values     Array of argument values
 * \param[in]  handle     Task handle where the arguments will be added
 */
xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle);

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
xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle * handle, xtasks_task_id * id);

/*!
 * \brief Try to get a task which execution finished for an accelerator
 * \param[in]  accel      Accelerator handle that will be used to retrieve a task from
 * \param[out] handle     Pointer to a valid xtasks_task_handle where the task handle may be set
 * \param[out] id         Pointer to a valid xtasks_task_id where the task identifier may be set
 * \returns    XTASKS_PENDING if no task has been synchronized,
 *             XTASKS_SUCCESS if a task has been synchronized and output parameter have been set
 */
xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel,
    xtasks_task_handle * handle, xtasks_task_id * id);

/*!
 * \brief Get instrumentation timestamps for a task
 * \param[in]  handle     Task handle which instrumentation data will be retrieved
 * \param[out] times      Pointer to a valid xtasks_ins_times poainter that will be set with the addess
 *                        of instrumentation data
 */
xtasks_stat xtasksGetInstrumentData(xtasks_task_handle const handle, xtasks_ins_times ** times);

#ifdef __cplusplus
}
#endif

#endif /* __LIBXTASKS_H__ */
