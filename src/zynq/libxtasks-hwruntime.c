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

#include "../common/common.h"
#include "../common/ticket-lock.h"
#include "../libxtasks.h"

#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libxdma.h>
#include <libxdma_version.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <assert.h>

#define DEF_ACCS_LEN 8  ///< Def. allocated slots in the accs array

#define CMD_IN_QUEUE_PATH "/dev/ompss_fpga/hwruntime/cmd_in_queue"
#define CMD_OUT_QUEUE_PATH "/dev/ompss_fpga/hwruntime/cmd_out_queue"
#define SPAWN_OUT_QUEUE_PATH "/dev/ompss_fpga/hwruntime/spawn_out_queue"
#define SPAWN_IN_QUEUE_PATH "/dev/ompss_fpga/hwruntime/spawn_in_queue"
#define HWRUNTIME_RST_PATH "/dev/ompss_fpga/hwruntime/rstn"
#define BITINFO_PATH "/dev/ompss_fpga/bitinfo"
#define BITINFO_GET_VERSION 13

//! Check that libxdma version is compatible
#define LIBXTASKS_MIN_MAJOR 3
#define LIBXTASKS_MIN_MINOR 1
#if !defined(LIBXDMA_VERSION_MAJOR) || !defined(LIBXDMA_VERSION_MINOR) || \
    LIBXDMA_VERSION_MAJOR < LIBXTASKS_MIN_MAJOR || \
    (LIBXDMA_VERSION_MAJOR == LIBXTASKS_MIN_MAJOR && LIBXDMA_VERSION_MINOR < LIBXTASKS_MIN_MINOR)
#error Installed libxdma is not supported (use >= 3.1)
#endif

#define MAX_ACC_CONFIG_SIZE 4096

//! \brief Platform and Backend strings
const char _platformName[] = "zynq";
const char _backendName[] = "hwruntime";

static size_t _ndevs = 0;                  ///< Number of FPGA devices in the system
static int _init_cnt = 0;                  ///< Counter of calls to init/fini
static size_t _numAccs;                    ///< Number of accelerators in the system
static acc_t *_accs;                       ///< Accelerators data
static bit_acc_type_t *_acc_types;         ///< Accelerator types data
static uint8_t *_cmdExecTaskBuff;          ///< Buffer to send the HW tasks
static task_t *_tasks;                     ///< Array with internal task information
static volatile uint64_t *_cmdInQueue;     ///< Command IN queue
static volatile uint64_t *_cmdOutQueue;    ///< Command OUT queue
static volatile uint64_t *_spawnOutQueue;  ///< Spawn OUT queue (buffer of FPGA spawned tasks)
static size_t _spawnOutQueueIdx;           ///< Reading index of the _spawnOutQueue
static volatile uint64_t *_spawnInQueue;   ///< Spawn IN queue (buffer of finished FPGA spawned tasks)
static size_t _spawnInQueueIdx;            ///< Writing index of the _spawnInQueue
static volatile uint32_t *_hwruntimeRst;   ///< Register to reset the HW runtime
static uint32_t _cmdInSubqueueLen;         ///< Size (words) of each subqueue of command IN queue
static uint32_t _cmdOutSubqueueLen;        ///< Size (words) of each subqueue of command OUT queue
static uint32_t _spawnInQueueLen;          ///< Size (words) of spawn IN queue
static uint32_t _spawnOutQueueLen;         ///< Size (words) of spawn OUT queue

static bool _instrAvail;
static size_t _numInstrEvents;            ///< Number of instrumentation events for each accelerator buffer
static xtasks_ins_event *_instrBuff;      ///< Buffer of instrumentation events
static xtasks_ins_event *_instrBuffPhy;   ///< Physical address of _instrBuff
static xdma_buf_handle _instrBuffHandle;  ///< Handle of _instrBuff in libxdma

/*!
 * \brief Set n times the byte c in dst
 * \note Assuming: n is multiple of 32, dst and src are aligned to 32 bits
 * \note Avoid optimizations for the function implementation as aarch64 cannot execute
 *       some fast copy instructions over non-cacheable memory
 */
void _memset(volatile void *dst, int c, size_t n);
inline void __attribute__((optimize("O1"))) _memset(volatile void *dst, int c, size_t n)
{
#if __aarch64__
    uint32_t *d = (uint32_t *)dst;
    char cc = c;
    uint32_t v;
    v = cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    for (size_t i = 0; i < n / sizeof(uint32_t); ++i) {
        d[i] = v;
    }
#else
    memset((void *)dst, c, n);
#endif
}

xtasks_stat xtasksInitHWIns(size_t const nEvents)
{
    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;
    size_t insBufferSize;
    unsigned long phyAddr;

    // At least we need 1 event + last event mark
    if (nEvents <= 1) return XTASKS_EINVAL;

    if (!_instrAvail) {
        return XTASKS_ENOAV;
    }

    s = xdmaInitHWInstrumentation();
    if (s != XDMA_SUCCESS) {
        // Return as there's nothing to undo
        return XTASKS_ENOAV;
        // goto intrInitErr;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents * _numAccs * sizeof(xtasks_ins_event);
    s = xdmaAllocateHost(0, (void **)&_instrBuff, &_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ENOMEM;
        goto instrAllocErr;
    }

    // get phy address
    s = xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        goto instrGetAddrErr;
    }
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    // Invalidate all entries
    for (size_t i = 0; i < _numInstrEvents * _numAccs; ++i) {
        _instrBuff[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    // Send the instrumentation buffer to each accelerator
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    memcpy((void *)cmd.header.commandArgs, (void *)&_numInstrEvents, CMD_SETUP_INS_ARGS_NUMEVS_BYTES);
    for (size_t i = 0; i < _numAccs; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents * i);

        ret = submitCommand(
            _accs + i, (uint64_t *)&cmd, sizeof(cmd_setup_hw_ins_t) / sizeof(uint64_t), _cmdInQueue, _cmdInSubqueueLen);

        if (ret != XTASKS_SUCCESS) {
            goto instrSendInit;
        }

        _accs[i].instrIdx = 0;
        _accs[i].instrLock = 0;
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
        // Instrumentation is not initialized or has been already finished
        return XTASKS_SUCCESS;
    }
    s1 = xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuff = NULL;
    _instrBuffPhy = NULL;
    s0 = xdmaFiniHWInstrumentation();

    return (s0 == XDMA_SUCCESS && s1 == XDMA_SUCCESS) ? XTASKS_SUCCESS : XTASKS_ERROR;
}

xtasks_stat xtasksInit()
{
    // Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    // Always initialize instrumentation as disabled
    _instrBuff = NULL;
    _numInstrEvents = 0;

    xtasks_stat ret = XTASKS_SUCCESS;
    xdma_status s;

    uint32_t *bitinfo = malloc(BITINFO_MAX_SIZE);
    if (bitinfo == NULL) {
        PRINT_ERROR("Unable to allocate memory for the bitinfo");
        ret = XTASKS_ERROR;
        goto err_bitinfo_malloc;
    }
    int bitinfo_fd = open(BITINFO_PATH, O_RDONLY);
    if (bitinfo_fd < 0) {
        PRINT_ERROR("Unable to open bitinfo file " BITINFO_PATH);
        ret = XTASKS_EFILE;
        goto err_bitinfo_open;
    }
    uint64_t driver_supported_version = 0;
    if (ioctl(bitinfo_fd, BITINFO_GET_VERSION, &driver_supported_version) != 0) {
        PRINT_ERROR("Unable to determine the driver supported version of the bitinfo");
        ret = XTASKS_EFILE;
        close(bitinfo_fd);
        goto err_bitinfo_ioctl;
    }
    if (driver_supported_version > BITINFO_MIN_REV) {
        PRINT_ERROR_ARGS("Found bitinfo version %" PRIu64
                         " which is older than the driver minimum supported version %u\n",
            driver_supported_version, BITINFO_MIN_REV);
        ret = XTASKS_ERROR;
        close(bitinfo_fd);
        goto err_bitinfo_driver_version;
    }
    int nread = read(bitinfo_fd, bitinfo, BITINFO_MAX_SIZE);
    close(bitinfo_fd);
    if (nread != BITINFO_MAX_SIZE) {
        PRINT_ERROR("Unable to read bitinfo");
        ret = XTASKS_EFILE;
        goto err_bitinfo_read;
    }

    // Check if bitstream is compatible
    bit_compatibility_t compat = checkbitstreamCompatibility(bitinfo);
    if (compat == BIT_NO_COMPAT) {
        ret = XTASKS_ERROR;
        goto err_bit_compat;
    }

    // Initialize xdma memory subsystem
    s = xdmaInit();
    if (s != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
        if (s == XDMA_ENOENT) {
            PRINT_ERROR("xdmaInitMem failed. Check if xdma device exist in the system");
        } else if (s == XDMA_EACCES) {
            PRINT_ERROR("xdmaInitMem failed. Current user cannot access xdma device");
        } else {
            PRINT_ERROR("xdmaInitMem failed");
        }
        goto err_xdma_init;
    }

    _cmdInSubqueueLen = bitinfo_get_cmd_in_len(bitinfo);
    _cmdOutSubqueueLen = bitinfo_get_cmd_out_len(bitinfo);
    _numAccs = bitinfo_get_acc_count(bitinfo);
    uint32_t numAccTypes = bitinfo_get_acc_type_count(bitinfo);
    _accs = malloc(_numAccs * sizeof(acc_t));
    if (_accs == NULL) {
        PRINT_ERROR("Could not allocate accelerators array");
        goto err_accs_malloc;
    }
    _acc_types = malloc(numAccTypes * sizeof(bit_acc_type_t));
    if (_acc_types == NULL) {
        PRINT_ERROR("Could not allocate accelerator types array");
        goto err_acctypes_malloc;
    }
    bitinfo_init_acc_types(bitinfo, _acc_types);
    initAccList(0, _accs, _acc_types, numAccTypes, _cmdInSubqueueLen);

    int cmdInQFd = open(CMD_IN_QUEUE_PATH, O_RDWR, (mode_t)0600);
    if (cmdInQFd < 0) {
        PRINT_ERROR("Cannot open basic hwruntime device files");
        goto err_cmdin_open;
    }
    _cmdInQueue = (uint64_t *)mmap(
        NULL, _cmdInSubqueueLen * _numAccs * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, cmdInQFd, 0);
    close(cmdInQFd);
    if (_cmdInQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_in_queue");
        goto err_cmdin_mmap;
    }

    int cmdOutQFd = open(CMD_OUT_QUEUE_PATH, O_RDWR, (mode_t)0600);
    if (cmdOutQFd < 0) {
        PRINT_ERROR("Cannot open basic hwruntime device files");
        goto err_cmdout_open;
    }
    _cmdOutQueue = (uint64_t *)mmap(
        NULL, _cmdOutSubqueueLen * _numAccs * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, cmdOutQFd, 0);
    close(cmdOutQFd);
    if (_cmdOutQueue == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/cmd_out_queue");
        goto err_cmdout_mmap;
    }

    _instrAvail = bitinfo_get_feature(bitinfo, BIT_FEATURE_INST);
    bool spawn_feature = bitinfo_get_feature(bitinfo, BIT_FEATURE_SPAWN_Q);
    if (spawn_feature) {
        _spawnInQueueLen = bitinfo_get_spawn_in_len(bitinfo);
        _spawnOutQueueLen = bitinfo_get_spawn_out_len(bitinfo);
        int spawnOutQFd = open(SPAWN_OUT_QUEUE_PATH, O_RDWR, (mode_t)0600);
        if (spawnOutQFd < 0) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open hwruntime/spawn_out_queue device file");
            goto err_spawnout_open;
        }
        _spawnOutQueue = (uint64_t *)mmap(
            NULL, _spawnOutQueueLen * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, spawnOutQFd, 0);
        close(spawnOutQFd);
        if (_spawnOutQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_out_queue");
            goto err_spawnout_mmap;
        }

        int spawnInQFd = open(SPAWN_IN_QUEUE_PATH, O_RDWR, (mode_t)0600);
        if (spawnInQFd < 0) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot open hwruntime/spawn_in_queue device file");
            goto err_spawnin_open;
        }
        _spawnInQueue = (uint64_t *)mmap(
            NULL, _spawnInQueueLen * sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, spawnInQFd, 0);
        if (_spawnInQueue == MAP_FAILED) {
            ret = XTASKS_EFILE;
            PRINT_ERROR("Cannot map hwruntime/spawn_in_queue");
            goto err_spawnin_mmap;
        }
    } else {
        _spawnInQueue = NULL;
        _spawnOutQueue = NULL;
    }

    int rstnFd = open(HWRUNTIME_RST_PATH, O_RDWR, (mode_t)0600);
    if (rstnFd < 0) {
        PRINT_ERROR("Cannot open hwruntime reset file");
        goto err_rst_open;
    }
    _hwruntimeRst = (uint32_t *)mmap(NULL, sizeof(uint32_t), PROT_READ | PROT_WRITE, MAP_SHARED, rstnFd, 0);
    if (_hwruntimeRst == MAP_FAILED) {
        ret = XTASKS_EFILE;
        PRINT_ERROR("Cannot map hwruntime/ctrl");
        goto err_rst_mmap;
    }

    // If any, invalidate commands in cmd_in_queue
    _memset(_cmdInQueue, 0, _cmdInSubqueueLen * _numAccs * sizeof(uint64_t));
    // If any, invalidate commands in cmd_out_queue
    _memset(_cmdOutQueue, 0, _cmdOutSubqueueLen * _numAccs * sizeof(uint64_t));
    if (spawn_feature) {
        _memset(_spawnInQueue, 0, _spawnInQueueLen * sizeof(uint64_t));
        _memset(_spawnOutQueue, 0, _spawnOutQueueLen * sizeof(uint64_t));
    }

    resetHWRuntime(_hwruntimeRst);

    // Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS * sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto err_tasks_malloc;
    }
    _cmdExecTaskBuff = (uint8_t *)malloc(NUM_RUN_TASKS * DEF_EXEC_TASK_SIZE);
    if (_cmdExecTaskBuff == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for exec. tasks buffer");
        goto err_exectask_malloc;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].cmdHeader = (cmd_header_t *)&_cmdExecTaskBuff[idx * DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)0;
        _tasks[idx].extSize = 0;
        _tasks[idx].periTask = 0;
    }

    _ndevs = 1;

    free(bitinfo);

    return ret;

err_exectask_malloc:
    free(_tasks);
err_tasks_malloc:
    munmap((void *)_hwruntimeRst, sizeof(uint32_t));
err_rst_mmap:
err_rst_open:
    if (spawn_feature) munmap((void *)_spawnInQueue, _spawnInQueueLen * sizeof(uint64_t));
err_spawnin_mmap:
err_spawnin_open:
    if (spawn_feature) munmap((void *)_spawnOutQueue, _spawnOutQueueLen * sizeof(uint64_t));
err_spawnout_mmap:
err_spawnout_open:
    munmap((void *)_cmdOutQueue, _cmdOutSubqueueLen * _numAccs * sizeof(uint64_t));
err_cmdout_mmap:
err_cmdout_open:
    munmap((void *)_cmdInQueue, _cmdInSubqueueLen * _numAccs * sizeof(uint64_t));
err_cmdin_mmap:
err_cmdin_open:
    free(_acc_types);
err_acctypes_malloc:
    free(_accs);
err_accs_malloc:
    xdmaFini();
err_xdma_init:
err_bit_compat:
err_bitinfo_read:
err_bitinfo_driver_version:
err_bitinfo_ioctl:
err_bitinfo_open:
    free(bitinfo);
err_bitinfo_malloc:
    __sync_sub_and_fetch(&_init_cnt, 1);
    return ret;
}

xtasks_stat xtasksFini()
{
    // Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    xtasks_stat ret = XTASKS_SUCCESS;

    // Free tasks array
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        if (_tasks[idx].extSize) {
            // Free tasks in extended mode
            free((void *)_tasks[idx].cmdHeader);
        }
    }
    free(_cmdExecTaskBuff);
    free(_tasks);
    free(_acc_types);
    free(_accs);
    _tasks = NULL;

    // Finialize the HW instrumentation if needed
    if (xtasksFiniHWIns() != XTASKS_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    // Unmap the HW runtime queues
    int statusRd, statusFi, statusNw, statusRFi, statusCtrl;
    statusCtrl = munmap((void *)_hwruntimeRst, sizeof(uint32_t));
    statusRFi = _spawnInQueue != NULL ? munmap((void *)_spawnInQueue, sizeof(uint64_t) * _spawnInQueueLen) : 0;
    statusNw = _spawnOutQueue != NULL ? munmap((void *)_spawnOutQueue, sizeof(uint64_t) * _spawnOutQueueLen) : 0;
    statusFi = munmap((void *)_cmdOutQueue, sizeof(uint64_t) * _numAccs * _cmdOutSubqueueLen);
    statusRd = munmap((void *)_cmdInQueue, sizeof(uint64_t) * _numAccs * _cmdInSubqueueLen);
    if (statusRd == -1 || statusFi == -1 || statusNw == -1 || statusRFi == -1 || statusCtrl == -1) {
        ret = XTASKS_EFILE;
    }

    _numAccs = 0;

    // Close xdma memory management
    if (xdmaFini() != XDMA_SUCCESS) {
        ret = XTASKS_ERROR;
    }

    return ret;
}

xtasks_stat xtasksGetPlatform(const char **name)
{
    if (name == NULL) return XTASKS_EINVAL;
    *name = _platformName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetBackend(const char **name)
{
    if (name == NULL) return XTASKS_EINVAL;
    *name = _backendName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumDevices(int *numDevices)
{
    if (numDevices == NULL) return XTASKS_EINVAL;
    *numDevices = _ndevs;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(int devId, size_t *count)
{
    if (devId >= _ndevs || count == NULL) return XTASKS_EINVAL;
    *count = _numAccs;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccs(int devId, size_t const maxCount, xtasks_acc_handle *array, size_t *count)
{
    if (array == NULL || count == NULL) return XTASKS_EINVAL;

    size_t tmp = maxCount > _numAccs ? _numAccs : maxCount;
    for (size_t i = 0; i < tmp; ++i) {
        array[i] = (xtasks_acc_handle)(&_accs[i]);
    }
    *count = tmp;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info *info)
{
    if (info == NULL) return XTASKS_EINVAL;

    acc_t *ptr = (acc_t *)handle;
    *info = ptr->info;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId, xtasks_task_id const parent,
    xtasks_comp_flags const compute, xtasks_task_handle *handle)
{
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    initializeTask(&_tasks[idx], id, accId, parent, compute);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_task_id const parent, xtasks_comp_flags const compute, unsigned int const numReps, unsigned int const period,
    xtasks_task_handle *handle)
{
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    initializePeriodicTask(&_tasks[idx], id, accId, parent, compute, numReps, period);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle *handle)
{
    task_t *task = (task_t *)(*handle);
    *handle = NULL;
    task->id = 0;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(
    xtasks_arg_id const id, xtasks_arg_flags const flags, xtasks_arg_val const value, xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt >= EXT_HW_TASK_ARGS_LEN) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (argsCnt == DEF_EXEC_TASK_ARGS_LEN) {
        // Entering in extended mode
        setExtendedModeTask(task);
    }

    argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET]++;
    task->cmdExecArgs[argsCnt].flags = flags;
    task->cmdExecArgs[argsCnt].id = id;
    task->cmdExecArgs[argsCnt].value = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(
    size_t const num, xtasks_arg_flags const flags, xtasks_arg_val *const values, xtasks_task_handle const handle)
{
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (num > EXT_HW_TASK_ARGS_LEN - argsCnt) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    } else if (num > (DEF_EXEC_TASK_ARGS_LEN - argsCnt) && argsCnt <= DEF_EXEC_TASK_ARGS_LEN) {
        // Entering in extended mode because:
        // 1) The number of args to add does not fit in current allocated space
        // 2) We are not in extended mode
        // 3) The number of args will fit in extended mode
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
    task_t *task = (task_t *)(handle);
    acc_t *acc = (acc_t *)task->accel;

    if (task == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }
    // Update the task/command information
    task->cmdHeader->valid = QUEUE_VALID;

    // NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numHeaderBytes = task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t);
    size_t const numCmdWords = (numHeaderBytes + sizeof(cmd_exec_task_arg_t) * argsCnt) / sizeof(uint64_t);
    return submitCommand(acc, (uint64_t *)task->cmdHeader, numCmdWords, _cmdInQueue, _cmdInSubqueueLen);
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle *handle, xtasks_task_id *id)
{
    if (handle == NULL || id == NULL) {
        return XTASKS_EINVAL;
    }

    xtasks_stat ret = XTASKS_PENDING;
    for (int d = 0; d < _ndevs; ++d) {
        ret = xtasksTryGetFinishedTaskDev(d, handle, id);
        if (ret != XTASKS_PENDING) return ret;
    }

    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskDev(int devId, xtasks_task_handle *handle, xtasks_task_id *id)
{
    if (handle == NULL || id == NULL || devId >= _ndevs) {
        return XTASKS_EINVAL;
    }

    xtasks_stat ret = XTASKS_PENDING;
    size_t i = 0;
    do {
        ret = xtasksTryGetFinishedTaskAccel(&_accs[i], handle, id);
    } while (++i < _numAccs && ret == XTASKS_PENDING);

    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel, xtasks_task_handle *handle, xtasks_task_id *id)
{
    acc_t *acc = (acc_t *)accel;
    volatile uint64_t *const subqueue = _cmdOutQueue + acc->info.id * _cmdOutSubqueueLen;

    if (handle == NULL || id == NULL || acc == NULL) {
        return XTASKS_EINVAL;
    }

    size_t idx = acc->cmdOutIdx;
    uint64_t cmdBuffer = subqueue[idx];
    cmd_header_t *cmd = (cmd_header_t *)&cmdBuffer;

    if (cmd->valid == QUEUE_VALID) {
        if (__sync_lock_test_and_set(&acc->cmdOutLock, 1) == 0) {
            // Read the command header
            idx = acc->cmdOutIdx;
            cmdBuffer = subqueue[idx];

            if (cmd->valid != QUEUE_VALID) {
                __sync_lock_release(&acc->cmdOutLock);
                return XTASKS_PENDING;
            }

#ifdef XTASKS_DEBUG
            if (cmd->commandCode != CMD_FINI_EXEC_CODE) {
                PRINT_ERROR("Found unexpected data when executing xtasksTryGetFinishedTaskAccel");
                __sync_lock_release(&acc->cmdOutLock);
                return XTASKS_ERROR;
            }
#endif /* XTASKS_DEBUG */

            // Read the command payload (task identifier)
            size_t const dataIdx = (idx + 1) % _cmdOutSubqueueLen;
            uint64_t const taskID = subqueue[dataIdx];
            subqueue[dataIdx] = 0;  //< Clean the buffer slot

            // Invalidate the buffer entry
            cmd = (cmd_header_t *)&subqueue[idx];
            cmd->valid = QUEUE_INVALID;

            // Update the read index and release the lock
            acc->cmdOutIdx = (idx + sizeof(cmd_out_exec_task_t) / sizeof(uint64_t)) % _cmdOutSubqueueLen;
            __sync_lock_release(&acc->cmdOutLock);

#ifdef XTASKS_DEBUG
            if (taskID < (uintptr_t)_tasks || taskID >= (uintptr_t)(_tasks + NUM_RUN_TASKS)) {
                PRINT_ERROR("Found an invalid task identifier when executing xtasksTryGetFinishedTaskAccel");
                return XTASKS_ERROR;
            }
#endif /* XTASKS_DEBUG */

            uintptr_t taskPtr = (uintptr_t)taskID;
            task_t *task = (task_t *)taskPtr;
            *handle = (xtasks_task_handle)task;
            *id = task->id;

            // Mark the task as executed (using the valid field as it is not used in the cached copy)<
            task->cmdHeader->valid = QUEUE_INVALID;

            return XTASKS_SUCCESS;
        }
    }
    return XTASKS_PENDING;
}

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event *events, size_t const maxCount)
{
    acc_t *acc = (acc_t *)(accel);
    size_t count, validEvents;

    if (events == NULL || (acc - _accs) >= _numAccs || maxCount <= 0)
        return XTASKS_EINVAL;
    else if (_instrBuff == NULL)
        return XTASKS_ENOAV;

    count = min(maxCount, _numInstrEvents - acc->instrIdx);
    validEvents = getAccEvents(acc, events, count, _numInstrEvents, _instrBuffHandle);
    if (validEvents < 0) {
        return XTASKS_ERROR;
    }
    if (validEvents < maxCount) {
        // Ensure invalid type of first non-wrote event slot in the caller buffer
        events[validEvents].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask **task)
{
    if (_spawnOutQueue == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the spawn out queue
    size_t idx, next, taskSize;
    new_task_header_t *hwTaskHeader;
    do {
        idx = _spawnOutQueueIdx;
        hwTaskHeader = (new_task_header_t *)(&_spawnOutQueue[idx]);
        if (hwTaskHeader->valid != QUEUE_VALID) {
            return XTASKS_PENDING;
        }
        taskSize =
            (sizeof(new_task_header_t) + sizeof(uint64_t) * hwTaskHeader->numArgs +
                sizeof(new_task_dep_t) * hwTaskHeader->numDeps + sizeof(new_task_copy_t) * hwTaskHeader->numCopies) /
            sizeof(uint64_t);
        next = (idx + taskSize) % _spawnOutQueueLen;
    } while (!__sync_bool_compare_and_swap(&_spawnOutQueueIdx, idx, next));

    getNewTaskFromQ(task, _spawnOutQueue, idx, _spawnOutQueueLen);

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id)
{
    // Get an empty slot into the TM remote finished queue
    size_t idx, next;
    rem_fini_task_t *entryHeader;
    do {
        idx = _spawnInQueueIdx;
        entryHeader = (rem_fini_task_t *)(&_spawnInQueue[idx]);
        if (entryHeader->valid == QUEUE_VALID) {
            return XTASKS_ENOENTRY;
        }
        next = (idx + sizeof(rem_fini_task_t) / sizeof(uint64_t)) % _spawnInQueueLen;
    } while (!__sync_bool_compare_and_swap(&_spawnInQueueIdx, idx, next));

    // NOTE: rem_fini_task_t->taskId is the 1st word
    idx = (idx + 1) % _spawnInQueueLen;
    _spawnInQueue[idx] = id;

    // NOTE: rem_fini_task_t->parentId is the 2nd word
    idx = (idx + 1) % _spawnInQueueLen;
    _spawnInQueue[idx] = parent;

    __sync_synchronize();
    entryHeader->valid = QUEUE_VALID;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp *timestamp)
{
    if (timestamp == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaGetDeviceTime(0, timestamp);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetMemorySize(int devId, uint32_t *size)
{
    *size = 0;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksMalloc(int devId, size_t len, xtasks_mem_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaAllocate(0, handle, len);
    return toXtasksStat(status);
}

xtasks_stat xtasksFree(xtasks_mem_handle handle)
{
    xdma_status status = xdmaFree(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetAccAddress(xtasks_mem_handle const handle, uint64_t *addr)
{
    if (addr == NULL) return XTASKS_EINVAL;

    unsigned long devAddr = 0;
    xdma_status status = xdmaGetDeviceAddress(handle, &devAddr);
    *addr = devAddr;
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpy(
    xtasks_mem_handle const handle, size_t offset, size_t len, void *usr, xtasks_memcpy_kind const kind)
{
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpy(usr, handle, len, offset, mode);
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpyAsync(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
    xtasks_memcpy_kind const kind, xtasks_memcpy_handle *cpyHandle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpyAsync(usr, handle, len, offset, mode, cpyHandle);
    return toXtasksStat(status);
}

xtasks_stat xtasksTestCopy(xtasks_memcpy_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaTestTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksSyncCopy(xtasks_memcpy_handle *handle)
{
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaWaitTransfer(handle);
    return toXtasksStat(status);
}

#ifdef XTASKS_DEBUG
uint64_t cmdInQueue(size_t const accID, size_t const idx) { return _cmdInQueue[accID * _cmdInSubqueueLen + idx]; }

uint64_t cmdOutQueue(size_t const accID, size_t const idx) { return _cmdOutQueue[accID * _cmdOutSubqueueLen + idx]; }

uint64_t spawnOutQueue(size_t const idx) { return _spawnOutQueue[idx]; }

uint64_t spawnInQueue(size_t const idx) { return _spawnInQueue[idx]; }

xtasks_ins_event accelGetEvent(size_t const aIdx, size_t const evIdx)
{
    return _instrBuff[aIdx * _numInstrEvents + evIdx];
}

void accelPrintInstrBuffer(size_t const aIdx)
{
    xtasks_ins_event *event = _instrBuff + aIdx * _numInstrEvents;
    fprintf(stderr, "timestamp, accid, eventid, value\n");
    while (event->eventId != XTASKS_EVENT_TYPE_INVALID) {
        fprintf(stderr, "%llu,\t%u,\t%u,\t%llu\n", (unsigned long long int)event->timestamp, event->eventType,
            event->eventId, (unsigned long long int)event->value);
        event++;
    }
}

xtasks_stat xtasksStartMonitor(int devId) { return XTASKS_ENOSYS; }

xtasks_stat xtasksStopMonitor(int devId) { return XTASKS_ENOSYS; }

xtasks_stat xtasksResetMonitor(int devId) { return XTASKS_ENOSYS; }

xtasks_stat xtasksGetMonitorData(int devId, xtasks_monitor_info *info) { return XTASKS_ENOSYS; }

#endif
