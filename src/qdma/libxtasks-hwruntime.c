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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "common/common.h"
#include "common/ticket-lock.h"
#include "libxdma.h"
#include "libxdma_version.h"
#include "libxtasks.h"
#include "pci_dev.h"

#define ACC_INFO_MAX_LEN 4096

#define BITINFO_ADDRESS 0x0

#define MAX_DEVICES 8
#define MAX_NODES 16
#define MAX_CLUSTER (MAX_NODES * MAX_DEVICES)

static int _ndevs = 0;
static int _cluster_size = 0;
static int _nnodes = 0;
static volatile uint32_t *_pciBar[MAX_DEVICES];
static acc_t *_accs[MAX_CLUSTER];
static bit_acc_type_t *_acc_types[MAX_CLUSTER];
static task_t *_tasks;
static unsigned int _numAccs[MAX_CLUSTER];
static uint32_t _memorySizes[MAX_DEVICES];

static int _sockfd[MAX_NODES];
static int _nodeid;
static int _client_node;
static int _fpgaid2localid[MAX_CLUSTER];
static int _fpgaid2nodeid[MAX_CLUSTER];
static int _nodeid2port[MAX_NODES];
static struct in_addr _nodeid2ip[MAX_NODES];

static volatile uint64_t *_cmdInQueue[MAX_DEVICES];
static volatile uint64_t *_cmdOutQueue[MAX_DEVICES];
static volatile uint64_t *_spawnOutQueue[MAX_DEVICES];
static size_t _spawnOutQueueIdx[MAX_DEVICES];
static volatile uint64_t *_spawnInQueue[MAX_DEVICES];
static size_t _spawnInQueueIdx[MAX_DEVICES];
static volatile uint32_t *_hwruntimeRst[MAX_DEVICES];
static uint8_t *_cmdExecTaskBuff;
static uint32_t _cmdInSubqueueLen[MAX_DEVICES];
static uint32_t _cmdOutSubqueueLen[MAX_DEVICES];
static uint32_t _spawnInQueueLen[MAX_DEVICES];
static uint32_t _spawnOutQueueLen[MAX_DEVICES];
static uint64_t _hwcounter_address[MAX_DEVICES];
static uint32_t *_sysmonAddr[MAX_DEVICES];
static uint32_t *_cmsAddr[MAX_DEVICES];

static bool _instrAvail;
static size_t _numInstrEvents;
static xtasks_ins_event *_instrBuffPhy;
static xdma_buf_handle _instrBuffHandle;
static volatile uint64_t *_instrCounter;

//! \brief Platform and Backend strings
const char _platformName[] = "qdma";
const char _backendName[] = "hwruntime";

//! Check that libxdma version is compatible
#if !defined(LIBXDMA_VERSION_MAJOR) || LIBXDMA_VERSION_MAJOR < 3
#error Installed libxdma is not supported (use >= 3.0)
#endif

static int read_cluster_config() {
    const char *filename = getenv("XTASKS_CLUSTER_FILE");
    if (filename == NULL) {
        filename = "xtasks.cluster";
    }
    FILE *file = fopen(filename, "r");
    if (file == NULL) {  // Assuming we are in single-node mode
        _nnodes = 0;
        _cluster_size = 0;
        _nodeid = 0;
        _client_node = 0;
        return 0;
    }

    int r;
    r = fscanf(file, "%d %d %d", &_cluster_size, &_nnodes, &_client_node);
    if (r != 3) {
        fprintf(stderr, "xtasks.cluster format not recognized\n");
        fclose(file);
        return 1;
    }
    for (int i = 0; i < _cluster_size; ++i) {
        r = fscanf(file, "%d %d", &_fpgaid2localid[i], &_fpgaid2nodeid[i]);
        if (r != 2) {
            fprintf(stderr, "xtasks.cluster format not recognized\n");
            fclose(file);
            return 1;
        }
    }
    char myhostname[32];
    if (gethostname(myhostname, 31)) {
        perror("gethostname with 31 len errno");
        return 1;
    }
    _nodeid = -1;
    for (int i = 0; i < _nnodes; ++i) {
        char ip[16];  // Longest IP string is 15 char (255.255.255.255)
        char fhostname[32];
        r = fscanf(file, "%15s %d %31s", ip, &_nodeid2port[i], fhostname);
        if (r != 3) {
            fprintf(stderr, "xtasks.cluster format not recognized\n");
            fclose(file);
            return 1;
        }
        if (inet_pton(AF_INET, ip, &_nodeid2ip[i]) != 1) {
            fprintf(stderr, "Invalid IP address %s\n", ip);
            fclose(file);
            return 1;
        }
        if (strcmp(fhostname, myhostname) == 0) {
            if (_nodeid == -1) {
                _nodeid = i;
            } else {
                fprintf(stderr, "Found duplicated hostname %s in xtasks.cluster\n", fhostname);
                fclose(file);
                return 1;
            }
        }
    }
    fclose(file);
    if (_nodeid == -1) {
        fprintf(stderr, "Could not find hostname %s in xtasks.cluster\n", myhostname);
        return 1;
    }

    return 0;
}

static int init_sockets() {
    struct sockaddr_in servaddr;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;

    int curnode = 0;
    for (int n = 0; n < _nnodes; ++n) {
        if (n == _nodeid) continue;

        _sockfd[n] = socket(AF_INET, SOCK_STREAM, 0);
        if (_sockfd[n] < 0) {
            perror("Could not open socket");
            goto err;
        }
        ++curnode;

        servaddr.sin_addr = _nodeid2ip[n];
        servaddr.sin_port = htons(_nodeid2port[n]);

        if (connect(_sockfd[n], (const struct sockaddr *)&servaddr, sizeof(servaddr)) != 0) {
            perror("Error connecting");
            goto err;
        }
    }

    return 0;

err:
    for (int n = 0; n < curnode; ++n) {
        if (n != _nodeid) close(_sockfd[n]);
    }
    return 1;
}

xtasks_stat xtasksInit() {
    xdma_status st;
    xtasks_stat ret;
    int ndevs;
    char **devNames;
    char *pciDevListStr;
    _ndevs = 0;

    ret = getPciDevListStr(&pciDevListStr);
    if (ret != XTASKS_SUCCESS) {
        return ret;
    }
    ret = getPciDevList(pciDevListStr, &ndevs, &devNames);
    if (ret != XTASKS_SUCCESS) {
        goto init_pci_dev_list_err;
    }

    if (read_cluster_config()) {
        goto init_read_cluster_config_err;
    }

    st = xdmaInit();
    if (st != XDMA_SUCCESS) {
        PRINT_ERROR("Could not initialize XDMA\n");
        ret = XTASKS_ERROR;
        goto init_xdma_err;
    }

    // Init sockets if I'm the client node
    if (_nodeid == _client_node && init_sockets()) {
        goto init_sockets_err;
    }

    int curdevMap = 0;
    for (int i = 0; i < ndevs; ++i, ++curdevMap) {
        uint32_t *pciBar;
        ret = mapPciDevice(devNames[i], &pciBar);
        if (ret != XTASKS_SUCCESS) {
            goto init_map_bar_err;
        }
        _pciBar[i] = pciBar;
    }

    _ndevs = ndevs;

    uint32_t *bitInfo = malloc(BITINFO_MAX_SIZE);
    if (bitInfo == NULL) {
        PRINT_ERROR("Could not allocate memory for the bitinfo");
        goto init_alloc_bitinfo_err;
    }

    int curdevBitinfo = 0;
    for (int d = 0; d < ndevs; ++d, ++curdevBitinfo) {
        memcpy(bitInfo, (void *)(_pciBar[d] + BITINFO_ADDRESS / sizeof(*_pciBar[0])), BITINFO_MAX_SIZE);

        // Check if bitstream is compatible
        bit_compatibility_t compat = checkbitstreamCompatibility(bitInfo);
        if (compat == BIT_NO_COMPAT) {
            ret = XTASKS_ENOAV;
            goto init_compat_err;
        }

        uint64_t cmdInAddr = bitinfo_get_cmd_in_addr(bitInfo);
        _cmdInSubqueueLen[d] = bitinfo_get_cmd_in_len(bitInfo);
        uint64_t cmdOutAddr = bitinfo_get_cmd_out_addr(bitInfo);
        _cmdOutSubqueueLen[d] = bitinfo_get_cmd_out_len(bitInfo);
        uint64_t hwruntime_rst_address = bitinfo_get_managed_rstn_addr(bitInfo);
        _hwcounter_address[d] = bitinfo_get_hwcounter_addr(bitInfo);
        _numAccs[d] = bitinfo_get_acc_count(bitInfo);
        _memorySizes[d] = bitinfo_get_memory_size(bitInfo);

        if (_nnodes == 0) {
            uint32_t numAccTypes = bitinfo_get_acc_type_count(bitInfo);
            _accs[d] = malloc(_numAccs[d] * sizeof(acc_t));
            if (_accs[d] == NULL) {
                PRINT_ERROR("Could not allocate accelerators array");
                goto init_alloc_acc_err;
            }
            _acc_types[d] = malloc(numAccTypes * sizeof(bit_acc_type_t));
            if (_acc_types[d] == NULL) {
                PRINT_ERROR("Could not allocate accelerator types array");
                goto init_alloc_acctype_err;
            }

            bitinfo_init_acc_types(bitInfo, _acc_types[d]);
            initAccList(d, _accs[d], _acc_types[d], numAccTypes, _cmdInSubqueueLen[d]);
        }

        // Initialize command queues
        _cmdInQueue[d] = (uint64_t *)(_pciBar[d] + (cmdInAddr / sizeof(*_pciBar[0])));
        // memset(_cmdInQueue, 0, _cmdInSubqueueLen * _numAccs * sizeof(*_cmdInQueue));
        for (int i = 0; i < _cmdInSubqueueLen[d] * _numAccs[d]; i++) {
            _cmdInQueue[d][i] = 0;
        }
        _cmdOutQueue[d] = (uint64_t *)(_pciBar[d] + (cmdOutAddr / sizeof(*_pciBar[0])));
        // memset(_cmdOutQueue, 0, _cmdOutSubqueueLen * _numAccs * sizeof(*_cmdOutQueue));
        for (int i = 0; i < _cmdOutSubqueueLen[d] * _numAccs[d]; i++) {
            _cmdOutQueue[d][i] = 0;
        }
        // initialize reset
        _hwruntimeRst[d] = (uint32_t *)(_pciBar[d] + (hwruntime_rst_address / sizeof(*_pciBar[0])));
        resetHWRuntime(_hwruntimeRst[d]);

        // Initialize power/thermal monitor if enabled
        if (bitinfo_get_feature(bitInfo, BIT_FEATURE_SYSMON_EN)) {
            _sysmonAddr[d] = (uint32_t *)_pciBar[d] + (bitinfo_get_sysmon_addr(bitInfo) / sizeof(*_pciBar[d]));
        } else {
            _sysmonAddr[d] = NULL;
        }
        if (bitinfo_get_feature(bitInfo, BIT_FEATURE_CMS_EN)) {
            _cmsAddr[d] = (uint32_t *)_pciBar[d] + (bitinfo_get_cms_addr(bitInfo) / sizeof(*_pciBar[d]));
        } else {
            _cmsAddr[d] = NULL;
        }

        bool feature = bitinfo_get_feature(bitInfo, BIT_FEATURE_SPAWN_Q);
        if (!feature) {
            _spawnOutQueue[d] = NULL;
            _spawnInQueue[d] = NULL;
        } else {
            uint64_t spawnInAddr = bitinfo_get_spawn_in_addr(bitInfo);
            _spawnInQueueLen[d] = bitinfo_get_spawn_in_len(bitInfo);
            uint64_t spawnOutAddr = bitinfo_get_spawn_out_addr(bitInfo);
            _spawnOutQueueLen[d] = bitinfo_get_spawn_out_len(bitInfo);
            _spawnOutQueue[d] = (uint64_t *)(_pciBar[d] + spawnOutAddr / sizeof(*_pciBar[0]));
            _spawnInQueue[d] = (uint64_t *)(_pciBar[d] + spawnInAddr / sizeof(*_pciBar[0]));
            _spawnInQueueIdx[d] = 0;
            _spawnOutQueueIdx[d] = 0;

            // Invalidate entries
            for (unsigned int i = 0; i < _spawnOutQueueLen[d]; i++) {
                _spawnOutQueue[d][i] = 0;
            }
            for (unsigned int i = 0; i < _spawnInQueueLen[d]; i++) {
                _spawnInQueue[d][i] = 0;
            }
        }
    }

    // Use the last bitinfo, assuming all FPGAs have the same bitstream
    int curDevCluster = 0;
    for (int d = 0; d < _cluster_size; ++curDevCluster, ++d) {
        _numAccs[d] = bitinfo_get_acc_count(bitInfo);
        uint32_t numAccTypes = bitinfo_get_acc_type_count(bitInfo);
        _accs[d] = malloc(_numAccs[d] * sizeof(acc_t));
        if (_accs[d] == NULL) {
            PRINT_ERROR("Could not allocate accelerators array");
            goto init_c_alloc_acc_err;
        }
        _acc_types[d] = malloc(numAccTypes * sizeof(bit_acc_type_t));
        if (_acc_types[d] == NULL) {
            PRINT_ERROR("Could not allocate accelerator types array");
            goto init_c_alloc_acctype_err;
        }

        bitinfo_init_acc_types(bitInfo, _acc_types[d]);
        initAccList(d, _accs[d], _acc_types[d], numAccTypes, _cmdInSubqueueLen[0]);
    }

    // Allocate tasks array
    _tasks = malloc(NUM_RUN_TASKS * sizeof(task_t));
    if (_tasks == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for tasks");
        goto init_alloc_tasks_err;
    }
    _cmdExecTaskBuff = (uint8_t *)malloc(NUM_RUN_TASKS * DEF_EXEC_TASK_SIZE);
    if (_cmdExecTaskBuff == NULL) {
        ret = XTASKS_ENOMEM;
        PRINT_ERROR("Cannot allocate memory for exec. tasks buffer");
        goto init_alloc_exec_tasks_err;
    }
    for (size_t idx = 0; idx < NUM_RUN_TASKS; ++idx) {
        _tasks[idx].id = 0;
        _tasks[idx].cmdHeader = (cmd_header_t *)&_cmdExecTaskBuff[idx * DEF_EXEC_TASK_SIZE];
        _tasks[idx].cmdExecArgs = (cmd_exec_task_arg_t *)0;
        _tasks[idx].extSize = 0;
        _tasks[idx].periTask = 0;
    }

    // Check for instrumentation
    _instrAvail = bitinfo_get_feature(bitInfo, BIT_FEATURE_INST);

    // Set the memory size in xdma
    if (_nnodes == 0) {
        xdmaSetMemorySizes(_memorySizes);
    } else {
        // For the moment, we can't get the bitinfo from remote FPGAs.
        // Assume all FPGAs have the same memory.
        uint32_t size = _memorySizes[0];
        for (int i = 0; i < _cluster_size; ++i) {
            uint32_t s;
            if (_fpgaid2nodeid[i] == _nodeid)
                s = _memorySizes[_fpgaid2localid[i]];
            else
                s = size;
            xdmaSetMemorySize(i, s);
        }
    }

    free(bitInfo);
    free(devNames);
    free(pciDevListStr);

    return XTASKS_SUCCESS;

    // Error cleanup
init_alloc_exec_tasks_err:
    free(_tasks);
init_alloc_tasks_err:
init_c_alloc_acctype_err:
    if (_nnodes != 0 && curDevCluster < _cluster_size) free(_accs[curDevCluster]);
init_c_alloc_acc_err:
    for (int d = 0; d < curDevCluster; ++d) {
        free(_accs[d]);
        free(_acc_types[d]);
    }
init_alloc_acctype_err:
    if (curdevBitinfo < ndevs) free(_accs[curdevBitinfo]);
init_alloc_acc_err:
init_compat_err:
    for (int d = 0; d < curdevBitinfo; ++d) {
        if (_nnodes == 0) {
            free(_accs[d]);
            free(_acc_types[d]);
        }
    }
    free(bitInfo);
init_alloc_bitinfo_err:
init_map_bar_err:
    for (int d = 0; d < curdevMap; ++d) unmapPciDev((uint32_t *)_pciBar[d]);
    if (_nodeid == _client_node)
        for (int n = 0; n < _nnodes; ++n) {
            if (n != _nodeid) close(_sockfd[n]);
        }
init_sockets_err:
    xdmaFini();
init_xdma_err:
    free(devNames);
init_read_cluster_config_err:
init_pci_dev_list_err:
    free(pciDevListStr);
    return ret;
}

xtasks_stat xtasksInitHWIns(const size_t nEvents) {
    xtasks_stat ret;
    size_t insBufferSize;

    if (nEvents < 1) {
        return XTASKS_EINVAL;
    }

    if (!_instrAvail) {
        return XTASKS_ENOAV;
    }

    _numInstrEvents = nEvents;
    insBufferSize = _numInstrEvents * _numAccs[0] * sizeof(xtasks_ins_event);

    xdma_status s = xdmaAllocate(0, &_instrBuffHandle, insBufferSize);
    if (s != XDMA_SUCCESS) {
        return XTASKS_ENOMEM;
    }
    unsigned long phyAddr;
    xdmaGetDeviceAddress(_instrBuffHandle, &phyAddr);
    _instrBuffPhy = (xtasks_ins_event *)((uintptr_t)phyAddr);

    // Invalidate all entries
    xtasks_ins_event *_invalBuffer = (xtasks_ins_event *)malloc(insBufferSize);
    for (unsigned int i = 0; i < _numInstrEvents * _numAccs[0]; ++i) {
        _invalBuffer[i].eventType = XTASKS_EVENT_TYPE_INVALID;
    }
    s = xdmaMemcpy(_invalBuffer, _instrBuffHandle, insBufferSize, 0, XDMA_TO_DEVICE);
    free(_invalBuffer);
    if (s != XDMA_SUCCESS) {
        PRINT_ERROR("Could not initialize instrumentation buffer");
        ret = XTASKS_ERROR;
        goto hwins_buff_init_err;
    }

    // Send instr setup command to acc
    cmd_setup_hw_ins_t cmd;
    cmd.header.commandCode = CMD_SETUP_INS_CODE;
    memcpy((void *)cmd.header.commandArgs, (void *)&_numInstrEvents, CMD_SETUP_INS_ARGS_NUMEVS_BYTES);
    for (size_t i = 0; i < _numAccs[0]; ++i) {
        cmd.bufferAddr = (uintptr_t)(_instrBuffPhy + _numInstrEvents * i);

        ret = submitCommand(_accs[0] + i, (uint64_t *)&cmd, sizeof(cmd_setup_hw_ins_t) / sizeof(uint64_t),
                            _cmdInQueue[0], _cmdInSubqueueLen[0]);
        if (ret != XTASKS_SUCCESS) {
            PRINT_ERROR("Error setting up instrumentation");
            goto hwins_cmd_err;
        }

        _accs[0][i].instrIdx = 0;
        _accs[0][i].instrLock = 0;
    }

    // Init HW counter
    if (_pciBar[0] == NULL) {
        ret = XTASKS_ERROR;
        PRINT_ERROR("xtasks has not been initialized");
        goto hwins_no_pcibar;
    }

    _instrCounter = (uint64_t *)(_pciBar[0] + _hwcounter_address[0] / sizeof(*_pciBar[0]));

    return XTASKS_SUCCESS;

hwins_no_pcibar:
hwins_cmd_err:
hwins_buff_init_err:
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuffPhy = NULL;
    return ret;
}

xtasks_stat xtasksFiniHWIns() {
    if (_numInstrEvents == 0) return XTASKS_SUCCESS;  // Instrumentation is not initialized
    xdmaFree(_instrBuffHandle);
    _numInstrEvents = 0;
    _instrBuffPhy = NULL;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksFini() {
    free(_cmdExecTaskBuff);
    free(_tasks);
    for (int d = 0; d < _ndevs; ++d) {
        unmapPciDev((uint32_t *)_pciBar[d]);  // Need the cast to drop volatile qualifier
        if (_nnodes == 0) {
            free(_acc_types[d]);
            free(_accs[d]);
        }
    }
    for (int d = 0; d < _cluster_size; ++d) {
        free(_acc_types[d]);
        free(_accs[d]);
    }
    if (_nodeid == _client_node)
        for (int n = 0; n < _nnodes; ++n) {
            if (n != _nodeid) close(_sockfd[n]);
        }
    xdmaFini();
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetPlatform(const char **name) {
    if (name == NULL) return XTASKS_EINVAL;
    *name = _platformName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetBackend(const char **name) {
    if (name == NULL) return XTASKS_EINVAL;
    *name = _backendName;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumDevices(int *numDevices) {
    if (numDevices == NULL) return XTASKS_EINVAL;
    if (_nnodes == 0) {
        *numDevices = _ndevs;
    } else {
        *numDevices = _cluster_size;
    }
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetNumAccs(int devId, size_t *count) {
    int maxDevs = _nnodes == 0 ? _ndevs : _cluster_size;
    if (devId >= maxDevs || count == NULL) return XTASKS_EINVAL;
    *count = _numAccs[devId];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccs(int devId, size_t const maxCount, xtasks_acc_handle *array, size_t *count) {
    if (array == NULL || count == NULL) return XTASKS_EINVAL;

    size_t tmp = maxCount > _numAccs[devId] ? _numAccs[devId] : maxCount;
    for (size_t i = 0; i < tmp; ++i) {
        array[i] = (xtasks_acc_handle)(&_accs[devId][i]);
    }
    *count = tmp;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info *info) {
    if (info == NULL) return XTASKS_EINVAL;

    acc_t *ptr = (acc_t *)handle;
    *info = ptr->info;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId, xtasks_task_id const parent,
                             xtasks_comp_flags const compute, xtasks_task_handle *handle) {
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOENTRY;
    }

    initializeTask(&_tasks[idx], id, accId, parent, compute);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksCreatePeriodicTask(xtasks_task_id const id, xtasks_acc_handle const accId,
                                     xtasks_task_id const parent, xtasks_comp_flags const compute,
                                     unsigned int const numReps, unsigned int const period,
                                     xtasks_task_handle *handle) {
    int idx = getFreeTaskEntry(_tasks);
    if (idx < 0) {
        return XTASKS_ENOMEM;
    }

    initializePeriodicTask(&_tasks[idx], id, accId, parent, compute, numReps, period);

    *handle = (xtasks_task_handle)&_tasks[idx];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle *handle) {
    task_t *task = (task_t *)(*handle);
    *handle = NULL;
    task->id = 0;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags, xtasks_arg_val const value,
                         xtasks_task_handle const handle) {
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt + 1 > DEF_EXEC_TASK_ARGS_LEN) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    }

    argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET]++;
    task->cmdExecArgs[argsCnt].flags = flags;
    task->cmdExecArgs[argsCnt].id = id;
    task->cmdExecArgs[argsCnt].value = value;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags, xtasks_arg_val *const values,
                          xtasks_task_handle const handle) {
    task_t *task = (task_t *)(handle);
    uint8_t argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    if (argsCnt + num > DEF_EXEC_TASK_ARGS_LEN) {
        // Unsupported number of arguments
        return XTASKS_ENOSYS;
    }

    for (size_t i = 0, idx = argsCnt; i < num; ++i, ++idx) {
        task->cmdExecArgs[idx].flags = flags;
        task->cmdExecArgs[idx].id = idx;
        task->cmdExecArgs[idx].value = values[i];
    }
    task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET] += num;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle) {
    task_t *task = (task_t *)(handle);
    acc_t *acc = (acc_t *)task->accel;
    int devId = acc->devId;

    // Update the task/command information
    task->cmdHeader->valid = QUEUE_VALID;

    // NOTE: cmdExecArgs array is after the header
    uint8_t const argsCnt = task->cmdHeader->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
    size_t const numHeaderBytes = task->periTask ? sizeof(cmd_peri_task_header_t) : sizeof(cmd_exec_task_header_t);
    size_t const numCmdWords = (numHeaderBytes + sizeof(cmd_exec_task_arg_t) * argsCnt) / sizeof(uint64_t);

    int localId;
    int nodeId;
    if (_nnodes != 0) {
        localId = _fpgaid2localid[devId];
        nodeId = _fpgaid2nodeid[devId];
    } else {
        localId = devId;
        nodeId = _nodeid;
    }

    if (nodeId != _nodeid) {
        int sockfd = _sockfd[nodeId];
        uint32_t header = 1;  // Submit task code
        header |= (uint32_t)argsCnt << 16;
        ssize_t n = send(sockfd, &header, sizeof(header), MSG_MORE);
        n += send(sockfd, task->cmdHeader, numCmdWords * sizeof(uint64_t), MSG_MORE);
        if ((size_t)n != sizeof(header) + numCmdWords * sizeof(uint64_t)) {
            fprintf(stderr, "Expected to send %lu bytes but found %ld\n",
                    sizeof(header) + numCmdWords * sizeof(uint64_t), n);
            return XTASKS_ERROR;
        }
        int accid = (int)(acc - _accs[devId]);
        uint64_t data[2];
        data[0] = devId | ((uint64_t)accid << 32);
        data[1] = (uint64_t)handle;
        n = send(sockfd, data, sizeof(data), 0);
        if (n < 0) {
            perror("Error in send");
            return XTASKS_ERROR;
        }
        return XTASKS_SUCCESS;
    }

    return submitCommand(acc, (uint64_t *)task->cmdHeader, numCmdWords, _cmdInQueue[localId],
                         _cmdInSubqueueLen[localId]);
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_handle *handle, xtasks_task_id *id) {
    if (handle == NULL || id == NULL) {
        return XTASKS_EINVAL;
    }
    if (_nnodes != 0) return XTASKS_ENOSYS;

    xtasks_stat ret = XTASKS_PENDING;
    for (int d = 0; d < _ndevs; ++d) {
        ret = xtasksTryGetFinishedTaskDev(d, handle, id);
        if (ret != XTASKS_PENDING) return ret;
    }

    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskDev(int devId, xtasks_task_handle *handle, xtasks_task_id *id) {
    if (handle == NULL || id == NULL) {
        return XTASKS_EINVAL;
    }

    if (_nnodes != 0 && _fpgaid2nodeid[devId] != _nodeid) {
        const int nodeId = _fpgaid2nodeid[devId];
        const int sockfd = _sockfd[nodeId];
        uint64_t taskID;
        ssize_t n = recv(sockfd, &taskID, sizeof(taskID), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN) return XTASKS_PENDING;
            fprintf(stderr, "Error in recv\n");
            return XTASKS_ERROR;
        } else if (n != sizeof(taskID)) {
            fprintf(stderr, "Expected to receive %lu but found %ld\n", sizeof(taskID), n);
            return XTASKS_ERROR;
        }
        uintptr_t taskPtr = (uintptr_t)taskID;
        task_t *task = (task_t *)taskPtr;
        *handle = (xtasks_task_handle)task;
        *id = task->id;

        // Mark the task as executed (using the valid field as it is not used in the cached copy)<
        task->cmdHeader->valid = QUEUE_INVALID;
        return XTASKS_SUCCESS;
    }

    xtasks_stat ret = XTASKS_PENDING;
    size_t i = 0;
    do {
        ret = xtasksTryGetFinishedTaskAccel(&_accs[devId][i], handle, id);
    } while (++i < _numAccs[devId] && ret == XTASKS_PENDING);

    return ret;
}

xtasks_stat xtasksTryGetFinishedTaskAccel(xtasks_acc_handle const accel, xtasks_task_handle *handle,
                                          xtasks_task_id *id) {
    acc_t *acc = (acc_t *)accel;
    int devId = acc->devId;
    int localId = _nnodes == 0 ? devId : _fpgaid2localid[devId];
    volatile uint64_t *const subqueue = _cmdOutQueue[localId] + acc->info.id * _cmdOutSubqueueLen[localId];

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
            size_t const dataIdx = (idx + 1) % _cmdOutSubqueueLen[localId];
            uint64_t const taskID = subqueue[dataIdx];
            subqueue[dataIdx] = 0;  //< Clean the buffer slot

            // Invalidate the buffer entry
            cmd = (cmd_header_t *)&subqueue[idx];
            cmd->valid = QUEUE_INVALID;

            // Update the read index and release the lock
            acc->cmdOutIdx = (idx + sizeof(cmd_out_exec_task_t) / sizeof(uint64_t)) % _cmdOutSubqueueLen[localId];
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

xtasks_stat xtasksGetInstrumentData(xtasks_acc_handle const accel, xtasks_ins_event *events, size_t const maxCount) {
    acc_t *acc = (acc_t *)(accel);
    size_t count, validEvents;
    int devId = acc->devId;

    if (events == NULL || (acc - _accs[devId]) >= _numAccs[devId] || maxCount <= 0) return XTASKS_EINVAL;

    count = min(maxCount, _numInstrEvents - acc->instrIdx);
    validEvents = getAccEvents(acc, events, count, _numInstrEvents, _instrBuffHandle, INSTR_BUF_NO_INVALIDATE);
    if (validEvents < 0) {
        return XTASKS_ERROR;
    }
    if (validEvents < maxCount) {
        // Ensure invalid type of first non-wrote event slot in the caller buffer
        events[validEvents].eventType = XTASKS_EVENT_TYPE_INVALID;
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksTryGetNewTask(xtasks_newtask **task) {
    int devId = 0;
    if (_spawnOutQueue[devId] == NULL) return XTASKS_PENDING;

    // Get a non-empty slot into the spawn out queue
    size_t idx, next, taskSize;
    new_task_header_t *hwTaskHeader;
    do {
        idx = _spawnOutQueueIdx[devId];
        hwTaskHeader = (new_task_header_t *)(&_spawnOutQueue[devId][idx]);
        if (hwTaskHeader->valid != QUEUE_VALID) {
            return XTASKS_PENDING;
        }
        taskSize =
            (sizeof(new_task_header_t) + sizeof(uint64_t) * hwTaskHeader->numArgs +
             sizeof(new_task_dep_t) * hwTaskHeader->numDeps + sizeof(new_task_copy_t) * hwTaskHeader->numCopies) /
            sizeof(uint64_t);
        next = (idx + taskSize) % _spawnOutQueueLen[devId];
    } while (!__sync_bool_compare_and_swap(&_spawnOutQueueIdx[devId], idx, next));

    getNewTaskFromQ(task, _spawnOutQueue[devId], idx, _spawnOutQueueLen[devId]);
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksNotifyFinishedTask(xtasks_task_id const parent, xtasks_task_id const id) {
    int devId = 0;
    // Get an empty slot into the TM remote finished queue
    size_t idx, next;
    rem_fini_task_t *entryHeader;
    do {
        idx = _spawnInQueueIdx[devId];
        entryHeader = (rem_fini_task_t *)(&_spawnInQueue[devId][idx]);
        if (entryHeader->valid == QUEUE_VALID) {
            return XTASKS_ENOENTRY;
        }
        next = (idx + sizeof(rem_fini_task_t) / sizeof(uint64_t)) % _spawnInQueueLen[devId];
    } while (!__sync_bool_compare_and_swap(&_spawnInQueueIdx[devId], idx, next));

    // NOTE: rem_fini_task_t->taskId is the 1st word
    idx = (idx + 1) % _spawnInQueueLen[devId];
    _spawnInQueue[devId][idx] = id;

    // NOTE: rem_fini_task_t->parentId is the 2nd word
    idx = (idx + 1) % _spawnInQueueLen[devId];
    _spawnInQueue[devId][idx] = parent;

    __sync_synchronize();
    entryHeader->valid = QUEUE_VALID;

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksGetMemorySize(int devId, uint32_t *size) {
    if (_fpgaid2nodeid[devId] != _nodeid) return XTASKS_ENOSYS;
    int localId = _fpgaid2localid[devId];

    *size = _memorySizes[localId];
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksMalloc(int devId, size_t len, xtasks_mem_handle *handle) {
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaAllocate(devId, handle, len);
    return toXtasksStat(status);
}

xtasks_stat xtasksFree(xtasks_mem_handle handle) {
    xdma_status status = xdmaFree(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetAccAddress(xtasks_mem_handle const handle, uint64_t *addr) {
    if (addr == NULL) return XTASKS_EINVAL;

    unsigned long devAddr = 0;
    xdma_status status = xdmaGetDeviceAddress(handle, &devAddr);
    *addr = devAddr;
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpy(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
                         xtasks_memcpy_kind const kind) {
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpy(usr, handle, len, offset, mode);
    return toXtasksStat(status);
}

xtasks_stat xtasksMemcpyAsync(xtasks_mem_handle const handle, size_t offset, size_t len, void *usr,
                              xtasks_memcpy_kind const kind, xtasks_memcpy_handle *cpyHandle) {
    if (handle == NULL) return XTASKS_EINVAL;
    xdma_dir mode = kind == XTASKS_ACC_TO_HOST ? XDMA_FROM_DEVICE : XDMA_TO_DEVICE;
    xdma_status status = xdmaMemcpyAsync(usr, handle, len, offset, mode, cpyHandle);
    return toXtasksStat(status);
}

xtasks_stat xtasksTestCopy(xtasks_memcpy_handle *handle) {
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaTestTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksSyncCopy(xtasks_memcpy_handle *handle) {
    if (handle == NULL) return XTASKS_EINVAL;

    xdma_status status = xdmaWaitTransfer(handle);
    return toXtasksStat(status);
}

xtasks_stat xtasksGetAccCurrentTime(xtasks_acc_handle const accel, xtasks_ins_timestamp *timestamp) {
    *timestamp = *_instrCounter;
    return XTASKS_SUCCESS;
}

xtasks_stat xtasksStartMonitor(int devId) {
    xtasks_stat ret = XTASKS_SUCCESS;
    if (_cmsAddr[devId]) {
        int err;
        err = cms_enable_power_monitor(_cmsAddr[devId]);
        if (err) {
            ret = XTASKS_ERROR;
        }
    } else {
        ret = XTASKS_ENOENTRY;
    }
    return ret;
}

xtasks_stat xtasksStopMonitor(int devId) {
    if (_cmsAddr[devId]) {
        cms_stop_power_monitor(_cmsAddr[devId]);
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksResetMonitor(int devId) {
    int err;
    xtasks_stat ret = XTASKS_SUCCESS;
    if (_cmsAddr[devId]) {
        err = cms_reset_power_monitor(_cmsAddr[devId]);
        if (err) {
            ret = XTASKS_ENOENTRY;
        }
    }
    return ret;
}

xtasks_stat xtasksGetMonitorData(int devId, xtasks_monitor_info *info) {
    cms_power_monitor_read_values(_cmsAddr[devId], info);
    return XTASKS_SUCCESS;
}
