/*
* Copyright (c) 2017, BSC (Barcelona Supercomputing Center)
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

#include <libxdma.h>
#include <sys/auxv.h>
#include <elf.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define STR_BUFFER_SIZE 128

typedef struct {
    xdma_device     xdmaDev;
    char            descBuffer[STR_BUFFER_SIZE];
    xtasks_acc_info info;
} acc_t;

static int _init_cnt = 0;  ///< Counter of calls to init/fini
static size_t _numAccs;    ///< Number of accelerators in the system
static acc_t * _accs;      ///< Accelerators data

static void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library requires reading a '.xtasks.config' file to obtain the");
    fprintf(stderr, "current FPGA configuration\n");
    fprintf(stderr, "       However, XTASKS_CONFIG_FILE environment variable was not defined and");
    fprintf(stderr, "library was not able to build the filename based on the application binary\n");
}

xtasks_stat xtasksInit()
{
    //Handle multiple inits
    int init_cnt = __sync_fetch_and_add(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    //Open libxdma
    if (xdmaOpen() != XDMA_SUCCESS) {
        err_0: __sync_sub_and_fetch(&_init_cnt, 1);
        return XTASKS_ERROR;
    }

    //Get the number of devices from libxdma
    int xdmaDevices;
    if (xdmaGetNumDevices(&xdmaDevices) != XDMA_SUCCESS) {
        err_1: _numAccs = 0;
        xdmaClose();
        goto err_0;
    }
    _numAccs = (size_t)xdmaDevices;

    //Preallocate accelerators array
    _accs = malloc(sizeof(acc_t)*_numAccs);

    //Generate the configuration file path
    char * buffer = malloc(sizeof(char)*STR_BUFFER_SIZE);
    const char * accMapPath = getenv("XTASKS_CONFIG_FILE"); //< 1st, environment var
    if (accMapPath == NULL) {
        accMapPath = (const char *)getauxval(AT_EXECFN); //< Get executable file path
        if ((strlen(accMapPath) + 14) > STR_BUFFER_SIZE) {
            free(buffer);
            buffer = malloc(sizeof(char)*(strlen(accMapPath) + 14));
        }
        strcpy(buffer, accMapPath);
        accMapPath = strcat(buffer, ".xtasks.config"); //< 2nd, exec file
        if (access(accMapPath, R_OK) == -1) {
            accMapPath = strrchr(accMapPath, '/'); //< 3rd, exec file in current dir
            if (accMapPath == NULL) {
                printErrorMsgCfgFile();
                err_2: free(buffer);
                free(_accs);
                goto err_1;
            }
            accMapPath++;
        }
    }

    //Open the configuration file and parse it
    FILE * accMapFile = fopen(accMapPath, "r");
    if (accMapFile == NULL) {
        fprintf(stderr, "ERROR: Cannot open file %s to read current FPGA configuration\n", accMapPath);
        goto err_2;
    }
    //NOTE: Assuming that the lines contain <128 characters
    buffer = fgets(buffer, STR_BUFFER_SIZE, accMapFile); //< Ignore 1st line, headers
    if (buffer == NULL) {
        goto err_2;
    }
    xtasks_acc_type t;
    size_t num, total;
    total = 0;
    while (fscanf(accMapFile, "%u %u %s", &t, &num, buffer) == 3) { //< Parse the file
    //while (fgets(buffer, STR_BUFFER_SIZE, accMapFile)) {
        total += num;
        for (size_t i = total - num; i < total && total <= _numAccs; ++i) {
            _accs[i].info.id = i;
            _accs[i].info.type = t;
            _accs[i].info.description = _accs[i].descBuffer;
            strcpy(_accs[i].descBuffer, buffer);
        }
    }
    fclose(accMapFile);
    free(buffer);

    //Update number of accelerators
    if (total > _numAccs) {
        fprintf(stderr, "WARN: xTasks configuration file contains %u ", total);
        fprintf(stderr, "accelerators, but only %u FPGA devices have been found\n", _numAccs);
        fprintf(stderr, "      Using only the first accelerators of configuration file\n");
    }
    _numAccs = (total < _numAccs) ? total : _numAccs;

    //Get device handles from libxdma for each accelerator
    xdma_device devs[_numAccs];
    int numAccs = _numAccs;
    if (xdmaGetDevices(numAccs, &devs[0], &xdmaDevices) != XDMA_SUCCESS || xdmaDevices != numAccs) {
        goto err_1;
        for (size_t i = 0; i < _numAccs; ++i) {
            _accs[i].xdmaDev = devs[i];
        }
    }

    return XTASKS_SUCCESS;
}

xtasks_stat xtasksFini()
{
    //Handle multiple inits
    int init_cnt = __sync_sub_and_fetch(&_init_cnt, 1);
    if (init_cnt > 0) return XTASKS_SUCCESS;

    _numAccs = 0;
    free(_accs);
    if (xdmaClose() != XDMA_SUCCESS) {
        return XTASKS_ERROR;
    }

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
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetAccInfo(xtasks_acc_handle const handle, xtasks_acc_info * info)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksCreateTask(xtasks_task_id const id, xtasks_acc_handle const accId,
    xtasks_comp_flags const compute, xtasks_task_handle * handle)
{
     return XTASKS_ENOSYS;
}

xtasks_stat xtasksDeleteTask(xtasks_task_handle * handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksAddArg(xtasks_arg_id const id, xtasks_arg_flags const flags,
    xtasks_arg_val const value, xtasks_task_handle const handle)
{
     return XTASKS_ENOSYS;
}

xtasks_stat xtasksAddArgs(size_t const num, xtasks_arg_flags const flags,
    xtasks_arg_val * const values, xtasks_task_handle const handle)
{
     return XTASKS_ENOSYS;
}

xtasks_stat xtasksSubmitTask(xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksWaitTask(xtasks_task_handle const handle)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksTryGetFinishedTask(xtasks_task_id * id)
{
    return XTASKS_ENOSYS;
}

xtasks_stat xtasksGetInstrumentData(xtasks_task_handle const handle, xtasks_ins_times * times)
{
    return XTASKS_ENOSYS;
}
