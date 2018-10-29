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

#ifndef __LIBXTASKS_COMMON_H__
#define __LIBXTASKS_COMMON_H__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/auxv.h>
#include <libxdma.h>

#define STR_BUFFER_SIZE     128
#define PRINT_ERROR(_str_)  fprintf(stderr, "[xTasks ERROR]: %s\n", _str_)
//#define PRINT_ERROR(_str_)

#define XTASKS_EXTENSION ".xtasks.config"
#define NANOX_EXTENSION ".nanox.config"
#define XTASKS_DEF_CONFIG_FILE "xtasks.config"

#define max(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

/*!
 * \brief Get the path of the configuration file
 *        The function allocates a buffer that caller must delete using free()
 * \return  Configuration file path, NULL on error
 */
char * getConfigFilePath()
{
    //1st -> environment var
    const char * accMapPath = getenv("XTASKS_CONFIG_FILE");
    if (accMapPath != NULL) {
        char * buffer = malloc(sizeof(char)*max(STR_BUFFER_SIZE, strlen(accMapPath)));
        if (buffer != NULL) {
            strcpy(buffer, accMapPath);
        }
        return buffer;
    }

    accMapPath = (const char *)getauxval(AT_EXECFN);
    size_t buffSize = max(STR_BUFFER_SIZE, strlen(accMapPath) + strlen(XTASKS_EXTENSION));
    char * buffer = malloc(sizeof(char)*buffSize);
    if (buffer != NULL) {
        //2nd -> executable file path
        strcpy(buffer, accMapPath);
        buffer = strcat(buffer, XTASKS_EXTENSION);
        if (access(buffer, R_OK) != -1) {
            return buffer;
        }

        //3rd -> exec file in current dir
        accMapPath = strrchr(buffer, '/');
        if (accMapPath == NULL) {
            //Uncontroled path
            free(buffer);
            return NULL;
        }
        accMapPath++; //< Remove the initial '/'
        memcpy(buffer, accMapPath, buffSize - (accMapPath - buffer)); //< Shift conntent to the beginning
        if (access(buffer, R_OK) != -1) {
            return buffer;
        }

        //4th -> XTASKS_DEF_CONFIG_FILE
        strcpy(buffer, XTASKS_DEF_CONFIG_FILE);
        if (access(buffer, R_OK) != -1) {
            return buffer;
        }

        //Configuration file not found
        free(buffer);
        buffer = NULL;
    }
    return buffer;
}

/*!
 * \brief Prints an error message in STDERR about configuration file not found
 */
void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library requires reading a formated file to obtain the ");
    fprintf(stderr, "current FPGA configuration.\n");
    fprintf(stderr, "       Available options are:\n");
    fprintf(stderr, "         1) Use XTASKS_CONFIG_FILE environment variable to define the file path.\n");
    fprintf(stderr, "         2) Create '<binary name>.xtasks.config' file (same location as binary) ");
    fprintf(stderr, "with the current FPGA configuration.\n");
    fprintf(stderr, "         3) Create './<binary name>.xtasks.config' file with the current FPGA ");
    fprintf(stderr, "configuration.\n");
    fprintf(stderr, "         4) Create './xtasks.config' file with the current FPGA configuration.\n");
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

#endif /* __LOCK_FREE_QUEUE_H__ */
