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

#define STR_BUFFER_SIZE         128
#define PRINT_ERROR(_str_)      fprintf(stderr, "[xTasks ERROR]: %s\n", _str_)
//#define PRINT_ERROR(_str_)
#define XTASKS_CONFIG_FILE_PATH "/dev/ompss_fpga/bit_info/xtasks"
#define BIT_INFO_FEATURES_PATH  "/dev/ompss_fpga/bit_info/features"

#define CMD_EXEC_TASK_CODE                0x01 ///< Command code for execute task commands
#define CMD_SETUP_INS_CODE                0x02 ///< Command code for setup instrumentation info
#define CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET 0    ///< Offset of Num. Args. field in the commandArgs array
#define CMD_EXEC_TASK_ARGS_COMP_OFFSET    3    ///< Offset of Compute flag in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_OFFSET  4    ///< Offset of Destination id (where accel will send finish signal) in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_PS      0x1F ///< Processing System identifier for the destId field
#define CMD_EXEC_TASK_ARGS_DESTID_TM      0x11 ///< Task manager identifier for the destId field

#define max(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

#define min(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; })

typedef enum {
    BIT_FEATURE_NO_AVAIL = 0,
    BIT_FEATURE_AVAIL = 1,
    BIT_FEATURE_UNKNOWN = 2
} bit_feature_t;

//! \brief Command header type
typedef struct __attribute__ ((__packed__)) {
    uint8_t commandCode;     //[0  :7  ] Command code
    uint8_t commandArgs[6];  //[8  :55 ] Command arguments
    uint8_t valid;           //[56 :63 ] Valid entry
} cmd_header_t;

//! \brief Header of execute task command
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     parentID;   //[64 :123] Parent task identifier (may be null)
    uint64_t     taskID;     //[64 :123] Task identifier
} cmd_exec_task_header_t;

//! \brief Argument entry of execute task command
typedef struct __attribute__ ((__packed__)) {
    uint8_t flags;           //[0  :7  ] Flags
    uint8_t _padding[3];     //[8  :32 ]
    uint32_t id;             //[32 :63 ] Argument ID
    uint64_t value;          //[64 :127] Address
} cmd_exec_task_arg_t;

//! \brief Setup hw instrumentation command
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     bufferAddr; //[64 :123] Instrumentation buffer address
} cmd_setup_hw_ins_t;

/*!
 * \brief Get the path of the configuration file
 *        The function allocates a buffer that caller must delete using free()
 * \return  Configuration file path, NULL on error
 */
char * getConfigFilePath()
{
    char * buffer = NULL;

    //1st -> environment var
    const char * accMapPath = getenv("XTASKS_CONFIG_FILE");
    if (accMapPath != NULL) {
        buffer = malloc(sizeof(char)*max(STR_BUFFER_SIZE, strlen(accMapPath)));
        strcpy(buffer, accMapPath);
    }

    //2nd -> /dev/ompss_fpga/bit_info/xtasks
    if (buffer == NULL) {
        buffer = malloc(sizeof(char)*STR_BUFFER_SIZE);
        strcpy(buffer, XTASKS_CONFIG_FILE_PATH);
    }

    return buffer;
}

/*!
 * \brief Prints an error message in STDERR about configuration file not found
 */
void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library cannot access the fpga configuration device file.\n");
    fprintf(stderr, "       Ensure that file '%s' exists and it has read permissions.\n",
        XTASKS_CONFIG_FILE_PATH);
    fprintf(stderr, "       Alternatively, you may force the configuration file path with \
        XTASKS_CONFIG_FILE environment variable.\n");
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
 * \breif Set n times the byte c in dst
 * \note Assuming: n is multiple of 32, dst and src are aligned to 32 bits
 * \note Avoid optimizations for the function implementation as aarch64 cannot execute
 *       some fast copy instructions over non-cacheable memory
 */
static inline void __attribute__((optimize("O1")))
    _memset( void * dst, int c, size_t n )
{
#if __aarch64__
    uint32_t * d = (uint32_t *)dst;
    char cc = c;
    uint32_t v;
    v = cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    v = (v << 8) | cc;
    for (size_t i = 0; i < n/sizeof(uint32_t); ++i) {
        d[i] = v;
    }
#else
    memset(dst, c, n);
#endif
}

/*!
 * \brief Checks whether a bitstrem feature is present in the current fpga configuration or not
 * \return  BIT_FEATURE_NO_AVAIL if the feature is not available
 *          BIT_FEATURE_AVAIL if the feature is available
 *          BIT_FEATURE_UNKNOWN if the check cannot be done or failed
 *
 */
bit_feature_t checkBitstremFeature(const char * featureName) {
    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    char buffer[strlen(BIT_INFO_FEATURES_PATH) + strlen(featureName) + 1];

    strcpy(buffer, BIT_INFO_FEATURES_PATH);
    strcat(buffer, "/");
    strcat(buffer, featureName);
    FILE * infoFile = fopen(buffer, "r");
    if (infoFile != NULL) {
        fread(&buffer, sizeof(char), 1, infoFile);
        fclose(infoFile);
        available = buffer[0] == '1' ? BIT_FEATURE_AVAIL :
            (buffer[0] == '0' ? BIT_FEATURE_NO_AVAIL : BIT_FEATURE_UNKNOWN);
    }
    return available;
}

#endif /* __LIBXTASKS_COMMON_H__ */
