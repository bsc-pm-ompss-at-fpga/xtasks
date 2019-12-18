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

#ifndef __LIBXTASKS_COMMON_H__
#define __LIBXTASKS_COMMON_H__

#include "../libxtasks.h"

#include <stdio.h>
#include <string.h>
#include <libxdma.h>

#define PRINT_ERROR(_str_)      fprintf(stderr, "[xTasks ERROR]: %s\n", _str_)
#define MIN_WRAPPER_VER         1
#define MAX_WRAPPER_VER         3

#define CMD_EXEC_TASK_CODE                0x01 ///< Command code for execute task commands
#define CMD_SETUP_INS_CODE                0x02 ///< Command code for setup instrumentation info
#define CMD_FINI_EXEC_CODE                0x03 ///< Command code for finished execute task commands
#define CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET 0    ///< Offset of Num. Args. field in the commandArgs array
#define CMD_EXEC_TASK_ARGS_COMP_OFFSET    3    ///< Offset of Compute flag in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_OFFSET  4    ///< Offset of Destination id (where accel will send finish signal) in the commandArgs array
#define CMD_EXEC_TASK_ARGS_DESTID_PS      0x1F ///< Processing System identifier for the destId field
#define CMD_EXEC_TASK_ARGS_DESTID_TM      0x11 ///< Task manager identifier for the destId field
#define CMD_FINI_EXEC_ARGS_ACCID_OFFSET   0    ///< Offset of Accelerator ID field in the commandArgs array

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
    BIT_FEATURE_UNKNOWN = 2,
    BIT_FEATURE_SKIP = 3
} bit_feature_t;

typedef enum {
    BIT_NO_COMPAT = 0,
    BIT_COMPAT = 1,
    BIT_COMPAT_UNKNOWN = 2,
    BIT_COMPAT_SKIP = 3
} bit_compatibility_t;

//! \brief Command header type
typedef struct __attribute__ ((__packed__)) {
    uint8_t commandCode;     //[0  :7  ] Command code
    uint8_t commandArgs[6];  //[8  :55 ] Command arguments
    uint8_t valid;           //[56 :63 ] Valid entry
} cmd_header_t;

//! \brief Header of execute task command
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     taskID;     //[64 :127] Task identifier
    uint64_t     parentID;   //[128:195] Parent task identifier (may be null)
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
    uint64_t     bufferAddr; //[64 :127] Instrumentation buffer address
} cmd_setup_hw_ins_t;

//! \brief Response out command for execute task commands
typedef struct __attribute__ ((__packed__)) {
    cmd_header_t header;     //[0  :63 ] Command header
    uint64_t     taskID;     //[64 :127] Executed task identifier
} cmd_out_exec_task_t;

/*!
 * \brief Returns the length of command based on its header
 */
uint8_t getCmdLength(cmd_header_t const * header) {
    uint8_t length = 0;
    uint8_t const cmdCode = header->commandCode;
    if (cmdCode == CMD_EXEC_TASK_CODE) {
        uint8_t const numArgs = header->commandArgs[CMD_EXEC_TASK_ARGS_NUMARGS_OFFSET];
        uint8_t const argsLength = sizeof(cmd_exec_task_arg_t)/sizeof(uint64_t)*numArgs;
        length = sizeof(cmd_exec_task_header_t)/sizeof(uint64_t) + argsLength;
    } else if (cmdCode == CMD_SETUP_INS_CODE) {
        length = sizeof(cmd_setup_hw_ins_t)/sizeof(uint64_t);
    } else if (cmdCode == CMD_FINI_EXEC_CODE) {
        length = sizeof(cmd_out_exec_task_t)/sizeof(uint64_t);
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

#endif /* __LIBXTASKS_COMMON_H__ */
