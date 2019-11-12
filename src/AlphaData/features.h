/*--------------------------------------------------------------------
  (C) Copyright 2017-2019 Barcelona Supercomputing Center
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <admxrc3.h>

#define PRINT_ERROR(_str_)      fprintf(stderr, "[xTasks ERROR]: %s\n", _str_)

#define BISTREAM_INFO_ADDRESS 0x0
#define BITINFO_FIELD_SEP 0xFFFFFFFF
#define BITINFO_MAX_WORDS 512
#define COMPATIBLE_WRAPPER_VER  1

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

/*!
 * \brief Checks whether a bitstream feature is present in the current fpga configuration or not
 * \return  BIT_FEATURE_NO_AVAIL if the feature is not available
 *          BIT_FEATURE_AVAIL if the feature is available
 *          BIT_FEATURE_SKIP if the check was skipped due to user requirements
 *          BIT_FEATURE_UNKNOWN if the check cannot be done or failed
 */
bit_feature_t checkbitstreamFeature(const char * featureName, ADMXRC3_HANDLE hDevice) {
    const char * featuresCheck = getenv("XTASKS_FEATURES_CHECK");
    if (featuresCheck != NULL && featuresCheck[0] == '0') {
        return BIT_FEATURE_SKIP;
    } else if (featuresCheck != NULL && featuresCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_FEATURES_CHECK, must be 0 or 1. Ignoring it");
    }

    uint32_t bitinfo[BITINFO_MAX_WORDS];
    ADMXRC3_STATUS stat = ADMXRC3_Read(hDevice, 1, 0, BISTREAM_INFO_ADDRESS, sizeof(uint32_t)*BITINFO_MAX_WORDS, &bitinfo);
    if (stat != ADMXRC3_SUCCESS) {
        return BIT_FEATURE_NO_AVAIL;
    }

    int i = 4;
    while (bitinfo[i] != BITINFO_FIELD_SEP)
        ++i;
    uint32_t features = bitinfo[i+1];
    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    if (strcmp(featureName, "hw_instrumentation") == 0) {
        available = features & 0x1 ? BIT_FEATURE_AVAIL:BIT_FEATURE_NO_AVAIL;
    }
    else if (strcmp(featureName, "task_manager") == 0) {
        available = (features >> 1) & 0x1 ? BIT_FEATURE_AVAIL:BIT_FEATURE_NO_AVAIL;
    }
    else if (strcmp(featureName, "ext_task_manager") == 0) {
        available = (features >> 2) & 0x1 ? BIT_FEATURE_AVAIL:BIT_FEATURE_NO_AVAIL;
    }

    return available;
}

/*!
 * \brief Checks whether the current fpga bitstream is compatible with the libxtasks version or not
 * \return  BIT_NO_COMPAT if the bitstream is not compatible
 *          BIT_COMPAT if the bitstream is compatible
 *          BIT_COMPAT_SKIP if the check was skipped due to user requirements
 *          BIT_COMPAT_UNKNOWN if the compatibility cannot be determined or failed
 */
bit_compatibility_t checkbitstreamCompatibility() {
    const char * compatCheck = getenv("XTASKS_COMPATIBILITY_CHECK");
    if (compatCheck != NULL && compatCheck[0] == '0') {
        return BIT_COMPAT_SKIP;
    } else if (compatCheck != NULL && compatCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_COMPATIBILITY_CHECK, must be 0 or 1. Ignoring it");
    }

    bit_compatibility_t compatible = BIT_COMPAT_UNKNOWN;
    FILE * infoFile = fopen(BIT_INFO_WRAPPER_PATH, "r");
    if (infoFile != NULL) {
        int wrapperVersion;
        if (fscanf(infoFile, "%d", &wrapperVersion) != 1 || wrapperVersion != COMPATIBLE_WRAPPER_VER) {
            //NOTE: If read value is not an integer, probably it is "?" which means that the
            //      bitstream is too old
            compatible = BIT_NO_COMPAT;
        }
    }
    return compatible;
}

#endif /* __LIBXTASKS_COMMON_H__ */
