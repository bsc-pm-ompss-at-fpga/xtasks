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

#ifndef __LIBXTASKS_PLATFORM_H__
#define __LIBXTASKS_PLATFORM_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util/common.h"

#define BISTREAM_INFO_ADDRESS 0x00020000
#define BISTREAM_INFO_MIN_REV 4
#define BISTREAM_INFO_REV_IDX 0
#define BISTREAM_INFO_FEATURES_IDX 3
#define BISTREAM_INFO_WRAPPER_IDX 6

/*!
 * \brief Get the path of the configuration file
 *        The function allocates a buffer that caller must delete using free()
 * \return  Configuration file path, NULL on error
 */
char *getConfigFilePath()
{
    char *buffer = NULL;

    // 1st -> environment var
    const char *accMapPath = getenv("XTASKS_CONFIG_FILE");
    if (accMapPath != NULL) {
        buffer = malloc(sizeof(char) * strlen(accMapPath));
        strcpy(buffer, accMapPath);
    }

    return buffer;
}

/*!
 * \brief Prints an error message in STDERR about configuration file not found
 */
void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library cannot access the fpga configuration device file.\n");
    fprintf(stderr, "       You may set the configuration file path with");
    fprintf(stderr, " XTASKS_CONFIG_FILE environment variable.\n");
}

/*!
 * \brief Prints an error message in STDERR about bitstream compatibility
 */
void printErrorBitstreamCompatibility()
{
    fprintf(stderr, "ERROR: Loaded FPGA bitstream may not be compatible with this version of libxtasks.\n");
    fprintf(stderr, "       The compatible versions are: [%d,%d]\n", MIN_WRAPPER_VER, MAX_WRAPPER_VER);
    fprintf(stderr, "       Alternatively, you may disable the compatibility check setting");
    fprintf(stderr, " XTASKS_COMPATIBILITY_CHECK environment variable to 0.\n");
}

/*!
 * \brief Checks whether a bitstream feature is present in the current fpga configuration or not.
 *          checkbitstreamCompatibility must be called before calling this function.
 * \return  BIT_FEATURE_NO_AVAIL if the feature is not available
 *          BIT_FEATURE_AVAIL if the feature is available
 *          BIT_FEATURE_SKIP if the check was skipped due to user requirements
 *          BIT_FEATURE_UNKNOWN if the check cannot be done or failed
 */
bit_feature_t checkbitstreamFeature(const char *featureName, const uint32_t *bitinfo)
{
    const char *featuresCheck = getenv("XTASKS_FEATURES_CHECK");
    if (featuresCheck != NULL && featuresCheck[0] == '0') {
        return BIT_FEATURE_SKIP;
    } else if (featuresCheck != NULL && featuresCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_FEATURES_CHECK, must be 0 or 1. Ignoring it");
    }

    const int i = getBitinfoOffset(BISTREAM_INFO_FEATURES_IDX, bitinfo);
    if (i >= BITINFO_MAX_WORDS) return BIT_FEATURE_UNKNOWN;

    const uint32_t features = bitinfo[i];
    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    if (strcmp(featureName, "hwcounter") == 0) {
        available = features & 0x1 ? BIT_FEATURE_AVAIL : BIT_FEATURE_NO_AVAIL;
    } else if (strcmp(featureName, "hwruntime") == 0) {
        available = (features >> 6) & 0x1 ? BIT_FEATURE_AVAIL : BIT_FEATURE_NO_AVAIL;
    } else if (strcmp(featureName, "hwruntime_ext") == 0) {
        available = (features >> 7) & 0x1 ? BIT_FEATURE_AVAIL : BIT_FEATURE_NO_AVAIL;
    }

    return available;
}

/*!
 * \brief Checks whether the current fpga bitstream is compatible with the libxtasks version or not.
 * \return  BIT_NO_COMPAT if the bitstream is not compatible
 *          BIT_COMPAT if the bitstream is compatible
 *          BIT_COMPAT_SKIP if the check was skipped due to user requirements
 *          BIT_COMPAT_UNKNOWN if the compatibility cannot be determined or failed
 */
bit_compatibility_t checkbitstreamCompatibility(const uint32_t *bitinfo)
{
    const char *compatCheck = getenv("XTASKS_COMPATIBILITY_CHECK");
    if (compatCheck != NULL && compatCheck[0] == '0') {
        return BIT_COMPAT_SKIP;
    } else if (compatCheck != NULL && compatCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_COMPATIBILITY_CHECK, must be 0 or 1. Ignoring it");
    }

    // The bitstream info BRAM version is old
    const int bitinfoRev = bitinfo[getBitinfoOffset(BISTREAM_INFO_REV_IDX, bitinfo)];
    if (bitinfoRev < BISTREAM_INFO_MIN_REV) {
        return BIT_NO_COMPAT;
    }

    const int i = getBitinfoOffset(BISTREAM_INFO_WRAPPER_IDX, bitinfo);
    if (i >= BITINFO_MAX_WORDS) {
        return BIT_FEATURE_UNKNOWN;
    }

    const uint32_t version = bitinfo[i];
    return MIN_WRAPPER_VER <= version && version <= MAX_WRAPPER_VER ? BIT_COMPAT : BIT_NO_COMPAT;
}

#endif /* __LIBXTASKS_PLATFORM_H__ */
