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

#define BITINFO_ADDRESS 0x00020000

/*!
 * \brief Prints an error message in STDERR about configuration file not found
 */
void printErrorMsgCfgFile()
{
    fprintf(stderr, "ERROR: xTasks Library cannot access the fpga configuration device file.\n");
    fprintf(stderr, "       You may set the configuration file path with");
    fprintf(stderr, " XTASKS_CONFIG_FILE environment variable.\n");
}

int getAccRawInfo(char *accInfo, const uint32_t *rawBitInfo)
{
    char *filePath;

    int size;
    filePath = getenv("XTASKS_CONFIG_FILE");
    if (filePath) {  // User config takes precedence over bitinfo data
        FILE *accFile = fopen(filePath, "r");
        if (!accFile) {
            printErrorMsgCfgFile();
            return -1;
        }
        size = fread(accInfo, sizeof(*rawBitInfo), BITINFO_MAX_SIZE, accFile);
        fclose(accFile);
    } else {  // Read from bit info
        // Apply the offset directly as its returned in 32-bit words
        rawBitInfo += BITINFO_XTASKS_IDX;
        // Find out the size of the acc info field in
        int idx;
        for (idx = 0; rawBitInfo[idx] != BITINFO_FIELD_SEP; idx++)
            ;
        size = idx * sizeof(*rawBitInfo);
        int accInfoOffset = 20;  // 20 header characters
        int nTypes = idx / 11;   // 11 32-bit words per task type
        sprintf(accInfo, "type\t#ins\tname\tfreq\n");

        for (int i = 0; i < nTypes; ++i) {
            uint32_t firstWord = rawBitInfo[0];
            uint32_t secondWord = rawBitInfo[1];
            uint32_t thirdWord = rawBitInfo[2];
            rawBitInfo += 3;
            // nInstances = firstWord[15:0]
            int nInstances = firstWord & 0xFFFF;
            // taskType = {secondWord[23:0], firstWord[31:16]}
            uint64_t taskType = (firstWord >> 16) | (((uint64_t)secondWord & 0xFFFFFF) << 16);
            // accFreq = {thirdWord[15:0], secondWord[31:24]}
            int accFreq = (secondWord >> 24) | ((thirdWord & 0xFFFF) << 8);
            sprintf(accInfo + accInfoOffset, "%019lu\t%03d\t", taskType, nInstances);
            accInfoOffset += 24;
            int charOffset = 0;
            uint32_t curWord = *rawBitInfo;
            ++rawBitInfo;
            for (int j = 0; j < 31; ++j) {
                accInfo[accInfoOffset++] = (curWord >> charOffset * 8) & 0xFF;
                if ((j + 1) % sizeof(*rawBitInfo) == 0) {
                    charOffset = 0;
                    curWord = *rawBitInfo;
                    ++rawBitInfo;
                } else {
                    charOffset++;
                }
            }
            // bitinfo freq in KHz, xtasks_config expects MHz
            sprintf(accInfo + accInfoOffset, "\t%03d\n", accFreq / 1000);
            accInfoOffset += 5;
        }
    }
    return size;
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

    const uint32_t features = bitinfo[BITINFO_FEATURES_IDX];
    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    if (strcmp(featureName, "hwcounter") == 0) {
        available = features & 0x1 ? BIT_FEATURE_AVAIL : BIT_FEATURE_NO_AVAIL;
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
 */
bit_compatibility_t checkbitstreamCompatibility(const uint32_t *bitinfo)
{
    const char *compatCheck = getenv("XTASKS_COMPATIBILITY_CHECK");
    if (compatCheck != NULL && compatCheck[0] == '0') {
        return BIT_COMPAT_SKIP;
    } else if (compatCheck != NULL && compatCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_COMPATIBILITY_CHECK, must be 0 or 1. Ignoring it");
    }

    const unsigned int bitinfoRev = bitinfo[BITINFO_REV_IDX];
    // The bitstream info BRAM version is old
    if (bitinfoRev != BITINFO_MIN_REV) {
        printErrorBitstreamVersionCompatibility(bitinfoRev);
        return BIT_NO_COMPAT;
    }

    const unsigned int version = bitinfo[BITINFO_WRAPPER_IDX];
    if (version < MIN_WRAPPER_VER || version > MAX_WRAPPER_VER) {
        printErrorBitstreamWrapperCompatibility(version);
        return BIT_NO_COMPAT;
    }

    return BIT_COMPAT;
}

uint32_t getBitstreamNumAccs(const uint32_t *bitinfo) { return bitinfo[BITINFO_NUMACCS_IDX]; }

void getBitStreamHwrIOStruct(const uint32_t *bitinfo, uint32_t hwruntimeIOStruct[BITINFO_HWRIO_STRUCT_WORDS])
{
    for (int i = 0; i < BITINFO_HWRIO_STRUCT_WORDS; ++i) {
        hwruntimeIOStruct[i] = bitinfo[BITINFO_HWRIO_IDX + i];
    }
}

#endif /* __LIBXTASKS_PLATFORM_H__ */
