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

#include "../util/common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define XTASKS_CONFIG_FILE_PATH "/dev/ompss_fpga/bitinfo/xtasks"
#define BITINFO_FEATURES_PATH "/dev/ompss_fpga/bitinfo/features"
#define BITINFO_WRAPPER_PATH "/dev/ompss_fpga/bitinfo/wrapper_version"
#define BITINFO_HWRIO_RAW_PATH "/dev/ompss_fpga/bitinfo/hwruntime_io/raw"
#define BITINFO_NUM_ACCS_PATH "/dev/ompss_fpga/bitinfo/num_accs"

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
        buffer = malloc(sizeof(char) * max(STR_BUFFER_SIZE, strlen(accMapPath)));
        strcpy(buffer, accMapPath);
    }

    // 2nd -> /dev/ompss_fpga/bitinfo/xtasks
    if (buffer == NULL) {
        buffer = malloc(sizeof(char) * STR_BUFFER_SIZE);
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
    fprintf(stderr, "       Ensure that file '%s' exists and it has read permissions.\n", XTASKS_CONFIG_FILE_PATH);
    fprintf(stderr, "       Alternatively, you may force the configuration file path with");
    fprintf(stderr, " XTASKS_CONFIG_FILE environment variable.\n");
}

/*!
 * \brief Prints an error message in STDERR about configuration file not found
 */
void printErrorWrapperVersionFile()
{
    fprintf(stderr, "ERROR: xTasks Library cannot access the fpga wrapper version device file.\n");
    fprintf(stderr, "       Ensure that file '%s' exists and it has read permissions.\n", BITINFO_WRAPPER_PATH);
    fprintf(stderr, "       Alternatively, you may force the configuration file path with");
    fprintf(stderr, " XTASKS_CONFIG_FILE environment variable.\n");
}

/*!
 * \brief Checks whether a bitstream feature is present in the current fpga configuration or not
 * \return  BIT_FEATURE_NO_AVAIL if the feature is not available
 *          BIT_FEATURE_AVAIL if the feature is available
 *          BIT_FEATURE_SKIP if the check was skipped due to user requirements
 *          BIT_FEATURE_UNKNOWN if the check cannot be done or failed
 */
bit_feature_t checkbitstreamFeature(const char *featureName)
{
    const char *featuresCheck = getenv("XTASKS_FEATURES_CHECK");
    if (featuresCheck != NULL && featuresCheck[0] == '0') {
        return BIT_FEATURE_SKIP;
    } else if (featuresCheck != NULL && featuresCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_FEATURES_CHECK, must be 0 or 1. Ignoring it");
    }

    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    char buffer[strlen(BITINFO_FEATURES_PATH) + strlen(featureName) + 1];
    strcpy(buffer, BITINFO_FEATURES_PATH);
    strcat(buffer, "/");
    strcat(buffer, featureName);
    FILE *infoFile = fopen(buffer, "r");
    size_t nRead;

    if (infoFile != NULL) {
        nRead = fread(&buffer, sizeof(char), 1, infoFile);
        if (nRead != 1) {
            fprintf(stderr, "ERROR: xTasks could not read feature %s\n", featureName);
        }
        fclose(infoFile);
        available = buffer[0] == '1' ? BIT_FEATURE_AVAIL
                                     : (buffer[0] == '0' ? BIT_FEATURE_NO_AVAIL : BIT_FEATURE_UNKNOWN);
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
bit_compatibility_t checkbitstreamCompatibility()
{
    const char *compatCheck = getenv("XTASKS_COMPATIBILITY_CHECK");
    if (compatCheck != NULL && compatCheck[0] == '0') {
        return BIT_COMPAT_SKIP;
    } else if (compatCheck != NULL && compatCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_COMPATIBILITY_CHECK, must be 0 or 1. Ignoring it");
    }

    bit_compatibility_t compatible = BIT_COMPAT_UNKNOWN;
    FILE *infoFile = fopen(BITINFO_WRAPPER_PATH, "r");
    if (infoFile != NULL) {
        unsigned int wrapperVersion;
        fscanf(infoFile, "%u", &wrapperVersion);
        if (wrapperVersion < MIN_WRAPPER_VER || wrapperVersion > MAX_WRAPPER_VER) {
            printErrorBitstreamWrapperCompatibility(wrapperVersion);
            compatible = BIT_NO_COMPAT;
        } else {
            compatible = BIT_COMPAT;
        }
        fclose(infoFile);
    } else {
        printErrorWrapperVersionFile();
    }
    return compatible;
}

int getBitStreamHwrIOStruct(uint32_t hwruntimeIOStruct[BITINFO_HWRIO_STRUCT_WORDS])
{
    size_t nRead;
    FILE *infoFile = fopen(BITINFO_HWRIO_RAW_PATH, "r");
    int success = -1;

    if (infoFile != NULL) {
        nRead = fread(hwruntimeIOStruct, sizeof(uint32_t), BITINFO_HWRIO_STRUCT_WORDS, infoFile);
        success = nRead == BITINFO_HWRIO_STRUCT_WORDS ? 0 : -1;
        fclose(infoFile);
    }
    return success;
}

int getBitStreamNumAccs()
{
    FILE *infoFile = fopen(BITINFO_NUM_ACCS_PATH, "r");
    if (infoFile != NULL) {
        int numAccs = -1;
        fscanf(infoFile, "%d", &numAccs);
        fclose(infoFile);
        return numAccs;
    }
    return -1;
}

#endif /* __LIBXTASKS_PLATFORM_H__ */
