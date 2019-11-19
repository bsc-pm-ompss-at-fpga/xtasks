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

#ifndef __LIBXTASKS_FEATURES_H__
#define __LIBXTASKS_FEATURES_H__

#include "../util/common.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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
bit_feature_t checkbitstreamFeature(const char * featureName) {
    const char * featuresCheck = getenv("XTASKS_FEATURES_CHECK");
    if (featuresCheck != NULL && featuresCheck[0] == '0') {
        return BIT_FEATURE_SKIP;
    } else if (featuresCheck != NULL && featuresCheck[0] != '1') {
        PRINT_ERROR("Invalid value in XTASKS_FEATURES_CHECK, must be 0 or 1. Ignoring it");
    }

    bit_feature_t available = BIT_FEATURE_UNKNOWN;
    char buffer[strlen(BIT_INFO_FEATURES_PATH) + strlen(featureName) + 1];
    strcpy(buffer, BIT_INFO_FEATURES_PATH);
    strcat(buffer, "/");
    strcat(buffer, featureName);
    FILE * infoFile = fopen(buffer, "r");
    size_t nRead;

    if (infoFile != NULL) {
        nRead = fread(&buffer, sizeof(char), 1, infoFile);
        if (nRead != sizeof(char)) {
            fprintf(stderr, "ERROR: xTasks could not read feature %s\n", featureName);
        }
        fclose(infoFile);
        available = buffer[0] == '1' ? BIT_FEATURE_AVAIL :
            (buffer[0] == '0' ? BIT_FEATURE_NO_AVAIL : BIT_FEATURE_UNKNOWN);
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
        if (fscanf(infoFile, "%d", &wrapperVersion) != 1 || wrapperVersion < MIN_WRAPPER_VER || wrapperVersion > MAX_WRAPPER_VER) {
            //NOTE: If read value is not an integer, probably it is "?" which means that the
            //      bitstream is too old
            compatible = BIT_NO_COMPAT;
        } else {
            compatible = BIT_COMPAT;
        }
    }
    return compatible;
}

#endif /* __LIBXTASKS_FEATURES_H__ */
