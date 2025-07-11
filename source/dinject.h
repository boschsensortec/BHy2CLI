/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    dinject.h
 * @brief   Header file for the data injection functions for the command line utility
 *
 */

#ifndef DINJECT_H_
#define DINJECT_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdbool.h>

#include "dinject_defs.h"
#include "bhy_defs.h"

#define DINJECT_FAILED       INT8_C(-1)
#define DINJECT_SUCCESSFUL   INT8_C(1)
#define DINJECT_IN_PROGRESS  INT8_C(0)

#define MAX_SAMPLE_LENGTH    256

/*! Data Injection Structure */
struct data_inject
{
    FILE *in_log_ptr;
    uint32_t file_size;
    uint16_t event_size;
};

/**
* @brief Function to initialize Data Injection structure
* @param[in] input      : Input Field Log
* @param[in] dinject    : Data Injection structure
* @param[in] bhy        : Device reference
* @return API error codes
*/
int8_t dinject_init(char *input, struct data_inject *dinject, const struct bhy_dev *bhy);

/**
* @brief Function to inject the data
* @param[in] event_id    : Event ID
* @param[in] dinject     : Data Injection structure
* @param[in] bhy         : Device reference
* @return API error codes
*/
int8_t dinject_inject_data(int event_id, const struct data_inject *dinject, struct bhy_dev *bhy);

/**
* @brief Function to uninitialize the Data Injection structure
* @param[in] dinject    : Data Injection structure
* @param[in] bhy        : Device reference
* @return API error codes
*/
int8_t dinject_deinit(struct data_inject *dinject, const struct bhy_dev *bhy);

/**
* @brief Function to parse the Input File
* @param[in] fp          : Input Field Log Pointer
* @param[in] hex_len     : Hex String Length
* @param[in] event_size  : Event Size
* @param[out] int_stream : Inject Stream
* @return API error codes
*/
int8_t dinject_parse_file(FILE *fp, size_t hex_len, size_t event_size, uint8_t int_stream[]);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* DINJECT_H_ */
