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
 * @file    dinject.c
 * @brief   Source file for the data injection functions for the command line utility
 *
 */

#include <stdio.h>
#include "verbose.h"
#include "coines.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <sys/stat.h>
#include "dinject.h"

#define DI_PARSE_ERROR    -1
#define DI_PARSE_SUCCESS  1

uint32_t pos = 0;
uint8_t inject_log[MAX_SAMPLE_LENGTH] = { 0 };
uint16_t event_size = 0;
uint16_t len = 0;

/*******************************************************************************/

/**
* @brief Function to convert hexadecimal number to decimal number
* @param[in] hex    : Hexadecimal string
* @param[in] length : Length of hexadecimal string
* @return Decimal number
*/
int dinject_hex_to_dec(const char *hex, uint8_t length)
{
    int hex_val = 0, dec_val = 0, exponent = 0;

    for (int i = length - 1; i >= 0; --i)
    {
        switch (hex[i])
        {
            case 'A':
            case 'a':
                hex_val = 10;
                break;
            case 'B':
            case 'b':
                hex_val = 11;
                break;
            case 'C':
            case 'c':
                hex_val = 12;
                break;
            case 'D':
            case 'd':
                hex_val = 13;
                break;
            case 'E':
            case 'e':
                hex_val = 14;
                break;
            case 'F':
            case 'f':
                hex_val = 15;
                break;
            default:
                hex_val = hex[i] - 0x30;
                break;
        }

        dec_val = dec_val + hex_val * (int)(pow((double)16, (double)exponent));
        ++exponent;
    }

    return dec_val;
}

/**
* @brief Function to inject debug, timestamp and meta data
* @param[in] size : Size of data to inject
* @param[in] bhy  : Device reference
* @return API error codes
*/
int8_t inject_data(uint16_t size, struct bhy_dev *bhy)
{
    int8_t rslt;

    CALL_OUT_DYNAMIC_SENSOR_API(bhy_inject_data, "bhy_inject_data", rslt, inject_log, size, bhy);
    if (rslt == 0)
    {
        memset(inject_log, 0, sizeof(inject_log));
    }

    len = 0;

    return rslt;
}

/**
* @brief Function to process and parse file
* @param[in] event_id  : Event ID
* @param[in] dinject   : Pointer to Data Injection structure
*/
void parse_file(int event_id, const struct data_inject *dinject)
{
    inject_log[len] = (uint8_t) event_id;
    (void)dinject_parse_file(dinject->in_log_ptr, _8BIT_HEX_LEN, event_size - 1, &inject_log[len + 1]);
    len += event_size;
}

/**
* @brief Function to inject debug, timestamp and meta data
* @param[in] event_id  : Event ID
* @param[in] dinject   : Pointer to Data Injection structure
* @param[in] bhy       : Device reference
* @return API error codes
*/
int8_t dinject_timestamp_data(int event_id, const struct data_inject *dinject, struct bhy_dev *bhy)
{
    static uint8_t last_timestamp = 0;
    int8_t rslt = 0;

    switch (event_id)
    {
        case TIMESTAMP_LARGE_DELTA_WU_ID:
        case TIMESTAMP_LARGE_DELTA_NWU_ID:
            if (len != 0)
            {
                rslt = inject_data(len, bhy);
            }

            event_size = TIMESTAMP_LARGE_DELTA_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;

        case TIMESTAMP_SMALL_DELTA_WU_ID:
        case TIMESTAMP_SMALL_DELTA_NWU_ID:
            if (len != 0)
            {
                rslt = inject_data(len, bhy);
            }

            event_size = TIMESTAMP_SMALL_DELTA_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;

        case FULL_TIMESTAMP_WU_ID:
        case FULL_TIMESTAMP_NWU_ID:
            if (last_timestamp == 0)
            {
                event_size = FULL_TIMESTAMP_EVENT_SIZE;
            }
            else
            {
                event_size = len; /*! inject previous line of data */
            }

            parse_file(event_id, dinject);
            rslt = inject_data(event_size, bhy);
            last_timestamp = !last_timestamp;
            break;

        case META_EVENT_WU_ID:
        case META_EVENT_NWU_ID:
            event_size = META_EVENT_SIZE;
            parse_file(event_id, dinject);
            rslt = inject_data(event_size, bhy);
            break;

        case DEBUT_EVENT_ID:
            event_size = DEBUG_EVENT_SIZE;
            parse_file(event_id, dinject);
            rslt = inject_data(event_size, bhy);
            break;

        default:
            pos = (uint32_t)ftell(dinject->in_log_ptr);
            PRINT("\r\nError parsing 0x%X @ %u (0x%X)\r\n", event_id, pos, pos);

            rslt = DINJECT_FAILED;
    }

    return rslt;
}

/**
* @brief Function to parse the Input File
* @param[in] fp          : Input Field Log Pointer
* @param[in] hex_len     : Hex String Length
* @param[in] event_size  : Event Size
* @param[out] int_stream : Inject Stream
* @return API error codes
*/
int8_t dinject_parse_file(FILE *fp, size_t hex_len, size_t event_size_t, uint8_t int_stream[])
{
    char char_string[8];
    char single_char;
    int i = 0;
    size_t string_len = 0;
    size_t data_size = hex_len - 1;
    size_t num_elements = 1;

    while (event_size_t > 0)
    {
        single_char = (char)fgetc(fp);

        if ((single_char == ' ') || (single_char == '\r') || (single_char == '\n'))
        {
            continue;
        }
        else
        {
            /*lint -e583*/
            if (single_char != EOF)
            {
                char_string[0] = single_char;
                string_len = fread(&char_string[1], data_size, num_elements, fp);
                if (string_len != num_elements)
                {
                    ERROR("File Parsing failed \r\n");

                    return DI_PARSE_ERROR;
                }

                int_stream[i] = (uint8_t)dinject_hex_to_dec(char_string, (uint8_t)hex_len);
                memset(char_string, 0, sizeof(char_string));
                i++;
                event_size_t = event_size_t - 1;
            }
            else
            {
                break;
            }
        }
    }

    return DI_PARSE_SUCCESS;
}

/*******************************************************************************/

/**
* @brief Function to initialize Data Injection structure
* @param[in] input      : Input Field Log
* @param[in] dinject    : Data Injection structure
* @param[in] bhy        : Device reference
* @return API error codes
*/
int8_t dinject_init(char *input, struct data_inject *dinject, const struct bhy_dev *bhy)
{
    (void)bhy;

    struct stat st;
    int8_t rslt = BHY_OK;

    /*! Open the Field Log*/
#ifdef PC
    dinject->in_log_ptr = fopen(input, "rb");
#else
    dinject->in_log_ptr = fopen(input, "r");
#endif

    if (!dinject->in_log_ptr)
    {
        ERROR("Could not open file %s\r\n\r\n", input);

        return BHY_E_INVALID_PARAM;
    }
    else
    {
        PRINT("Opened Log File %s \r\n\r\n", input);
    }

    /*! Compute the File size */
    (void)stat((char*)input, &st);
    dinject->file_size = (uint32_t)st.st_size;
    PRINT("File Size : %ld\r\n", dinject->file_size);

    return rslt;
}

/**
* @brief Function to parse the injection data
* @param[in] event_id    : Event ID
* @param[in] dinject     : Data Injection structure
* @return
*/
static void dinject_parse_inject_data(int event_id, const struct data_inject *dinject)
{
    switch (event_id)
    {
        case ACCEL_INJECT_ID:
        case ACC_COR_INJECT_DRIVER_ID:
        case ACC_RAW_INJECT_DRIVER_ID:
        case ACC_PASSTH_INJECT_DRIVER_ID:
        case LINEACC_INJECT_DRIVER_ID:
            event_size = ACCEL_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case GYRO_INJECT_ID:
        case GYRO_COR_INJECT_DRIVER_ID:
        case GYRO_RAW_INJECT_DRIVER_ID:
        case GYRO_PASSTH_INJECT_DRIVER_ID:
            event_size = GYRO_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case MAG_INJECT_ID:
            event_size = MAG_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case GAME_ROTATION_INJECT_DRIVER_ID:
            event_size = QUAT_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case ORIENTATION_INJECT_DRIVER_ID:
            event_size = EULER_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case PRESSURE_INJECT_ID:
            event_size = PRESSURE_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case GAS_INJECT_ID:
            event_size = GAS_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        case IAQ_INJECT_ID:
            event_size = IAQ_INJECT_EVENT_SIZE;
            parse_file(event_id, dinject);
            break;
        default:
            break;
    }
}

/**
* @brief Function to inject the data
* @param[in] event_id    : Event ID
* @param[in] dinject     : Data Injection structure
* @param[in] bhy         : Device reference
* @return API error codes
*/
int8_t dinject_inject_data(int event_id, const struct data_inject *dinject, struct bhy_dev *bhy)
{
    int8_t rslt;

    if (!feof(dinject->in_log_ptr) && (event_id != EOF))
    {
        if (event_id < 100)
        {
            dinject_parse_inject_data(event_id, dinject);
        }
        else
        {
            rslt = dinject_timestamp_data(event_id, dinject, bhy);

            if (rslt == DINJECT_FAILED)
            {
                return DINJECT_FAILED;
            }
        }
    }
    else
    {
        PRINT("Data Injection Completed\r\n");

        return DINJECT_SUCCESSFUL;
    }

    return DINJECT_IN_PROGRESS;
}

/**
* @brief Function to uninitialize the Data Injection structure
* @param[in] dinject    : Data Injection structure
* @param[in] bhy        : Device reference
* @return API error codes
*/
int8_t dinject_deinit(struct data_inject *dinject, const struct bhy_dev *bhy)
{
    (void)bhy;

    pos = 0;
    memset(inject_log, 0, sizeof(inject_log));

    /*! Close the Field Log file, if open */
    PRINT("Closing the files\r\n");

    if (dinject->in_log_ptr)
    {
        fclose(dinject->in_log_ptr);
    }

    /*! Reset the Data Injection Structure */
    memset(dinject, 0, sizeof(struct data_inject));

    return BHY_OK;
}
