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
* @file       bhy_swim_defs.h
* @date       2025-03-28
* @version    v1.0.0
*
*/

#ifndef _BHY_SWIM_PARAM_DEFS_H_
#define _BHY_SWIM_PARAM_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include "bhy_event_data_defs.h"

#ifndef BHY_PACKED
#define BHY_PACKED                     __attribute__ ((__packed__))
#endif

/*! Sensor ID for Swim */
#define BHY_SENSOR_ID_SWIM             UINT8_C(114)

/*! Swim Parameter Page Base Address*/
#define BHY_SWIM_PAGE                  UINT16_C(0x0B)

/*! Swim Configuration pages */
#define BHY_SWIM_CONFIG_PARAM          (BHY_SWIM_PAGE << 8)
#define BHY_SWIM_VERSION               ((BHY_SWIM_PAGE << 8) | 1)
#define BHY_SWIM_ENABLE_LOGGING        ((BHY_SWIM_PAGE << 8) | 2)

/*! Swim configurations enable/ disable from parameter page */
#define BHY_SWIM_ENABLE_CONFIG         UINT8_C(1)
#define BHY_SWIM_DISABLE_CONFIG        UINT8_C(0)

/*! Device in left/right hand */
#define BHY_SWIM_DEVICE_ON_LEFT_HAND   UINT8_C(1)
#define BHY_SWIM_DEVICE_ON_RIGHT_HAND  UINT8_C(0)

/*! Pool length */
#define BHY_SWIM_POOL_LENGTH_25M       UINT8_C(25)
#define BHY_SWIM_POOL_LENGTH_50M       UINT8_C(50)

#define BHY_SWIM_DATA_LENGTH           UINT8_C(4)

/*!
 * bhy swim algo parameter.
 */
typedef struct  bhy_swim_param_config
{
    uint8_t update_swim_config;
    uint8_t dev_on_left_hand;
    uint8_t pool_length_integral;
    uint8_t pool_length_floating;
} BHY_PACKED bhy_swim_param_config_t;

/*!
 * callback data capturing structure
 */
typedef struct  bhy_swim_param_data_parse
{
    uint8_t sensor_id;
    uint8_t data_size;
    uint8_t *data_ptr;
    uint64_t *time_stamp;
} BHY_PACKED bhy_swim_param_data_parse_t;

/*! @brief structure definition to hold the version information of the software */
typedef struct bhy_swim_param_version
{
    uint8_t major;
    uint8_t minor;
    uint8_t platform;
    uint8_t reserved;
} BHY_PACKED bhy_swim_param_version_t;

/*!
 * swim output structure
 */
typedef struct
{
    uint16_t total_distance;
    uint16_t length_count;
    uint16_t lengths_freestyle;
    uint16_t lengths_breaststroke;
    uint16_t lengths_butterfly;
    uint16_t lengths_backstroke;
    uint16_t stroke_count;
} BHY_PACKED bhy_event_data_swim_output_t;

typedef void (*bhy_swim_param_algo_callback_func)(uint32_t seconds, uint32_t nanoseconds,
                                                  const struct bhy_swim_param_data_parse *callback_info);

typedef int8_t (*bhy_swim_param_get_config_func)(struct bhy_swim_param_config *conf, struct bhy_dev *dev);

typedef int8_t (*bhy_swim_param_set_config_func)(const struct bhy_swim_param_config *conf, struct bhy_dev *dev);

typedef int8_t (*bhy_swim_param_get_version_func)(struct bhy_swim_param_version *version, struct bhy_dev *dev);

typedef int8_t (*bhy_swim_param_set_logging_func)(uint8_t data, struct bhy_dev *dev);

typedef bhy_event_data_swim_output_t* (*bhy_get_swim_data_func)(void);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHY_SWIM_PARAM_DEFS_H_ */
