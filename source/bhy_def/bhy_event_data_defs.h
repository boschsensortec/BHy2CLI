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
* @file       bhy_event_data_defs.h
* @date       2025-03-28
* @version    v0.6.0
*
*/

#ifndef __BHY_EVENT_DATA_DEFS_H__
#define __BHY_EVENT_DATA_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdio.h>
#include <stdint.h>

#ifndef BHY_PACKED
#define BHY_PACKED  __attribute__ ((__packed__))
#endif

struct bhy_event_data_xyz
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct bhy_event_data_quaternion
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    uint16_t accuracy;
};

struct bhy_event_data_orientation
{
    int16_t heading;
    int16_t pitch;
    int16_t roll;
};

typedef struct
{
    uint8_t instance;

    /*!
     * A value of -1 means no new learning has occurred. If the value is >= 0,
     * then a new pattern has been learnt, and reading of this pattern may be
     * performed.
     */
    int8_t index;

    /*!
     * While learning a new pattern, this field counts from 0 to 100. When 100 is
     * reached a new pattern will be learnt. If learning is interrupted, this
     * progress will return to 0, and change reason will be set to indicate
     * why learning was interrupted.
     */
    uint8_t progress;

    /*!
     * | Value | Description                                                            |
     * |:-----:|:-----------------------------------------------------------------------|
     * | 0     | Learning is progressing.                                               |
     * | 1     | Learning was interrupted by a non-repetitive activity.                 |
     * | 2     | Learning was interrupted because no significant movement was detected. |
     */
    uint8_t change_reason;
} BHY_PACKED bhy_event_data_klio_learning_t;

typedef struct
{
    uint8_t instance;

    /*! The index of the recognized activity. 255 means no activity was
     * recognized. */
    uint8_t index;

    /*! The current repetition count of the recognized activity. */
    float count;
} BHY_PACKED bhy_event_data_klio_recognition_t;

/*!
 *
 * @brief bhy klio combined data structure
 *
 * When the algorithm generates a new data frame, it is sent in this structure.
 *
 */
typedef struct
{
    bhy_event_data_klio_learning_t learn;
    bhy_event_data_klio_recognition_t recognize;
} BHY_PACKED bhy_event_data_klio_t;

/*! Sensor Structure for Head Orientation Quaternion */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
} BHY_PACKED bhy_event_data_head_orientation_quat;

/*! Sensor Structure for Head Orientation Euler */
typedef struct
{
    int16_t heading;
    int16_t pitch;
    int16_t roll;
} BHY_PACKED bhy_event_data_head_orientation_eul;

/*!
 * Multi Tap Setting.
 */
typedef enum {
    BHY_NO_TAP,
    BHY_SINGLE_TAP,
    BHY_DOUBLE_TAP,
    BHY_DOUBLE_SINGLE_TAP,
    BHY_TRIPLE_TAP,
    BHY_TRIPLE_SINGLE_TAP,
    BHY_TRIPLE_DOUBLE_TAP,
    BHY_TRIPLE_DOUBLE_SINGLE_TAP
} bhy_event_data_multi_tap;

/*!
 * Multi Tap Output.
 */
static const char * const bhy_event_data_multi_tap_string_out[] = {
    [BHY_NO_TAP] = "NO_TAP", [BHY_SINGLE_TAP] = "SINGLE_TAP", [BHY_DOUBLE_TAP] = "DOUBLE_TAP",
    [BHY_DOUBLE_SINGLE_TAP] = "DOUBLE_SINGLE_TAP", [BHY_TRIPLE_TAP] = "TRIPLE_TAP",
    [BHY_TRIPLE_SINGLE_TAP] = "TRIPLE_SINGLE_TAP", [BHY_TRIPLE_DOUBLE_TAP] = "TRIPLE_DOUBLE_TAP",
    [BHY_TRIPLE_DOUBLE_SINGLE_TAP] = "TRIPLE_DOUBLE_SINGLE_TAP" };                                                /*lint -e528
                                                                                                           * */

enum bhy_event_data_wrist_gesture_activity {
    BHY_NO_GESTURE,
    BHY_WRIST_SHAKE_JIGGLE = 0x03,
    BHY_FLICK_IN,
    BHY_FLICK_OUT
};

typedef struct bhy_event_data_wrist_gesture_detect
{
    enum bhy_event_data_wrist_gesture_activity wrist_gesture;
} __attribute__ ((packed)) bhy_event_data_wrist_gesture_detect_t;

static const char * const bhy_event_data_wrist_gesture_detect_output[] = {
    [BHY_NO_GESTURE] = "NO_GESTURE", [BHY_WRIST_SHAKE_JIGGLE] = "WRIST_SHAKE_JIGGLE", [BHY_FLICK_IN] = "FLICK_IN",
    [BHY_FLICK_OUT] = "FLICK_OUT" }; /*lint -e528 */

typedef struct
{
    uint16_t iaq;
    uint16_t siaq;
    uint16_t voc;
    uint32_t co2;
    uint8_t iaq_accuracy;
    int16_t comp_temperature;
    uint16_t comp_humidity;
    uint32_t raw_gas;
} BHY_PACKED bhy_event_data_iaq_output_t;

typedef void (*bhy_event_data_parse_quaternion_func)(const uint8_t *data, struct bhy_event_data_quaternion *quaternion);

typedef void (*bhy_event_data_parse_orientation_func)(const uint8_t *data,
                                                      struct bhy_event_data_orientation *orientation);

typedef void (*bhy_event_data_parse_xyz_func)(const uint8_t *data, struct bhy_event_data_xyz *vector);

typedef void (*bhy_event_data_head_orientation_quat_parsing_func)(const uint8_t *payload,
                                                                  bhy_event_data_head_orientation_quat *data);

typedef void (*bhy_event_data_head_orientation_eul_parsing_func)(const uint8_t *payload,
                                                                 bhy_event_data_head_orientation_eul *data);

typedef int8_t (*bhy_event_data_multi_tap_parsing_func)(const uint8_t *data, uint8_t *output);

typedef int8_t (*bhy_event_data_wrist_gesture_detect_parsing_func)(const uint8_t *data,
                                                                   bhy_event_data_wrist_gesture_detect_t *output);

typedef void (*bhy_event_data_parse_air_quality_func)(const uint8_t *payload,
                                                      bhy_event_data_iaq_output_t *air_quality_data);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_EVENT_DATA_DEFS_H__ */
