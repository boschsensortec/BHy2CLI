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
* @file       bhy_parse.h
* @date       2025-08-20
* @version    v1.0.0
*
*/
#ifndef __BHY_PARSE_H__
#define __BHY_PARSE_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdbool.h>
#include "stdint.h"
#include "bhy_logbin_defs.h"

#define PARSE_FLAG_NONE              UINT8_C(0x00)
#define PARSE_FLAG_STREAM            UINT8_C(0x01)
#define PARSE_FLAG_LOG               UINT8_C(0x02)
#define PARSE_FLAG_HEXSTREAM         UINT8_C(0x04)

#define PARSE_SET_FLAG(var, flag)    (var | flag)
#define PARSE_CLEAR_FLAG(var, flag)  (var & ~flag)

#define BHY2CLI_MAX_STRING_LENGTH    UINT16_C(32)

struct bhy_parse_sensor_details
{
    uint8_t id;
    uint8_t accuracy;
    float scaling_factor;
    uint8_t parse_flag;
};

struct bhy_parse_ref
{
    struct bhy_parse_sensor_details sensor[BHY_MAX_SIMUL_SENSORS];
    struct bhy_dev *bhy;
    struct bhy_logbin_dev logdev;
};

typedef struct bhy_parse_sensor_details *(*bhy_parse_get_sensor_details_func)(uint8_t id, struct bhy_parse_ref *ref);

typedef struct bhy_parse_sensor_details *(*bhy_parse_add_sensor_details_func)(uint8_t id, struct bhy_parse_ref *ref);

typedef void (*bhy_parse_meta_event_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_3axis_s16_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_euler_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_quaternion_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_s16_as_float_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_scalar_u32_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_scalar_event_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_activity_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_u16_as_float_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_u24_as_float_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_proximity_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_scalar_u8_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_generic_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_device_ori_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_gps_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_debug_message_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/*typedef void (*bhy_parse_acc_gyro_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref); */

typedef void (*bhy_parse_multitap_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_wrist_gesture_detect_func)(const struct bhy_fifo_parse_data_info *callback_info,
                                                    void *callback_ref);

typedef void (*bhy_parse_step_counter_data_func)(const struct bhy_fifo_parse_data_info *callback_info,
                                                 void *callback_ref);
typedef void (*bhy_parse_wrist_wear_wakeup_data_func)(const struct bhy_fifo_parse_data_info *callback_info,
                                                      void *callback_ref);

typedef void (*bhy_parse_air_quality_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_hmc_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_oc_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_ec_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_add_accuracy_to_sensor_data_func)(const uint8_t *data_ptr, uint8_t *data_buf, uint8_t data_size,
                                                     uint8_t accuracy);

typedef void (*bhy_set_downsampling_flag_func)(uint8_t sen_id, bool enable);

typedef bool (*bhy_get_downsampling_flag_func)(uint8_t sen_id);

typedef void (*bhy_set_downsampling_odr_func)(uint8_t sen_id, int16_t odr);

typedef void (*bhy_parse_temperature_celsius_func)(const uint8_t *data, bhy_float *temperature);

typedef void (*bhy_parse_humidity_func)(const uint8_t *data, bhy_float *humidity);

typedef void (*bhy_parse_pressure_func)(const uint8_t *data, bhy_float *pressure);

typedef void (*bhy_parse_altitude_func)(const uint8_t *data, bhy_float *altitude);

typedef void (*bhy_parse_klio_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_klio_log_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

typedef void (*bhy_parse_swim_func)(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_PARSE_H__ */
