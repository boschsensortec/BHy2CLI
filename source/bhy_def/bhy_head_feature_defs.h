/**
* Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bhy_head_feature_defs.h
* @date       2026-08-06
* @version    v1.3.0
*
*/

#ifndef _BHY_HEAD_FEATURE_PARAM_DEFS_H_
#define _BHY_HEAD_FEATURE_PARAM_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>

/*! Virtual Sensor Macros */
#define BHY_SENSOR_ID_HEAD_ORI_MIS_ALG                         UINT8_C(120)                         /*Head Orientation
                                                                                                     * Misalignment*/
#define BHY_SENSOR_ID_IMU_HEAD_ORI_Q                           UINT8_C(121)                         /*IMU Head
                                                                                                     * Orientation
                                                                                                     * Quaternion*/
#define BHY_SENSOR_ID_NDOF_HEAD_ORI_Q                          UINT8_C(122)                         /*NDOF Head
                                                                                                     * Orientation
                                                                                                     * Quaternion*/
#define BHY_SENSOR_ID_IMU_HEAD_ORI_E                           UINT8_C(123)                         /*IMU Head
                                                                                                     * Orientation
                                                                                                     * Euler*/
#define BHY_SENSOR_ID_NDOF_HEAD_ORI_E                          UINT8_C(124)                         /*NDOF Head
                                                                                                     * Orientation
                                                                                                     * Euler*/

#define BHY_HEAD_FEATURE_PARAM_QUAT_INITIAL_HEAD_CORR_DISABLE  UINT8_C(0)
#define BHY_HEAD_FEATURE_PARAM_QUAT_INITIAL_HEAD_CORR_ENABLE   UINT8_C(1)
#define BHY_HEAD_FEATURE_PARAM_NDOF_INITIAL_HEAD_CORR_DISABLE  UINT8_C(0)
#define BHY_HEAD_FEATURE_PARAM_NDOF_INITIAL_HEAD_CORR_ENABLE   UINT8_C(1)

/*! Structure for Head Misalignment Configuration */
typedef struct
{
    uint8_t still_phase_max_dur;
    uint8_t still_phase_min_dur;
    uint8_t still_phase_max_samples;
    int32_t acc_diff_threshold;
    uint8_t reserved;
} BHY_PACKED bhy_head_feature_param_misalignment_config;

/*! Structure for Head Orientation /Head Misalignment version */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t reserved;
} BHY_PACKED bhy_head_feature_param_ver;

/*! Structure for Head Misalignment Quaternion Correction */
typedef struct
{
    union bhy_float_conv quaternion_x;
    union bhy_float_conv quaternion_y;
    union bhy_float_conv quaternion_z;
    union bhy_float_conv quaternion_w;
    union bhy_float_conv accuracy;
} BHY_PACKED bhy_head_feature_param_misalignment_quat_corr;

/*! Structure for Head Misalignment Mode and Vector X value */
typedef struct
{
    uint8_t mode;
    union bhy_float_conv vector_x_0;
    union bhy_float_conv vector_x_1;
    union bhy_float_conv vector_x_2;
    uint8_t reserved[3];
} BHY_PACKED bhy_head_misalignment_mode_vector_x;

/*! Structure for Head Gesture Algorithm Version */
typedef struct
{
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t major_bug_fix;
    uint8_t reserved;
} BHY_PACKED bhy_head_gesture_algo_version_t;

/*! Structure for Head Gesture Time Duration of One Gesture */
typedef struct
{
    union bhy_float_conv time_duration_of_one_gesture;
} BHY_PACKED bhy_hgd_time_duration_of_one_gesture_t;

/*! Structure for Head Gesture Time Duration of One Gesture Tilt */
typedef struct
{
    union bhy_float_conv time_duration_of_one_gesture_tilt;
} BHY_PACKED bhy_hgd_time_duration_of_one_gesture_tilt_t;

typedef int8_t (*bhy_head_feature_param_trigger_hmc_calibration_func)(struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_hmc_configuration_func)(const bhy_head_feature_param_misalignment_config *
                                                                    config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_hmc_configuration_func)(bhy_head_feature_param_misalignment_config *config,
                                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_default_hmc_cfg_func)(struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_hmc_version_func)(bhy_head_feature_param_ver *hmc_version,
                                                              struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_hmc_quat_cal_cor_cfg_func)(const
                                                                       bhy_head_feature_param_misalignment_quat_corr*
                                                                       config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_hmc_quat_cal_cor_cfg_func)(bhy_head_feature_param_misalignment_quat_corr *
                                                                       config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_hmc_mode_vector_x_func)(const bhy_head_misalignment_mode_vector_x *config,
                                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_hmc_mode_vector_x_func)(bhy_head_misalignment_mode_vector_x *config,
                                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_quat_init_head_corr_func)(const uint8_t* config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_quat_init_head_corr_func)(uint8_t *buffer, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_ho_version_func)(bhy_head_feature_param_ver *ho_version,
                                                             struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_set_ndof_init_head_corr_func)(const uint8_t *config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_get_ndof_init_head_corr_func)(uint8_t *config, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_get_algo_version_func)(bhy_head_gesture_algo_version_t *ho_version,
                                                                   struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_get_time_duration_of_one_gesture_func)(
    bhy_hgd_time_duration_of_one_gesture_t *time_dur, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_set_time_duration_of_one_gesture_func)(const
                                                                                   bhy_hgd_time_duration_of_one_gesture_t
                                                                                   *time_dur, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_set_to_default_func)(uint8_t value, struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_get_thres_primary_motion_angular_rate_func)(uint8_t *thres,
                                                                                        struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_set_thres_primary_motion_angular_rate_func)(uint8_t thres,
                                                                                        struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_get_time_duration_of_one_gesture_tilt_func)(union bhy_float_conv *
                                                                                        time_dur_tilt,
                                                                                        struct bhy_dev *dev);

typedef int8_t (*bhy_head_feature_param_hgd_set_time_duration_of_one_gesture_tilt_func)(union bhy_float_conv *
                                                                                        time_dur_tilt,
                                                                                        struct bhy_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHY_HEAD_FEATURE_PARAM_DEFS_H_ */
