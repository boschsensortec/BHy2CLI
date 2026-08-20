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
* @file       bhy_generic_features_defs.h
* @date       2026-08-06
* @version    v1.3.0
*
*/

#ifndef __BHY_GENERIC_FEATURES_DEFS_H__
#define __BHY_GENERIC_FEATURES_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy_defs.h"
#include <stdint.h>
#include <stdlib.h>

/*!
 *
 * @brief bhy wrist gesture detector control
 *
 */
typedef struct
{
    uint16_t min_flick_peak_y_thres; /* Minimum threshold for flick peak y-axis */
    uint16_t min_flick_peak_z_thres; /* Minimum threshold for flick peak z-axis */
    uint16_t gravity_bounds_x_pos; /* Maximum expected value of positive gravitational accel on x-axis */
    uint16_t gravity_bounds_x_neg; /* Maximum expected value of negative gravitational accel on x-axis */
    uint16_t gravity_bounds_y_neg; /* Maximum expected value of negative gravitational accel on y-axis */
    uint16_t gravity_bounds_z_neg; /* Maximum expected value of negative gravitational accel on z-axis */
    uint16_t flick_peak_decay_coeff; /* Exponential smoothing coefficient for adaptive peak threshold decay */
    uint16_t lp_mean_filter_coeff; /* Exponential smoothing coefficient for accel mean estimation */
    uint16_t max_duration_jiggle_peaks; /* Maximum duration between 2 peaks of jiggle in sample @50Hz */
    uint16_t device_pos; /* Device in left(0) or right (1) arm */
} BHY_PACKED bhy_generic_features_param_wrist_gesture_detector;

typedef int8_t (*bhy_generic_features_param_set_wrist_gesture_cfg_func)(const
                                                                        bhy_generic_features_param_wrist_gesture_detector
                                                                        *config, struct bhy_dev *dev);

typedef int8_t (*bhy_generic_features_param_get_wrist_gesture_cfg_func)(
    bhy_generic_features_param_wrist_gesture_detector *config, struct bhy_dev *dev);

typedef int8_t (*bhy_generic_features_param_reset_wrist_gesture_cfg_func)(struct bhy_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_GENERIC_FEATURES_DEFS_H__ */