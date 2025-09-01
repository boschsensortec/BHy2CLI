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
* @file       bhy_defs.h
* @date       2025-08-20
* @version    v1.0.0
*
*/

#ifndef __BHY_DEFS_H__
#define __BHY_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
"C" {
#endif /*__cplusplus */

#ifdef __KERNEL__
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#else
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "bhy_common_defs.h"
#include "bhy_activity_defs.h"
#include "bhy_bsec_defs.h"
#include "bhy_bsx_defs.h"
#include "bhy_event_data_defs.h"
#include "bhy_head_orientation_defs.h"
#include "bhy_klio_defs.h"
#include "bhy_logbin_defs.h"
#include "bhy_multi_tap_defs.h"
#include "bhy_phy_sensor_ctrl_defs.h"
#include "bhy_swim_defs.h"
#include "bhy_system_param_defs.h"
#include "bhy_virtual_sensor_conf_defs.h"
#include "bhy_virtual_sensor_info_defs.h"
#include "bhy_parse_defs.h"
#endif /*~ __KERNEL__ */

typedef int8_t (*bhy_init_func)(enum bhy_intf intf, bhy_read_fptr_t read, bhy_write_fptr_t write,
                                bhy_delay_us_fptr_t delay_us, uint32_t read_write_len, void *intf_ptr,
                                struct bhy_dev *dev);

typedef int8_t (*bhy_get_chip_id_func)(uint8_t *chip_id, struct bhy_dev *dev);

typedef int8_t (*bhy_get_and_process_fifo_func)(uint8_t *work_buffer, uint32_t buffer_size, struct bhy_dev *dev);

typedef int8_t (*bhy_register_fifo_parse_callback_func)(uint8_t sensor_id, bhy_fifo_parse_callback_t callback,
                                                        void *callback_ref, struct bhy_dev *dev);

typedef int8_t (*bhy_get_regs_func)(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, struct bhy_dev *dev);

typedef int8_t (*bhy_set_regs_func)(uint8_t reg_addr, const uint8_t *reg_data, uint16_t length, struct bhy_dev *dev);

typedef int8_t (*bhy_get_product_id_func)(uint8_t *product_id, struct bhy_dev *dev);

typedef int8_t (*bhy_get_revision_id_func)(uint8_t *revision_id, struct bhy_dev *dev);

typedef int8_t (*bhy_get_rom_version_func)(uint16_t *rom_version, struct bhy_dev *dev);

typedef int8_t (*bhy_get_kernel_version_func)(uint16_t *kernel_version, struct bhy_dev *dev);

typedef int8_t (*bhy_get_user_version_func)(uint16_t *user_version, struct bhy_dev *dev);

typedef int8_t (*bhy_get_boot_status_func)(uint8_t *boot_status, struct bhy_dev *dev);

typedef int8_t (*bhy_get_host_status_func)(uint8_t *host_status, struct bhy_dev *dev);

typedef int8_t (*bhy_get_feature_status_func)(uint8_t *feat_status, struct bhy_dev *dev);

typedef int8_t (*bhy_set_virt_sensor_range_func)(uint8_t sensor_id, uint16_t range, struct bhy_dev *dev);

typedef int8_t (*bhy_get_fifo_wmark_wkup_func)(uint32_t *watermark, struct bhy_dev *dev);

typedef int8_t (*bhy_get_fifo_wmark_nonwkup_func)(uint32_t *watermark, struct bhy_dev *dev);

typedef int8_t (*bhy_flush_fifo_func)(uint8_t sensor_id, struct bhy_dev *dev);

typedef int8_t (*bhy_set_fifo_format_ctrl_func)(uint8_t param, struct bhy_dev *dev);

typedef int8_t (*bhy_upload_firmware_to_ram_func)(const uint8_t *firmware, uint32_t length, struct bhy_dev *dev);

typedef int8_t (*bhy_upload_firmware_to_ram_partly_func)(const uint8_t *firmware, uint32_t total_size, uint32_t cur_pos,
                                                         uint32_t packet_len, struct bhy_dev *dev);

typedef int8_t (*bhy_boot_from_ram_func)(struct bhy_dev *dev);

typedef int8_t (*bhy_set_host_interrupt_ctrl_func)(uint8_t hintr_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_get_host_interrupt_ctrl_func)(uint8_t *hintr_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_get_interrupt_status_func)(uint8_t *int_status, struct bhy_dev *dev);

typedef int8_t (*bhy_set_host_intf_ctrl_func)(uint8_t hintf_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_get_host_intf_ctrl_func)(uint8_t *hintf_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_set_timestamp_event_req_func)(uint8_t ts_ev_req, struct bhy_dev *dev);

typedef int8_t (*bhy_get_hw_timestamp_ns_func)(uint64_t *timestamp_ns, struct bhy_dev *dev);

typedef int8_t (*bhy_set_host_ctrl_func)(uint8_t host_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_get_host_ctrl_func)(uint8_t *host_ctrl, struct bhy_dev *dev);

typedef int8_t (*bhy_soft_reset_func)(struct bhy_dev *dev);

typedef int8_t (*bhy_perform_self_test_func)(uint8_t phys_sensor_id, struct bhy_self_test_resp *self_test_resp,
                                             struct bhy_dev *dev);

typedef int8_t (*bhy_perform_foc_func)(uint8_t phys_sensor_id, struct bhy_foc_resp *foc_resp, struct bhy_dev *dev);

typedef int8_t (*bhy_get_orientation_matrix_func)(uint8_t phys_sensor_id,
                                                  struct bhy_system_param_orient_matrix *orient_matrix,
                                                  struct bhy_dev *dev);

typedef int8_t (*bhy_get_post_mortem_data_func)(uint8_t *post_mortem, uint32_t buffer_len, uint32_t *actual_len,
                                                struct bhy_dev *dev);

typedef int8_t (*bhy_deregister_fifo_parse_callback_func)(uint8_t sensor_id, struct bhy_dev *dev);

typedef int8_t (*bhy_update_virtual_sensor_list_func)(struct bhy_dev *dev);

typedef int8_t (*bhy_get_sensor_info_func)(uint8_t sensor_id, struct bhy_virtual_sensor_info_param_info *info,
                                           struct bhy_dev *dev);

typedef int8_t (*bhy_set_parameter_func)(uint16_t param, const uint8_t *payload, uint32_t length, struct bhy_dev *dev);

typedef int8_t (*bhy_get_parameter_func)(uint16_t param, uint8_t *payload, uint32_t payload_len, uint32_t *actual_len,
                                         struct bhy_dev *dev);

typedef int8_t (*bhy_get_error_value_func)(uint8_t *error_value, struct bhy_dev *dev);

typedef int8_t (*bhy_soft_passthrough_transfer_func)(union bhy_soft_passthrough_conf *conf, uint8_t reg_addr,
                                                     uint8_t length, uint8_t *reg_data, struct bhy_dev *dev);

typedef uint8_t (*bhy_is_sensor_available_func)(uint8_t sensor_id, const struct bhy_dev *dev);

typedef uint8_t (*bhy_is_physical_sensor_available_func)(uint8_t sensor_id, const struct bhy_dev *dev);

typedef int8_t (*bhy_get_variant_id_func)(uint32_t *variant_id, struct bhy_dev *dev);

typedef int8_t (*bhy_inject_data_func)(const uint8_t *payload, uint32_t payload_len, struct bhy_dev *dev);

typedef int8_t (*bhy_set_data_injection_mode_func)(enum bhy_data_inj_mode mode, struct bhy_dev *dev);

typedef int8_t (*bhy_clear_fifo_func)(uint8_t flush_cfg, struct bhy_dev *dev);

typedef int8_t (*bhy_read_status_func)(uint16_t *status_code, uint8_t *status_buff, uint32_t status_len,
                                       uint32_t *actual_len, struct bhy_dev *dev);

extern bhy_init_func bhy_init;
extern bhy_get_chip_id_func bhy_get_chip_id;
extern bhy_get_and_process_fifo_func bhy_get_and_process_fifo;
extern bhy_register_fifo_parse_callback_func bhy_register_fifo_parse_callback;
extern bhy_set_host_intf_ctrl_func bhy_set_host_intf_ctrl;
extern bhy_get_host_intf_ctrl_func bhy_get_host_intf_ctrl;
extern bhy_activity_param_set_hearable_config_func bhy_activity_param_set_hearable_config;
extern bhy_activity_param_get_hearable_config_func bhy_activity_param_get_hearable_config;
extern bhy_activity_param_set_wearable_config_func bhy_activity_param_set_wearable_config;
extern bhy_activity_param_get_wearable_config_func bhy_activity_param_get_wearable_config;
extern bhy_bsec_param_set_algo_state_func bhy_bsec_param_set_algo_state;
extern bhy_bsec_param_get_algo_state_func bhy_bsec_param_get_algo_state;
extern bhy_bsec_param_set_temp_offset_func bhy_bsec_param_set_temp_offset;
extern bhy_bsec_param_get_temp_offset_func bhy_bsec_param_get_temp_offset;
extern bhy_bsec_param_set_sample_rate_func bhy_bsec_param_set_sample_rate;
extern bhy_bsec_param_get_sample_rate_func bhy_bsec_param_get_sample_rate;
extern bhy_bsx_algo_param_get_bsx_states_func bhy_bsx_algo_param_get_bsx_states;
extern bhy_bsx_algo_param_set_bsx_states_func bhy_bsx_algo_param_set_bsx_states;
extern bhy_bsx_algo_param_get_bsx_version_func bhy_bsx_algo_param_get_bsx_version;
extern bhy_event_data_parse_quaternion_func bhy_event_data_parse_quaternion;
extern bhy_event_data_parse_orientation_func bhy_event_data_parse_orientation;
extern bhy_event_data_parse_xyz_func bhy_event_data_parse_xyz;
extern bhy_event_data_head_orientation_quat_parsing_func bhy_event_data_head_orientation_quat_parsing;
extern bhy_event_data_head_orientation_eul_parsing_func bhy_event_data_head_orientation_eul_parsing;
extern bhy_event_data_multi_tap_parsing_func bhy_event_data_multi_tap_parsing;
extern bhy_event_data_wrist_gesture_detect_parsing_func bhy_event_data_wrist_gesture_detect_parsing;
extern bhy_event_data_parse_air_quality_func bhy_event_data_parse_air_quality;
extern bhy_head_orientation_param_trigger_hmc_calibration_func bhy_head_orientation_param_trigger_hmc_calibration;
extern bhy_head_orientation_param_set_hmc_configuration_func bhy_head_orientation_param_set_hmc_configuration;
extern bhy_head_orientation_param_get_hmc_configuration_func bhy_head_orientation_param_get_hmc_configuration;
extern bhy_head_orientation_param_set_default_hmc_cfg_func bhy_head_orientation_param_set_default_hmc_cfg;
extern bhy_head_orientation_param_get_hmc_version_func bhy_head_orientation_param_get_hmc_version;
extern bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg_func bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg;
extern bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg_func bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg;
extern bhy_head_orientation_param_set_hmc_mode_vector_x_func bhy_head_orientation_param_set_hmc_mode_vector_x;
extern bhy_head_orientation_param_get_hmc_mode_vector_x_func bhy_head_orientation_param_get_hmc_mode_vector_x;
extern bhy_head_orientation_param_set_quat_init_head_corr_func bhy_head_orientation_param_set_quat_init_head_corr;
extern bhy_head_orientation_param_get_quat_init_head_corr_func bhy_head_orientation_param_get_quat_init_head_corr;
extern bhy_head_orientation_param_get_ho_version_func bhy_head_orientation_param_get_ho_version;
extern bhy_head_orientation_param_set_eul_init_head_corr_func bhy_head_orientation_param_set_eul_init_head_corr;
extern bhy_head_orientation_param_get_eul_init_head_corr_func bhy_head_orientation_param_get_eul_init_head_corr;
extern bhy_logbin_start_meta_func bhy_logbin_start_meta;
extern bhy_logbin_add_meta_func bhy_logbin_add_meta;
extern bhy_logbin_end_meta_func bhy_logbin_end_meta;
extern bhy_logbin_add_data_func bhy_logbin_add_data;
extern bhy_multi_tap_param_set_config_func bhy_multi_tap_param_set_config;
extern bhy_multi_tap_param_get_config_func bhy_multi_tap_param_get_config;
extern bhy_multi_tap_param_detector_set_config_func bhy_multi_tap_param_detector_set_config;
extern bhy_multi_tap_param_detector_get_config_func bhy_multi_tap_param_detector_get_config;
extern bhy_phy_sensor_ctrl_param_accel_set_foc_calibration_func bhy_phy_sensor_ctrl_param_accel_set_foc_calibration;
extern bhy_phy_sensor_ctrl_param_accel_get_foc_calibration_func bhy_phy_sensor_ctrl_param_accel_get_foc_calibration;
extern bhy_phy_sensor_ctrl_param_accel_set_power_mode_func bhy_phy_sensor_ctrl_param_accel_set_power_mode;
extern bhy_phy_sensor_ctrl_param_accel_get_power_mode_func bhy_phy_sensor_ctrl_param_accel_get_power_mode;
extern bhy_phy_sensor_ctrl_param_accel_set_axis_remapping_func bhy_phy_sensor_ctrl_param_accel_set_axis_remapping;
extern bhy_phy_sensor_ctrl_param_accel_get_axis_remapping_func bhy_phy_sensor_ctrl_param_accel_get_axis_remapping;
extern bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing_func bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing;
extern bhy_phy_sensor_ctrl_param_accel_get_nvm_status_func bhy_phy_sensor_ctrl_param_accel_get_nvm_status;
extern bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration_func bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration;
extern bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration_func bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration;
extern bhy_phy_sensor_ctrl_param_gyro_set_ois_config_func bhy_phy_sensor_ctrl_param_gyro_set_ois_config;
extern bhy_phy_sensor_ctrl_param_gyro_get_ois_config_func bhy_phy_sensor_ctrl_param_gyro_get_ois_config;
extern bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg_func bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg;
extern bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg_func bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg;
extern bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim_func bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim;
extern bhy_phy_sensor_ctrl_param_gyro_get_crt_data_func bhy_phy_sensor_ctrl_param_gyro_get_crt_data;
extern bhy_phy_sensor_ctrl_param_set_gyro_data_func bhy_phy_sensor_ctrl_param_set_gyro_data;
extern bhy_phy_sensor_ctrl_param_gyro_set_power_mode_func bhy_phy_sensor_ctrl_param_gyro_set_power_mode;
extern bhy_phy_sensor_ctrl_param_gyro_get_power_mode_func bhy_phy_sensor_ctrl_param_gyro_get_power_mode;
extern bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg_func bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg;
extern bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg_func bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg;
extern bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing_func bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing;
extern bhy_phy_sensor_ctrl_param_gyro_get_nvm_status_func bhy_phy_sensor_ctrl_param_gyro_get_nvm_status;
extern bhy_phy_sensor_ctrl_param_magnet_set_power_mode_func bhy_phy_sensor_ctrl_param_magnet_set_power_mode;
extern bhy_phy_sensor_ctrl_param_magnet_get_power_mode_func bhy_phy_sensor_ctrl_param_magnet_get_power_mode;
extern bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg_func bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg;
extern bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg_func bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg;
extern bhy_phy_sensor_ctrl_param_set_any_motion_config_func bhy_phy_sensor_ctrl_param_set_any_motion_config;
extern bhy_phy_sensor_ctrl_param_get_any_motion_config_func bhy_phy_sensor_ctrl_param_get_any_motion_config;
extern bhy_phy_sensor_ctrl_param_set_no_motion_config_func bhy_phy_sensor_ctrl_param_set_no_motion_config;
extern bhy_phy_sensor_ctrl_param_get_no_motion_config_func bhy_phy_sensor_ctrl_param_get_no_motion_config;
extern bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg_func bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg;
extern bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg_func bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg;
extern bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg_func bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg;
extern bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg_func bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg;
extern bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg_func bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg;
extern bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg_func bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg;
extern bhy_phy_sensor_ctrl_param_set_step_counter_config_func bhy_phy_sensor_ctrl_param_set_step_counter_config;
extern bhy_phy_sensor_ctrl_param_get_step_counter_config_func bhy_phy_sensor_ctrl_param_get_step_counter_config;
extern bhy_system_param_set_meta_event_control_func bhy_system_param_set_meta_event_control;
extern bhy_system_param_get_meta_event_control_func bhy_system_param_get_meta_event_control;
extern bhy_system_param_set_wakeup_fifo_control_func bhy_system_param_set_wakeup_fifo_control;
extern bhy_system_param_set_nonwakeup_fifo_control_func bhy_system_param_set_nonwakeup_fifo_control;
extern bhy_system_param_get_fifo_control_func bhy_system_param_get_fifo_control;
extern bhy_system_param_get_firmware_version_func bhy_system_param_get_firmware_version;
extern bhy_system_param_get_timestamps_func bhy_system_param_get_timestamps;
extern bhy_system_param_get_virtual_sensor_present_func bhy_system_param_get_virtual_sensor_present;
extern bhy_system_param_get_physical_sensor_present_func bhy_system_param_get_physical_sensor_present;
extern bhy_system_param_get_physical_sensor_info_func bhy_system_param_get_physical_sensor_info;
extern bhy_system_param_set_physical_sensor_info_func bhy_system_param_set_physical_sensor_info;
extern bhy_virtual_sensor_conf_param_set_cfg_func bhy_virtual_sensor_conf_param_set_cfg;
extern bhy_virtual_sensor_conf_param_get_cfg_func bhy_virtual_sensor_conf_param_get_cfg;
extern bhy_virtual_sensor_info_param_get_info_func bhy_virtual_sensor_info_param_get_info;
extern bhy_get_regs_func bhy_get_regs;
extern bhy_set_regs_func bhy_set_regs;
extern bhy_get_product_id_func bhy_get_product_id;
extern bhy_get_revision_id_func bhy_get_revision_id;
extern bhy_get_rom_version_func bhy_get_rom_version;
extern bhy_get_kernel_version_func bhy_get_kernel_version;
extern bhy_get_user_version_func bhy_get_user_version;
extern bhy_get_boot_status_func bhy_get_boot_status;
extern bhy_get_host_status_func bhy_get_host_status;
extern bhy_get_feature_status_func bhy_get_feature_status;
extern bhy_set_virt_sensor_range_func bhy_set_virt_sensor_range;

/*
 * extern bhy_get_fifo_wmark_wkup_func bhy_get_fifo_wmark_wkup;
 * extern bhy_get_fifo_wmark_nonwkup_func bhy_get_fifo_wmark_nonwkup;
 */
extern bhy_flush_fifo_func bhy_flush_fifo;
extern bhy_set_fifo_format_ctrl_func bhy_set_fifo_format_ctrl;
extern bhy_upload_firmware_to_ram_func bhy_upload_firmware_to_ram;
extern bhy_upload_firmware_to_ram_partly_func bhy_upload_firmware_to_ram_partly;
extern bhy_boot_from_ram_func bhy_boot_from_ram;
extern bhy_set_host_interrupt_ctrl_func bhy_set_host_interrupt_ctrl;
extern bhy_get_host_interrupt_ctrl_func bhy_get_host_interrupt_ctrl;
extern bhy_get_interrupt_status_func bhy_get_interrupt_status;
extern bhy_set_timestamp_event_req_func bhy_set_timestamp_event_req;
extern bhy_get_hw_timestamp_ns_func bhy_get_hw_timestamp_ns;
extern bhy_set_host_ctrl_func bhy_set_host_ctrl;
extern bhy_get_host_ctrl_func bhy_get_host_ctrl;
extern bhy_soft_reset_func bhy_soft_reset;
extern bhy_perform_self_test_func bhy_perform_self_test;
extern bhy_perform_foc_func bhy_perform_foc;
extern bhy_get_orientation_matrix_func bhy_get_orientation_matrix;
extern bhy_get_post_mortem_data_func bhy_get_post_mortem_data;
extern bhy_deregister_fifo_parse_callback_func bhy_deregister_fifo_parse_callback;
extern bhy_update_virtual_sensor_list_func bhy_update_virtual_sensor_list;
extern bhy_get_sensor_info_func bhy_get_sensor_info;
extern bhy_set_parameter_func bhy_set_parameter;
extern bhy_get_parameter_func bhy_get_parameter;
extern bhy_get_error_value_func bhy_get_error_value;
extern bhy_soft_passthrough_transfer_func bhy_soft_passthrough_transfer;
extern bhy_is_sensor_available_func bhy_is_sensor_available;
extern bhy_is_physical_sensor_available_func bhy_is_physical_sensor_available;
extern bhy_get_variant_id_func bhy_get_variant_id;
extern bhy_inject_data_func bhy_inject_data;
extern bhy_set_data_injection_mode_func bhy_set_data_injection_mode;
extern bhy_clear_fifo_func bhy_clear_fifo;
extern bhy_read_status_func bhy_read_status;
extern bhy_parse_get_sensor_details_func bhy_parse_get_sensor_details;
extern bhy_parse_add_sensor_details_func bhy_parse_add_sensor_details;
extern bhy_parse_meta_event_func bhy_parse_meta_event;
extern bhy_parse_3axis_s16_func bhy_parse_3axis_s16;
extern bhy_parse_euler_func bhy_parse_euler;
extern bhy_parse_quaternion_func bhy_parse_quaternion;
extern bhy_parse_s16_as_float_func bhy_parse_s16_as_float;
extern bhy_parse_scalar_u32_func bhy_parse_scalar_u32;
extern bhy_parse_scalar_event_func bhy_parse_scalar_event;
extern bhy_parse_activity_func bhy_parse_activity;

/*extern bhy_parse_u16_as_float_func bhy_parse_u16_as_float; */
extern bhy_parse_u24_as_float_func bhy_parse_u24_as_float;

/*extern bhy_parse_proximity_func bhy_parse_proximity; */
extern bhy_parse_scalar_u8_func bhy_parse_scalar_u8;
extern bhy_parse_generic_func bhy_parse_generic;
extern bhy_parse_device_ori_func bhy_parse_device_ori;

/*extern bhy_parse_gps_func bhy_parse_gps; */
extern bhy_parse_debug_message_func bhy_parse_debug_message;

/*extern bhy_parse_acc_gyro_func bhy_parse_acc_gyro; */
extern bhy_parse_multitap_func bhy_parse_multitap;
extern bhy_parse_wrist_gesture_detect_func bhy_parse_wrist_gesture_detect;
extern bhy_parse_step_counter_data_func bhy_parse_step_counter_data;
extern bhy_parse_wrist_wear_wakeup_data_func bhy_parse_wrist_wear_wakeup_data;
extern bhy_parse_air_quality_func bhy_parse_air_quality;
extern bhy_parse_hmc_func bhy_parse_hmc;
extern bhy_parse_oc_func bhy_parse_oc;
extern bhy_parse_ec_func bhy_parse_ec;

/*extern bhy_add_accuracy_to_sensor_data_func bhy_add_accuracy_to_sensor_data; */
extern bhy_set_downsampling_flag_func bhy_set_downsampling_flag;
extern bhy_get_downsampling_flag_func bhy_get_downsampling_flag;
extern bhy_set_downsampling_odr_func bhy_set_downsampling_odr;
extern bhy_parse_temperature_celsius_func bhy_parse_temperature_celsius;
extern bhy_parse_humidity_func bhy_parse_humidity;
extern bhy_parse_pressure_func bhy_parse_pressure;
extern bhy_parse_altitude_func bhy_parse_altitude;

extern bhy_klio_param_read_reset_driver_status_func bhy_klio_param_read_reset_driver_status;
extern bhy_klio_param_read_pattern_func bhy_klio_param_read_pattern;
extern bhy_klio_param_set_state_func bhy_klio_param_set_state;
extern bhy_klio_param_get_state_func bhy_klio_param_get_state;
extern bhy_klio_param_write_pattern_func bhy_klio_param_write_pattern;
extern bhy_klio_param_set_pattern_states_func bhy_klio_param_set_pattern_states;
extern bhy_klio_param_similarity_score_func bhy_klio_param_similarity_score;
extern bhy_klio_param_similarity_score_multiple_func bhy_klio_param_similarity_score_multiple;
extern bhy_klio_param_set_parameter_func bhy_klio_param_set_parameter;
extern bhy_klio_param_get_parameter_func bhy_klio_param_get_parameter;
extern bhy_klio_param_set_pattern_parameter_func bhy_klio_param_set_pattern_parameter;
extern bhy_klio_param_get_pattern_parameter_func bhy_klio_param_get_pattern_parameter;
extern bhy_klio_param_reset_func bhy_klio_param_reset;
extern bhy_parse_klio_func bhy_parse_klio;
extern bhy_parse_klio_log_func bhy_parse_klio_log;
extern bhy_get_klio_info_func bhy_get_klio_info;
extern bhy_set_klio_info_func bhy_set_klio_info;

/*     //bhy_swim_param_algo_callback; */
extern bhy_swim_param_get_config_func bhy_swim_param_get_config;
extern bhy_swim_param_set_config_func bhy_swim_param_set_config;
extern bhy_swim_param_get_version_func bhy_swim_param_get_version;
extern bhy_swim_param_set_logging_func bhy_swim_param_set_logging;
extern bhy_parse_swim_func bhy_parse_swim;
extern bhy_get_swim_data_func bhy_get_swim_data;

extern uint8_t cli_load_sensor_api_entry(uint8_t chip_id);
extern void *cli_load_api(const char *name);

#ifdef PC
#define QUIT_ON_ERROR(code)  exit(code)
#else

/*in MCU mode, let it runs into NULL pointer exception to restart cli connection*/
#define QUIT_ON_ERROR(code)
#endif

#define CALL_VOID_DYNAMIC_SENSOR_API(func_ptr, func_name, ...) \
    do { \
        if ((func_ptr) == NULL) { \
            func_ptr = cli_load_api(func_name); \
            if ((func_ptr) == NULL) { \
                ERROR("Function %s not found. May check if the feature is supported.\n", func_name); \
                QUIT_ON_ERROR(-1); \
            } \
        } \
        (void)func_ptr(__VA_ARGS__); \
    } while (0)

#define CALL_OUT_DYNAMIC_SENSOR_API(func_ptr, func_name, ret, ...) \
    do { \
        if ((func_ptr) == NULL) { \
            func_ptr = cli_load_api(func_name); \
            if ((func_ptr) == NULL) { \
                ERROR("Function %s not found. May check if the feature is supported.\n", func_name); \
                QUIT_ON_ERROR(-1); \
            } \
        } \
        ret = func_ptr(__VA_ARGS__); \
    } while (0)

#define LOAD_DYNAMIC_SENSOR_API(func_ptr, func_name, dest) \
    do { \
        if ((func_ptr) == NULL) { \
            func_ptr = cli_load_api(func_name); \
        } \
        dest = func_ptr; \
    } while (0)

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_DEFS_H__ */
