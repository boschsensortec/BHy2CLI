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
* @date       2025-03-28
* @version    v0.6.0
*
*/

#include "bhy_defs.h"
#include "verbose.h"

typedef struct
{
    const char *sensor_api_name;
    void *implementation_ptr;
} SensorAPIEntry;

typedef struct
{
    uint8_t chip_id;
    const SensorAPIEntry *api_entry;
} ChipAPIEntry;

#include "bhi360_api_entry.h"
#include "bhi360_defs.h"

bhy_init_func bhy_init = NULL;
bhy_get_chip_id_func bhy_get_chip_id = NULL;
bhy_get_and_process_fifo_func bhy_get_and_process_fifo = NULL;
bhy_register_fifo_parse_callback_func bhy_register_fifo_parse_callback = NULL;
bhy_set_host_intf_ctrl_func bhy_set_host_intf_ctrl = NULL;
bhy_get_host_intf_ctrl_func bhy_get_host_intf_ctrl = NULL;
bhy_activity_param_set_hearable_config_func bhy_activity_param_set_hearable_config = NULL;
bhy_activity_param_get_hearable_config_func bhy_activity_param_get_hearable_config = NULL;
bhy_activity_param_set_wearable_config_func bhy_activity_param_set_wearable_config = NULL;
bhy_activity_param_get_wearable_config_func bhy_activity_param_get_wearable_config = NULL;
bhy_bsec_param_set_algo_state_func bhy_bsec_param_set_algo_state = NULL;
bhy_bsec_param_get_algo_state_func bhy_bsec_param_get_algo_state = NULL;
bhy_bsec_param_set_temp_offset_func bhy_bsec_param_set_temp_offset = NULL;
bhy_bsec_param_get_temp_offset_func bhy_bsec_param_get_temp_offset = NULL;
bhy_bsec_param_set_sample_rate_func bhy_bsec_param_set_sample_rate = NULL;
bhy_bsec_param_get_sample_rate_func bhy_bsec_param_get_sample_rate = NULL;
bhy_bsx_algo_param_get_bsx_states_func bhy_bsx_algo_param_get_bsx_states = NULL;
bhy_bsx_algo_param_set_bsx_states_func bhy_bsx_algo_param_set_bsx_states = NULL;
bhy_bsx_algo_param_get_bsx_version_func bhy_bsx_algo_param_get_bsx_version = NULL;
bhy_event_data_parse_quaternion_func bhy_event_data_parse_quaternion = NULL;
bhy_event_data_parse_orientation_func bhy_event_data_parse_orientation = NULL;
bhy_event_data_parse_xyz_func bhy_event_data_parse_xyz = NULL;
bhy_event_data_head_orientation_quat_parsing_func bhy_event_data_head_orientation_quat_parsing = NULL;
bhy_event_data_head_orientation_eul_parsing_func bhy_event_data_head_orientation_eul_parsing = NULL;
bhy_event_data_multi_tap_parsing_func bhy_event_data_multi_tap_parsing = NULL;
bhy_event_data_wrist_gesture_detect_parsing_func bhy_event_data_wrist_gesture_detect_parsing = NULL;
bhy_event_data_parse_air_quality_func bhy_event_data_parse_air_quality = NULL;
bhy_head_orientation_param_trigger_hmc_calibration_func bhy_head_orientation_param_trigger_hmc_calibration = NULL;
bhy_head_orientation_param_set_hmc_configuration_func bhy_head_orientation_param_set_hmc_configuration = NULL;
bhy_head_orientation_param_get_hmc_configuration_func bhy_head_orientation_param_get_hmc_configuration = NULL;
bhy_head_orientation_param_set_default_hmc_cfg_func bhy_head_orientation_param_set_default_hmc_cfg = NULL;
bhy_head_orientation_param_get_hmc_version_func bhy_head_orientation_param_get_hmc_version = NULL;
bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg_func bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg = NULL;
bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg_func bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg = NULL;
bhy_head_orientation_param_set_hmc_mode_vector_x_func bhy_head_orientation_param_set_hmc_mode_vector_x = NULL;
bhy_head_orientation_param_get_hmc_mode_vector_x_func bhy_head_orientation_param_get_hmc_mode_vector_x = NULL;
bhy_head_orientation_param_set_quat_init_head_corr_func bhy_head_orientation_param_set_quat_init_head_corr = NULL;
bhy_head_orientation_param_get_quat_init_head_corr_func bhy_head_orientation_param_get_quat_init_head_corr = NULL;
bhy_head_orientation_param_get_ho_version_func bhy_head_orientation_param_get_ho_version = NULL;
bhy_head_orientation_param_set_eul_init_head_corr_func bhy_head_orientation_param_set_eul_init_head_corr = NULL;
bhy_head_orientation_param_get_eul_init_head_corr_func bhy_head_orientation_param_get_eul_init_head_corr = NULL;
bhy_logbin_start_meta_func bhy_logbin_start_meta = NULL;
bhy_logbin_add_meta_func bhy_logbin_add_meta = NULL;
bhy_logbin_end_meta_func bhy_logbin_end_meta = NULL;
bhy_logbin_add_data_func bhy_logbin_add_data = NULL;
bhy_multi_tap_param_set_config_func bhy_multi_tap_param_set_config = NULL;
bhy_multi_tap_param_get_config_func bhy_multi_tap_param_get_config = NULL;
bhy_multi_tap_param_detector_set_config_func bhy_multi_tap_param_detector_set_config = NULL;
bhy_multi_tap_param_detector_get_config_func bhy_multi_tap_param_detector_get_config = NULL;
bhy_phy_sensor_ctrl_param_accel_set_foc_calibration_func bhy_phy_sensor_ctrl_param_accel_set_foc_calibration = NULL;
bhy_phy_sensor_ctrl_param_accel_get_foc_calibration_func bhy_phy_sensor_ctrl_param_accel_get_foc_calibration = NULL;
bhy_phy_sensor_ctrl_param_accel_set_power_mode_func bhy_phy_sensor_ctrl_param_accel_set_power_mode = NULL;
bhy_phy_sensor_ctrl_param_accel_get_power_mode_func bhy_phy_sensor_ctrl_param_accel_get_power_mode = NULL;
bhy_phy_sensor_ctrl_param_accel_set_axis_remapping_func bhy_phy_sensor_ctrl_param_accel_set_axis_remapping = NULL;
bhy_phy_sensor_ctrl_param_accel_get_axis_remapping_func bhy_phy_sensor_ctrl_param_accel_get_axis_remapping = NULL;
bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing_func bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing = NULL;
bhy_phy_sensor_ctrl_param_accel_get_nvm_status_func bhy_phy_sensor_ctrl_param_accel_get_nvm_status = NULL;
bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration_func bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration_func bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration = NULL;
bhy_phy_sensor_ctrl_param_gyro_set_ois_config_func bhy_phy_sensor_ctrl_param_gyro_set_ois_config = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_ois_config_func bhy_phy_sensor_ctrl_param_gyro_get_ois_config = NULL;
bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg_func bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg_func bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg = NULL;
bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim_func bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_crt_status_func bhy_phy_sensor_ctrl_param_gyro_get_crt_status = NULL;
bhy_phy_sensor_ctrl_param_gyro_set_power_mode_func bhy_phy_sensor_ctrl_param_gyro_set_power_mode = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_power_mode_func bhy_phy_sensor_ctrl_param_gyro_get_power_mode = NULL;
bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg_func bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg_func bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg = NULL;
bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing_func bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing = NULL;
bhy_phy_sensor_ctrl_param_gyro_get_nvm_status_func bhy_phy_sensor_ctrl_param_gyro_get_nvm_status = NULL;
bhy_phy_sensor_ctrl_param_magnet_set_power_mode_func bhy_phy_sensor_ctrl_param_magnet_set_power_mode = NULL;
bhy_phy_sensor_ctrl_param_magnet_get_power_mode_func bhy_phy_sensor_ctrl_param_magnet_get_power_mode = NULL;
bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg_func bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg = NULL;
bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg_func bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg = NULL;
bhy_phy_sensor_ctrl_param_set_any_motion_config_func bhy_phy_sensor_ctrl_param_set_any_motion_config = NULL;
bhy_phy_sensor_ctrl_param_get_any_motion_config_func bhy_phy_sensor_ctrl_param_get_any_motion_config = NULL;
bhy_phy_sensor_ctrl_param_set_no_motion_config_func bhy_phy_sensor_ctrl_param_set_no_motion_config = NULL;
bhy_phy_sensor_ctrl_param_get_no_motion_config_func bhy_phy_sensor_ctrl_param_get_no_motion_config = NULL;
bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg_func bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg = NULL;
bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg_func bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg = NULL;
bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg_func bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg = NULL;
bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg_func bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg = NULL;
bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg_func bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg = NULL;
bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg_func bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg = NULL;
bhy_phy_sensor_ctrl_param_set_step_counter_config_func bhy_phy_sensor_ctrl_param_set_step_counter_config = NULL;
bhy_phy_sensor_ctrl_param_get_step_counter_config_func bhy_phy_sensor_ctrl_param_get_step_counter_config = NULL;
bhy_system_param_set_meta_event_control_func bhy_system_param_set_meta_event_control = NULL;
bhy_system_param_get_meta_event_control_func bhy_system_param_get_meta_event_control = NULL;
bhy_system_param_set_wakeup_fifo_control_func bhy_system_param_set_wakeup_fifo_control = NULL;
bhy_system_param_set_nonwakeup_fifo_control_func bhy_system_param_set_nonwakeup_fifo_control = NULL;
bhy_system_param_get_fifo_control_func bhy_system_param_get_fifo_control = NULL;
bhy_system_param_get_firmware_version_func bhy_system_param_get_firmware_version = NULL;
bhy_system_param_get_timestamps_func bhy_system_param_get_timestamps = NULL;
bhy_system_param_get_virtual_sensor_present_func bhy_system_param_get_virtual_sensor_present = NULL;
bhy_system_param_get_physical_sensor_present_func bhy_system_param_get_physical_sensor_present = NULL;
bhy_system_param_get_physical_sensor_info_func bhy_system_param_get_physical_sensor_info = NULL;
bhy_system_param_set_physical_sensor_info_func bhy_system_param_set_physical_sensor_info = NULL;
bhy_virtual_sensor_conf_param_set_cfg_func bhy_virtual_sensor_conf_param_set_cfg = NULL;
bhy_virtual_sensor_conf_param_get_cfg_func bhy_virtual_sensor_conf_param_get_cfg = NULL;
bhy_virtual_sensor_info_param_get_info_func bhy_virtual_sensor_info_param_get_info = NULL;
bhy_get_regs_func bhy_get_regs = NULL;
bhy_set_regs_func bhy_set_regs = NULL;
bhy_get_product_id_func bhy_get_product_id = NULL;
bhy_get_revision_id_func bhy_get_revision_id = NULL;
bhy_get_rom_version_func bhy_get_rom_version = NULL;
bhy_get_kernel_version_func bhy_get_kernel_version = NULL;
bhy_get_user_version_func bhy_get_user_version = NULL;
bhy_get_boot_status_func bhy_get_boot_status = NULL;
bhy_get_host_status_func bhy_get_host_status = NULL;
bhy_get_feature_status_func bhy_get_feature_status = NULL;
bhy_set_virt_sensor_range_func bhy_set_virt_sensor_range = NULL;

/*
 * bhy_get_fifo_wmark_wkup_func bhy_get_fifo_wmark_wkup = NULL;
 * bhy_get_fifo_wmark_nonwkup_func bhy_get_fifo_wmark_nonwkup = NULL;
 */
bhy_flush_fifo_func bhy_flush_fifo = NULL;
bhy_set_fifo_format_ctrl_func bhy_set_fifo_format_ctrl = NULL;
bhy_upload_firmware_to_ram_func bhy_upload_firmware_to_ram = NULL;
bhy_upload_firmware_to_ram_partly_func bhy_upload_firmware_to_ram_partly = NULL;
bhy_boot_from_ram_func bhy_boot_from_ram = NULL;
bhy_set_host_interrupt_ctrl_func bhy_set_host_interrupt_ctrl = NULL;
bhy_get_host_interrupt_ctrl_func bhy_get_host_interrupt_ctrl = NULL;
bhy_get_interrupt_status_func bhy_get_interrupt_status = NULL;
bhy_set_timestamp_event_req_func bhy_set_timestamp_event_req = NULL;
bhy_get_hw_timestamp_ns_func bhy_get_hw_timestamp_ns = NULL;
bhy_set_host_ctrl_func bhy_set_host_ctrl = NULL;
bhy_get_host_ctrl_func bhy_get_host_ctrl = NULL;
bhy_soft_reset_func bhy_soft_reset = NULL;
bhy_perform_self_test_func bhy_perform_self_test = NULL;
bhy_perform_foc_func bhy_perform_foc = NULL;
bhy_get_orientation_matrix_func bhy_get_orientation_matrix = NULL;
bhy_get_post_mortem_data_func bhy_get_post_mortem_data = NULL;
bhy_deregister_fifo_parse_callback_func bhy_deregister_fifo_parse_callback = NULL;
bhy_update_virtual_sensor_list_func bhy_update_virtual_sensor_list = NULL;
bhy_get_sensor_info_func bhy_get_sensor_info = NULL;
bhy_set_parameter_func bhy_set_parameter = NULL;
bhy_get_parameter_func bhy_get_parameter = NULL;
bhy_get_error_value_func bhy_get_error_value = NULL;
bhy_soft_passthrough_transfer_func bhy_soft_passthrough_transfer = NULL;
bhy_is_sensor_available_func bhy_is_sensor_available = NULL;
bhy_is_physical_sensor_available_func bhy_is_physical_sensor_available = NULL;
bhy_get_variant_id_func bhy_get_variant_id = NULL;
bhy_inject_data_func bhy_inject_data = NULL;
bhy_set_data_injection_mode_func bhy_set_data_injection_mode = NULL;
bhy_clear_fifo_func bhy_clear_fifo = NULL;
bhy_read_status_func bhy_read_status = NULL;

bhy_parse_get_sensor_details_func bhy_parse_get_sensor_details = NULL;
bhy_parse_add_sensor_details_func bhy_parse_add_sensor_details = NULL;
bhy_parse_meta_event_func bhy_parse_meta_event = NULL;
bhy_parse_3axis_s16_func bhy_parse_3axis_s16 = NULL;
bhy_parse_euler_func bhy_parse_euler = NULL;
bhy_parse_quaternion_func bhy_parse_quaternion = NULL;
bhy_parse_s16_as_float_func bhy_parse_s16_as_float = NULL;
bhy_parse_scalar_u32_func bhy_parse_scalar_u32 = NULL;
bhy_parse_scalar_event_func bhy_parse_scalar_event = NULL;
bhy_parse_activity_func bhy_parse_activity = NULL;

/*bhy_parse_u16_as_float_func bhy_parse_u16_as_float = NULL; */
bhy_parse_u24_as_float_func bhy_parse_u24_as_float = NULL;

/*bhy_parse_proximity_func bhy_parse_proximity = NULL; */
bhy_parse_scalar_u8_func bhy_parse_scalar_u8 = NULL;
bhy_parse_generic_func bhy_parse_generic = NULL;
bhy_parse_device_ori_func bhy_parse_device_ori = NULL;

/*bhy_parse_gps_func bhy_parse_gps = NULL; */
bhy_parse_debug_message_func bhy_parse_debug_message = NULL;

/*bhy_parse_acc_gyro_func bhy_parse_acc_gyro = NULL; */
bhy_parse_multitap_func bhy_parse_multitap = NULL;
bhy_parse_wrist_gesture_detect_func bhy_parse_wrist_gesture_detect = NULL;
bhy_parse_air_quality_func bhy_parse_air_quality = NULL;
bhy_parse_hmc_func bhy_parse_hmc = NULL;
bhy_parse_oc_func bhy_parse_oc = NULL;
bhy_parse_ec_func bhy_parse_ec = NULL;

/*bhy_add_accuracy_to_sensor_data_func bhy_add_accuracy_to_sensor_data = NULL; */
bhy_set_downsampling_flag_func bhy_set_downsampling_flag = NULL;
bhy_get_downsampling_flag_func bhy_get_downsampling_flag = NULL;
bhy_set_downsampling_odr_func bhy_set_downsampling_odr = NULL;

bhy_parse_temperature_celsius_func bhy_parse_temperature_celsius = NULL;
bhy_parse_humidity_func bhy_parse_humidity = NULL;
bhy_parse_pressure_func bhy_parse_pressure = NULL;
bhy_parse_altitude_func bhy_parse_altitude = NULL;

bhy_parse_klio_func bhy_parse_klio = NULL;
bhy_parse_klio_generic_func bhy_parse_klio_generic = NULL;
bhy_parse_klio_log_func bhy_parse_klio_log = NULL;
bhy_klio_param_read_reset_driver_status_func bhy_klio_param_read_reset_driver_status = NULL;
bhy_klio_param_read_pattern_func bhy_klio_param_read_pattern = NULL;
bhy_klio_param_set_state_func bhy_klio_param_set_state = NULL;
bhy_klio_param_set_generic_recognition_state_func bhy_klio_param_set_generic_recognition_state = NULL;
bhy_klio_param_get_state_func bhy_klio_param_get_state = NULL;
bhy_klio_param_get_generic_recognition_state_func bhy_klio_param_get_generic_recognition_state = NULL;
bhy_klio_param_write_pattern_func bhy_klio_param_write_pattern = NULL;
bhy_klio_param_write_generic_pattern_func bhy_klio_param_write_generic_pattern = NULL;
bhy_klio_param_write_gesture_config_func bhy_klio_param_write_gesture_config = NULL;
bhy_klio_param_write_timing_config_func bhy_klio_param_write_timing_config = NULL;
bhy_klio_param_set_pattern_states_func bhy_klio_param_set_pattern_states = NULL;
bhy_klio_param_similarity_score_func bhy_klio_param_similarity_score = NULL;
bhy_klio_param_similarity_score_multiple_func bhy_klio_param_similarity_score_multiple = NULL;
bhy_klio_param_set_parameter_func bhy_klio_param_set_parameter = NULL;
bhy_klio_param_get_parameter_func bhy_klio_param_get_parameter = NULL;
bhy_klio_param_reset_func bhy_klio_param_reset = NULL;

bhy_get_klio_info_func bhy_get_klio_info = NULL;
bhy_set_klio_info_func bhy_set_klio_info = NULL;

/*     //bhy_swim_param_algo_callback = NULL; */
bhy_swim_param_get_config_func bhy_swim_param_get_config = NULL;
bhy_swim_param_set_config_func bhy_swim_param_set_config = NULL;
bhy_swim_param_get_version_func bhy_swim_param_get_version = NULL;
bhy_swim_param_set_logging_func bhy_swim_param_set_logging = NULL;
bhy_parse_swim_func bhy_parse_swim = NULL;
bhy_get_swim_data_func bhy_get_swim_data = NULL;

static const SensorAPIEntry *sensor_api_table = NULL;

#define MAX_BHY_API_NUM  250 /*no less than the number the above APIs */
static bool linked[MAX_BHY_API_NUM] = { false };

void *cli_load_api(const char *name)
{
    size_t i = 0;

    while (sensor_api_table[i].implementation_ptr != NULL /* means end of api table */
           && i <= MAX_BHY_API_NUM)
    {
        if ((linked[i] == false) && (strcmp(sensor_api_table[i].sensor_api_name, name) == 0))
        {
            linked[i] = true; /* Mark as linked */
            return sensor_api_table[i].implementation_ptr;
        }

        i++;
    }

    return NULL; /* Return NULL if the key is not found */
}

static const ChipAPIEntry all_sensor_api_entry[] = {
    { BHI360_CHIP_ID, bhi360_sensor_api_entry }, { 0, NULL } /* End of
                                                                                                           * table
                                                                                                           * marker */
};

uint8_t cli_load_sensor_api_entry(uint8_t chip_id)
{
    for (size_t i = 0; all_sensor_api_entry[i].api_entry != NULL; i++)
    {
        PRINT("Include sensor API with chip id: 0x%X\r\n", all_sensor_api_entry[i].chip_id);

        if (sensor_api_table == NULL && all_sensor_api_entry[i].chip_id == chip_id)
        {
            sensor_api_table = all_sensor_api_entry[i].api_entry;
        }
    }

    if (sensor_api_table == NULL)
    {
        PRINT("Unknown chip id: 0x%X, use BHI360 API as default\r\n", chip_id);
        sensor_api_table = bhi360_sensor_api_entry; /* default to use BHI360 API*/
    }

    /*check length of API table*/
    size_t i = 0;
    while (sensor_api_table[i].implementation_ptr != NULL /* means end of api table */
           && i <= MAX_BHY_API_NUM)
    {
        i++;
    }

    if (i > MAX_BHY_API_NUM)
    {
        return BHY_E_API_NUM_OUT;
    }

    bhy_init = (bhy_init_func)cli_load_api("bhy_init");
    bhy_get_chip_id = (bhy_get_chip_id_func)cli_load_api("bhy_get_chip_id");
    bhy_get_and_process_fifo = (bhy_get_and_process_fifo_func)cli_load_api("bhy_get_and_process_fifo");
    bhy_register_fifo_parse_callback = (bhy_register_fifo_parse_callback_func)cli_load_api(
        "bhy_register_fifo_parse_callback");
    bhy_set_host_intf_ctrl = (bhy_set_host_intf_ctrl_func)cli_load_api("bhy_set_host_intf_ctrl");
    bhy_get_host_intf_ctrl = (bhy_get_host_intf_ctrl_func)cli_load_api("bhy_get_host_intf_ctrl");
    bhy_activity_param_set_hearable_config = (bhy_activity_param_set_hearable_config_func)cli_load_api(
        "bhy_activity_param_set_hearable_config");
    bhy_activity_param_get_hearable_config = (bhy_activity_param_get_hearable_config_func)cli_load_api(
        "bhy_activity_param_get_hearable_config");
    bhy_activity_param_set_wearable_config = (bhy_activity_param_set_wearable_config_func)cli_load_api(
        "bhy_activity_param_set_wearable_config");
    bhy_activity_param_get_wearable_config = (bhy_activity_param_get_wearable_config_func)cli_load_api(
        "bhy_activity_param_get_wearable_config");
    bhy_bsec_param_set_algo_state = (bhy_bsec_param_set_algo_state_func)cli_load_api("bhy_bsec_param_set_algo_state");
    bhy_bsec_param_get_algo_state = (bhy_bsec_param_get_algo_state_func)cli_load_api("bhy_bsec_param_get_algo_state");
    bhy_bsec_param_set_temp_offset =
        (bhy_bsec_param_set_temp_offset_func)cli_load_api("bhy_bsec_param_set_temp_offset");
    bhy_bsec_param_get_temp_offset =
        (bhy_bsec_param_get_temp_offset_func)cli_load_api("bhy_bsec_param_get_temp_offset");
    bhy_bsec_param_set_sample_rate =
        (bhy_bsec_param_set_sample_rate_func)cli_load_api("bhy_bsec_param_set_sample_rate");
    bhy_bsec_param_get_sample_rate =
        (bhy_bsec_param_get_sample_rate_func)cli_load_api("bhy_bsec_param_get_sample_rate");
    bhy_bsx_algo_param_get_bsx_states = (bhy_bsx_algo_param_get_bsx_states_func)cli_load_api(
        "bhy_bsx_algo_param_get_bsx_states");
    bhy_bsx_algo_param_set_bsx_states = (bhy_bsx_algo_param_set_bsx_states_func)cli_load_api(
        "bhy_bsx_algo_param_set_bsx_states");
    bhy_bsx_algo_param_get_bsx_version = (bhy_bsx_algo_param_get_bsx_version_func)cli_load_api(
        "bhy_bsx_algo_param_get_bsx_version");
    bhy_event_data_parse_quaternion = (bhy_event_data_parse_quaternion_func)cli_load_api(
        "bhy_event_data_parse_quaternion");
    bhy_event_data_parse_orientation = (bhy_event_data_parse_orientation_func)cli_load_api(
        "bhy_event_data_parse_orientation");
    bhy_event_data_parse_xyz = (bhy_event_data_parse_xyz_func)cli_load_api("bhy_event_data_parse_xyz");
    bhy_event_data_head_orientation_quat_parsing = (bhy_event_data_head_orientation_quat_parsing_func)cli_load_api(
        "bhy_event_data_head_orientation_quat_parsing");
    bhy_event_data_head_orientation_eul_parsing = (bhy_event_data_head_orientation_eul_parsing_func)cli_load_api(
        "bhy_event_data_head_orientation_eul_parsing");
    bhy_event_data_multi_tap_parsing = (bhy_event_data_multi_tap_parsing_func)cli_load_api(
        "bhy_event_data_multi_tap_parsing");
    bhy_event_data_wrist_gesture_detect_parsing = (bhy_event_data_wrist_gesture_detect_parsing_func)cli_load_api(
        "bhy_event_data_wrist_gesture_detect_parsing");
    bhy_event_data_parse_air_quality = (bhy_event_data_parse_air_quality_func)cli_load_api(
        "bhy_event_data_parse_air_quality");
    bhy_head_orientation_param_trigger_hmc_calibration =
        (bhy_head_orientation_param_trigger_hmc_calibration_func)cli_load_api(
            "bhy_head_orientation_param_trigger_hmc_calibration");
    bhy_head_orientation_param_set_hmc_configuration =
        (bhy_head_orientation_param_set_hmc_configuration_func)cli_load_api(
            "bhy_head_orientation_param_set_hmc_configuration");
    bhy_head_orientation_param_get_hmc_configuration =
        (bhy_head_orientation_param_get_hmc_configuration_func)cli_load_api(
            "bhy_head_orientation_param_get_hmc_configuration");
    bhy_head_orientation_param_set_default_hmc_cfg = (bhy_head_orientation_param_set_default_hmc_cfg_func)cli_load_api(
        "bhy_head_orientation_param_set_default_hmc_cfg");
    bhy_head_orientation_param_get_hmc_version = (bhy_head_orientation_param_get_hmc_version_func)cli_load_api(
        "bhy_head_orientation_param_get_hmc_version");
    bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg =
        (bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg_func)cli_load_api(
            "bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg");
    bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg =
        (bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg_func)cli_load_api(
            "bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg");
    bhy_head_orientation_param_set_hmc_mode_vector_x =
        (bhy_head_orientation_param_set_hmc_mode_vector_x_func)cli_load_api(
            "bhy_head_orientation_param_set_hmc_mode_vector_x");
    bhy_head_orientation_param_get_hmc_mode_vector_x =
        (bhy_head_orientation_param_get_hmc_mode_vector_x_func)cli_load_api(
            "bhy_head_orientation_param_get_hmc_mode_vector_x");
    bhy_head_orientation_param_set_quat_init_head_corr =
        (bhy_head_orientation_param_set_quat_init_head_corr_func)cli_load_api(
            "bhy_head_orientation_param_set_quat_init_head_corr");
    bhy_head_orientation_param_get_quat_init_head_corr =
        (bhy_head_orientation_param_get_quat_init_head_corr_func)cli_load_api(
            "bhy_head_orientation_param_get_quat_init_head_corr");
    bhy_head_orientation_param_get_ho_version = (bhy_head_orientation_param_get_ho_version_func)cli_load_api(
        "bhy_head_orientation_param_get_ho_version");
    bhy_head_orientation_param_set_eul_init_head_corr =
        (bhy_head_orientation_param_set_eul_init_head_corr_func)cli_load_api(
            "bhy_head_orientation_param_set_eul_init_head_corr");
    bhy_head_orientation_param_get_eul_init_head_corr =
        (bhy_head_orientation_param_get_eul_init_head_corr_func)cli_load_api(
            "bhy_head_orientation_param_get_eul_init_head_corr");
    bhy_logbin_start_meta = (bhy_logbin_start_meta_func)cli_load_api("bhy_logbin_start_meta");
    bhy_logbin_add_meta = (bhy_logbin_add_meta_func)cli_load_api("bhy_logbin_add_meta");
    bhy_logbin_end_meta = (bhy_logbin_end_meta_func)cli_load_api("bhy_logbin_end_meta");
    bhy_logbin_add_data = (bhy_logbin_add_data_func)cli_load_api("bhy_logbin_add_data");
    bhy_multi_tap_param_set_config =
        (bhy_multi_tap_param_set_config_func)cli_load_api("bhy_multi_tap_param_set_config");
    bhy_multi_tap_param_get_config =
        (bhy_multi_tap_param_get_config_func)cli_load_api("bhy_multi_tap_param_get_config");
    bhy_multi_tap_param_detector_set_config = (bhy_multi_tap_param_detector_set_config_func)cli_load_api(
        "bhy_multi_tap_param_detector_set_config");
    bhy_multi_tap_param_detector_get_config = (bhy_multi_tap_param_detector_get_config_func)cli_load_api(
        "bhy_multi_tap_param_detector_get_config");
    bhy_phy_sensor_ctrl_param_accel_set_foc_calibration =
        (bhy_phy_sensor_ctrl_param_accel_set_foc_calibration_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_accel_set_foc_calibration");
    bhy_phy_sensor_ctrl_param_accel_get_foc_calibration =
        (bhy_phy_sensor_ctrl_param_accel_get_foc_calibration_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_accel_get_foc_calibration");
    bhy_phy_sensor_ctrl_param_accel_set_power_mode = (bhy_phy_sensor_ctrl_param_accel_set_power_mode_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_accel_set_power_mode");
    bhy_phy_sensor_ctrl_param_accel_get_power_mode = (bhy_phy_sensor_ctrl_param_accel_get_power_mode_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_accel_get_power_mode");
    bhy_phy_sensor_ctrl_param_accel_set_axis_remapping =
        (bhy_phy_sensor_ctrl_param_accel_set_axis_remapping_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_accel_set_axis_remapping");
    bhy_phy_sensor_ctrl_param_accel_get_axis_remapping =
        (bhy_phy_sensor_ctrl_param_accel_get_axis_remapping_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_accel_get_axis_remapping");
    bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing =
        (bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing");
    bhy_phy_sensor_ctrl_param_accel_get_nvm_status = (bhy_phy_sensor_ctrl_param_accel_get_nvm_status_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_accel_get_nvm_status");
    bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration =
        (bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration");
    bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration =
        (bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration");
    bhy_phy_sensor_ctrl_param_gyro_set_ois_config = (bhy_phy_sensor_ctrl_param_gyro_set_ois_config_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_set_ois_config");
    bhy_phy_sensor_ctrl_param_gyro_get_ois_config = (bhy_phy_sensor_ctrl_param_gyro_get_ois_config_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_get_ois_config");
    bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg =
        (bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg");
    bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg =
        (bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg");
    bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim =
        (bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim");
    bhy_phy_sensor_ctrl_param_gyro_get_crt_status = (bhy_phy_sensor_ctrl_param_gyro_get_crt_status_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_get_crt_status");
    bhy_phy_sensor_ctrl_param_gyro_set_power_mode = (bhy_phy_sensor_ctrl_param_gyro_set_power_mode_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_set_power_mode");
    bhy_phy_sensor_ctrl_param_gyro_get_power_mode = (bhy_phy_sensor_ctrl_param_gyro_get_power_mode_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_get_power_mode");
    bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg =
        (bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg");
    bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg =
        (bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg");
    bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing =
        (bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing");
    bhy_phy_sensor_ctrl_param_gyro_get_nvm_status = (bhy_phy_sensor_ctrl_param_gyro_get_nvm_status_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_gyro_get_nvm_status");
    bhy_phy_sensor_ctrl_param_magnet_set_power_mode =
        (bhy_phy_sensor_ctrl_param_magnet_set_power_mode_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_magnet_set_power_mode");
    bhy_phy_sensor_ctrl_param_magnet_get_power_mode =
        (bhy_phy_sensor_ctrl_param_magnet_get_power_mode_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_magnet_get_power_mode");
    bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg =
        (bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg");
    bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg =
        (bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg");
    bhy_phy_sensor_ctrl_param_set_any_motion_config =
        (bhy_phy_sensor_ctrl_param_set_any_motion_config_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_set_any_motion_config");
    bhy_phy_sensor_ctrl_param_get_any_motion_config =
        (bhy_phy_sensor_ctrl_param_get_any_motion_config_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_get_any_motion_config");
    bhy_phy_sensor_ctrl_param_set_no_motion_config = (bhy_phy_sensor_ctrl_param_set_no_motion_config_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_set_no_motion_config");
    bhy_phy_sensor_ctrl_param_get_no_motion_config = (bhy_phy_sensor_ctrl_param_get_no_motion_config_func)cli_load_api(
        "bhy_phy_sensor_ctrl_param_get_no_motion_config");
    bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg =
        (bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg");
    bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg =
        (bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg");
    bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg =
        (bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg");
    bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg =
        (bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg");
    bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg =
        (bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg");
    bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg =
        (bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg");
    bhy_phy_sensor_ctrl_param_set_step_counter_config =
        (bhy_phy_sensor_ctrl_param_set_step_counter_config_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_set_step_counter_config");
    bhy_phy_sensor_ctrl_param_get_step_counter_config =
        (bhy_phy_sensor_ctrl_param_get_step_counter_config_func)cli_load_api(
            "bhy_phy_sensor_ctrl_param_get_step_counter_config");
    bhy_system_param_set_meta_event_control = (bhy_system_param_set_meta_event_control_func)cli_load_api(
        "bhy_system_param_set_meta_event_control");
    bhy_system_param_get_meta_event_control = (bhy_system_param_get_meta_event_control_func)cli_load_api(
        "bhy_system_param_get_meta_event_control");
    bhy_system_param_set_wakeup_fifo_control = (bhy_system_param_set_wakeup_fifo_control_func)cli_load_api(
        "bhy_system_param_set_wakeup_fifo_control");
    bhy_system_param_set_nonwakeup_fifo_control = (bhy_system_param_set_nonwakeup_fifo_control_func)cli_load_api(
        "bhy_system_param_set_nonwakeup_fifo_control");
    bhy_system_param_get_fifo_control = (bhy_system_param_get_fifo_control_func)cli_load_api(
        "bhy_system_param_get_fifo_control");
    bhy_system_param_get_firmware_version = (bhy_system_param_get_firmware_version_func)cli_load_api(
        "bhy_system_param_get_firmware_version");
    bhy_system_param_get_timestamps = (bhy_system_param_get_timestamps_func)cli_load_api(
        "bhy_system_param_get_timestamps");
    bhy_system_param_get_virtual_sensor_present = (bhy_system_param_get_virtual_sensor_present_func)cli_load_api(
        "bhy_system_param_get_virtual_sensor_present");
    bhy_system_param_get_physical_sensor_present = (bhy_system_param_get_physical_sensor_present_func)cli_load_api(
        "bhy_system_param_get_physical_sensor_present");
    bhy_system_param_get_physical_sensor_info = (bhy_system_param_get_physical_sensor_info_func)cli_load_api(
        "bhy_system_param_get_physical_sensor_info");
    bhy_system_param_set_physical_sensor_info = (bhy_system_param_set_physical_sensor_info_func)cli_load_api(
        "bhy_system_param_set_physical_sensor_info");
    bhy_virtual_sensor_conf_param_set_cfg = (bhy_virtual_sensor_conf_param_set_cfg_func)cli_load_api(
        "bhy_virtual_sensor_conf_param_set_cfg");
    bhy_virtual_sensor_conf_param_get_cfg = (bhy_virtual_sensor_conf_param_get_cfg_func)cli_load_api(
        "bhy_virtual_sensor_conf_param_get_cfg");
    bhy_virtual_sensor_info_param_get_info = (bhy_virtual_sensor_info_param_get_info_func)cli_load_api(
        "bhy_virtual_sensor_info_param_get_info");
    bhy_get_regs = (bhy_get_regs_func)cli_load_api("bhy_get_regs");
    bhy_set_regs = (bhy_set_regs_func)cli_load_api("bhy_set_regs");
    bhy_get_product_id = (bhy_get_product_id_func)cli_load_api("bhy_get_product_id");
    bhy_get_revision_id = (bhy_get_revision_id_func)cli_load_api("bhy_get_revision_id");
    bhy_get_rom_version = (bhy_get_rom_version_func)cli_load_api("bhy_get_rom_version");
    bhy_get_kernel_version = (bhy_get_kernel_version_func)cli_load_api("bhy_get_kernel_version");
    bhy_get_user_version = (bhy_get_user_version_func)cli_load_api("bhy_get_user_version");
    bhy_get_boot_status = (bhy_get_boot_status_func)cli_load_api("bhy_get_boot_status");
    bhy_get_host_status = (bhy_get_host_status_func)cli_load_api("bhy_get_host_status");
    bhy_get_feature_status = (bhy_get_feature_status_func)cli_load_api("bhy_get_feature_status");
    bhy_set_virt_sensor_range = (bhy_set_virt_sensor_range_func)cli_load_api("bhy_set_virt_sensor_range");

    /*
     * bhy_get_fifo_wmark_wkup_func bhy_get_fifo_wmark_wkup = NULL;
     * bhy_get_fifo_wmark_nonwkup_func bhy_get_fifo_wmark_nonwkup = NULL;
     */
    bhy_flush_fifo = (bhy_flush_fifo_func)cli_load_api("bhy_flush_fifo");
    bhy_set_fifo_format_ctrl = (bhy_set_fifo_format_ctrl_func)cli_load_api("bhy_set_fifo_format_ctrl");
    bhy_upload_firmware_to_ram = (bhy_upload_firmware_to_ram_func)cli_load_api("bhy_upload_firmware_to_ram");
    bhy_upload_firmware_to_ram_partly = (bhy_upload_firmware_to_ram_partly_func)cli_load_api(
        "bhy_upload_firmware_to_ram_partly");
    bhy_boot_from_ram = (bhy_boot_from_ram_func)cli_load_api("bhy_boot_from_ram");
    bhy_set_host_interrupt_ctrl = (bhy_set_host_interrupt_ctrl_func)cli_load_api("bhy_set_host_interrupt_ctrl");
    bhy_get_host_interrupt_ctrl = (bhy_get_host_interrupt_ctrl_func)cli_load_api("bhy_get_host_interrupt_ctrl");
    bhy_get_interrupt_status = (bhy_get_interrupt_status_func)cli_load_api("bhy_get_interrupt_status");
    bhy_set_timestamp_event_req = (bhy_set_timestamp_event_req_func)cli_load_api("bhy_set_timestamp_event_req");
    bhy_get_hw_timestamp_ns = (bhy_get_hw_timestamp_ns_func)cli_load_api("bhy_get_hw_timestamp_ns");
    bhy_set_host_ctrl = (bhy_set_host_ctrl_func)cli_load_api("bhy_set_host_ctrl");
    bhy_get_host_ctrl = (bhy_get_host_ctrl_func)cli_load_api("bhy_get_host_ctrl");
    bhy_soft_reset = (bhy_soft_reset_func)cli_load_api("bhy_soft_reset");
    bhy_perform_self_test = (bhy_perform_self_test_func)cli_load_api("bhy_perform_self_test");
    bhy_perform_foc = (bhy_perform_foc_func)cli_load_api("bhy_perform_foc");
    bhy_get_orientation_matrix = (bhy_get_orientation_matrix_func)cli_load_api("bhy_get_orientation_matrix");
    bhy_get_post_mortem_data = (bhy_get_post_mortem_data_func)cli_load_api("bhy_get_post_mortem_data");
    bhy_deregister_fifo_parse_callback = (bhy_deregister_fifo_parse_callback_func)cli_load_api(
        "bhy_deregister_fifo_parse_callback");
    bhy_update_virtual_sensor_list =
        (bhy_update_virtual_sensor_list_func)cli_load_api("bhy_update_virtual_sensor_list");
    bhy_get_sensor_info = (bhy_get_sensor_info_func)cli_load_api("bhy_get_sensor_info");
    bhy_set_parameter = (bhy_set_parameter_func)cli_load_api("bhy_set_parameter");
    bhy_get_parameter = (bhy_get_parameter_func)cli_load_api("bhy_get_parameter");
    bhy_get_error_value = (bhy_get_error_value_func)cli_load_api("bhy_get_error_value");
    bhy_soft_passthrough_transfer = (bhy_soft_passthrough_transfer_func)cli_load_api("bhy_soft_passthrough_transfer");
    bhy_is_sensor_available = (bhy_is_sensor_available_func)cli_load_api("bhy_is_sensor_available");
    bhy_is_physical_sensor_available = (bhy_is_physical_sensor_available_func)cli_load_api(
        "bhy_is_physical_sensor_available");
    bhy_get_variant_id = (bhy_get_variant_id_func)cli_load_api("bhy_get_variant_id");
    bhy_inject_data = (bhy_inject_data_func)cli_load_api("bhy_inject_data");
    bhy_set_data_injection_mode = (bhy_set_data_injection_mode_func)cli_load_api("bhy_set_data_injection_mode");
    bhy_clear_fifo = (bhy_clear_fifo_func)cli_load_api("bhy_clear_fifo");
    bhy_read_status = (bhy_read_status_func)cli_load_api("bhy_read_status");

    bhy_parse_get_sensor_details = (bhy_parse_get_sensor_details_func)cli_load_api("bhy_parse_get_sensor_details");
    bhy_parse_add_sensor_details = (bhy_parse_add_sensor_details_func)cli_load_api("bhy_parse_add_sensor_details");
    bhy_parse_meta_event = (bhy_parse_meta_event_func)cli_load_api("bhy_parse_meta_event");
    bhy_parse_3axis_s16 = (bhy_parse_3axis_s16_func)cli_load_api("bhy_parse_3axis_s16");
    bhy_parse_euler = (bhy_parse_euler_func)cli_load_api("bhy_parse_euler");
    bhy_parse_quaternion = (bhy_parse_quaternion_func)cli_load_api("bhy_parse_quaternion");
    bhy_parse_s16_as_float = (bhy_parse_s16_as_float_func)cli_load_api("bhy_parse_s16_as_float");
    bhy_parse_scalar_u32 = (bhy_parse_scalar_u32_func)cli_load_api("bhy_parse_scalar_u32");
    bhy_parse_scalar_event = (bhy_parse_scalar_event_func)cli_load_api("bhy_parse_scalar_event");
    bhy_parse_activity = (bhy_parse_activity_func)cli_load_api("bhy_parse_activity");

    /*bhy_parse_u16_as_float_func bhy_parse_u16_as_float = NULL; */
    bhy_parse_u24_as_float = (bhy_parse_u24_as_float_func)cli_load_api("bhy_parse_u24_as_float");

    /*bhy_parse_proximity_func bhy_parse_proximity = NULL; */
    bhy_parse_scalar_u8 = (bhy_parse_scalar_u8_func)cli_load_api("bhy_parse_scalar_u8");
    bhy_parse_generic = (bhy_parse_generic_func)cli_load_api("bhy_parse_generic");
    bhy_parse_device_ori = (bhy_parse_device_ori_func)cli_load_api("bhy_parse_device_ori");

    /*bhy_parse_gps_func bhy_parse_gps = NULL; */
    bhy_parse_debug_message = (bhy_parse_debug_message_func)cli_load_api("bhy_parse_debug_message");

    /*bhy_parse_acc_gyro = (bhy_parse_acc_gyro_func)cli_load_api("bhy_parse_acc_gyro"); */
    bhy_parse_multitap = (bhy_parse_multitap_func)cli_load_api("bhy_parse_multitap");
    bhy_parse_wrist_gesture_detect =
        (bhy_parse_wrist_gesture_detect_func)cli_load_api("bhy_parse_wrist_gesture_detect");
    bhy_parse_air_quality = (bhy_parse_air_quality_func)cli_load_api("bhy_parse_air_quality");
    bhy_parse_hmc = (bhy_parse_hmc_func)cli_load_api("bhy_parse_hmc");
    bhy_parse_oc = (bhy_parse_oc_func)cli_load_api("bhy_parse_oc");
    bhy_parse_ec = (bhy_parse_ec_func)cli_load_api("bhy_parse_ec");

    /*bhy_add_accuracy_to_sensor_data_func bhy_add_accuracy_to_sensor_data = NULL; */
    bhy_set_downsampling_flag = (bhy_set_downsampling_flag_func)cli_load_api("bhy_set_downsampling_flag");
    bhy_get_downsampling_flag = (bhy_get_downsampling_flag_func)cli_load_api("bhy_get_downsampling_flag");
    bhy_set_downsampling_odr = (bhy_set_downsampling_odr_func)cli_load_api("bhy_set_downsampling_odr");
    bhy_parse_temperature_celsius = (bhy_parse_temperature_celsius_func)cli_load_api("bhy_parse_temperature_celsius");
    bhy_parse_humidity = (bhy_parse_humidity_func)cli_load_api("bhy_parse_humidity");
    bhy_parse_pressure = (bhy_parse_pressure_func)cli_load_api("bhy_parse_pressure");
    bhy_parse_altitude = (bhy_parse_altitude_func)cli_load_api("bhy_parse_altitude");
    bhy_parse_klio = (bhy_parse_klio_func)cli_load_api("bhy_parse_klio");
    bhy_parse_klio_generic = (bhy_parse_klio_generic_func)cli_load_api("bhy_parse_klio_generic");
    bhy_parse_klio_log = (bhy_parse_klio_log_func)cli_load_api("bhy_parse_klio_log");
    bhy_klio_param_read_reset_driver_status = (bhy_klio_param_read_reset_driver_status_func)cli_load_api(
        "bhy_klio_param_read_reset_driver_status");
    bhy_klio_param_read_pattern = (bhy_klio_param_read_pattern_func)cli_load_api("bhy_klio_param_read_pattern");
    bhy_klio_param_set_state = (bhy_klio_param_set_state_func)cli_load_api("bhy_klio_param_set_state");
    bhy_klio_param_set_generic_recognition_state = (bhy_klio_param_set_generic_recognition_state_func)cli_load_api(
        "bhy_klio_param_set_generic_recognition_state");
    bhy_klio_param_get_state = (bhy_klio_param_get_state_func)cli_load_api("bhy_klio_param_get_state");
    bhy_klio_param_get_generic_recognition_state = (bhy_klio_param_get_generic_recognition_state_func)cli_load_api(
        "bhy_klio_param_get_generic_recognition_state");
    bhy_klio_param_write_pattern = (bhy_klio_param_write_pattern_func)cli_load_api("bhy_klio_param_write_pattern");
    bhy_klio_param_write_generic_pattern = (bhy_klio_param_write_generic_pattern_func)cli_load_api(
        "bhy_klio_param_write_generic_pattern");
    bhy_klio_param_write_gesture_config = (bhy_klio_param_write_gesture_config_func)cli_load_api(
        "bhy_klio_param_write_gesture_config");
    bhy_klio_param_write_timing_config = (bhy_klio_param_write_timing_config_func)cli_load_api(
        "bhy_klio_param_write_timing_config");
    bhy_klio_param_set_pattern_states = (bhy_klio_param_set_pattern_states_func)cli_load_api(
        "bhy_klio_param_set_pattern_states");
    bhy_klio_param_similarity_score = (bhy_klio_param_similarity_score_func)cli_load_api(
        "bhy_klio_param_similarity_score");
    bhy_klio_param_similarity_score_multiple = (bhy_klio_param_similarity_score_multiple_func)cli_load_api(
        "bhy_klio_param_similarity_score_multiple");
    bhy_klio_param_set_parameter = (bhy_klio_param_set_parameter_func)cli_load_api("bhy_klio_param_set_parameter");
    bhy_klio_param_get_parameter = (bhy_klio_param_get_parameter_func)cli_load_api("bhy_klio_param_get_parameter");
    bhy_klio_param_reset = (bhy_klio_param_reset_func)cli_load_api("bhy_klio_param_reset");
    bhy_get_klio_info = (bhy_get_klio_info_func)cli_load_api("bhy_get_klio_info");
    bhy_set_klio_info = (bhy_set_klio_info_func)cli_load_api("bhy_set_klio_info");

    /*     //bhy_swim_param_algo_callback = NULL; */
    bhy_swim_param_get_config = (bhy_swim_param_get_config_func)cli_load_api("bhy_swim_param_get_config");
    bhy_swim_param_set_config = (bhy_swim_param_set_config_func)cli_load_api("bhy_swim_param_set_config");
    bhy_swim_param_get_version = (bhy_swim_param_get_version_func)cli_load_api("bhy_swim_param_get_version");
    bhy_swim_param_set_logging = (bhy_swim_param_set_logging_func)cli_load_api("bhy_swim_param_set_logging");
    bhy_parse_swim = (bhy_parse_swim_func)cli_load_api("bhy_parse_swim");
    bhy_get_swim_data = (bhy_get_swim_data_func)cli_load_api("bhy_get_swim_data");

    return BHY_OK;
}
