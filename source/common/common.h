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
 * @file    common.h
 * @brief   Common header file for the BHI260/BHA260 examples
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdbool.h>
#include "bhy_defs.h"
#include "coines.h"

#define BHY260_APP20_CS_PIN     COINES_SHUTTLE_PIN_7
#define BHY260_APP20_INT_PIN    COINES_SHUTTLE_PIN_21
#define BHY260_APP20_RESET_PIN  COINES_SHUTTLE_PIN_8
#define BHY260_APP30_CS_PIN     COINES_MINI_SHUTTLE_PIN_2_1
#define BHY260_APP30_INT_PIN    COINES_MINI_SHUTTLE_PIN_1_6
#define BHY260_APP30_RESET_PIN  COINES_MINI_SHUTTLE_PIN_2_6

#ifdef PC
#ifdef COINES_BRIDGE
#define BHY_RD_WR_LEN           256   /* Coines bridge maximum read write length */
#else
#define BHY_RD_WR_LEN           44    /* USB maximum read write length(DD firmware) */
#endif
#else
#define BHY_RD_WR_LEN           256   /* MCU maximum read write length */
#endif

/**
* @brief Function to get COINES error
* @param[in] rslt  : result value
* @return String corresponds to COINES error
*/
char *get_coines_error(int16_t rslt);

/**
* @brief Function to get API error
* @param[in] error_code  : Error code
* @return String corresponds to error code
*/
char *get_api_error(int8_t error_code);

/**
* @brief Function to get sensor error in text format
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_error_text(uint8_t sensor_error);

/**
* @brief Function to get sensor name
* @param[in] sensor_id  : Sensor ID
* @return String represents sensor name
*/
char *get_sensor_name(uint8_t sensor_id);

/**
* @brief Function to get physical sensor name
* @param[in] sensor_id  : Sensor ID
* @return String represents physical sensor name
*/
char *get_physical_sensor_name(uint8_t sensor_id);

/**
* @brief Function to get physical sensor ID
* @param[in] virt_sensor_id  : Virtual sensor ID
* @return Physical sensor ID
*/
uint8_t get_physical_sensor_id(uint8_t virt_sensor_id);

/**
* @brief Function to get dynamic scaling factor
* @param[in] sensor_id     : Sensor ID
* @param[in] dynamic_range : Dynamic range
* @return Physical sensor ID
*/
float get_sensor_dynamic_range_scaling(uint8_t sensor_id, float dynamic_range);

/**
* @brief Function to get sensor SI unit
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor SI unit
*/
char *get_sensor_si_unit(uint8_t sensor_id);

/**
* @brief Function to get sensor parse format
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor parse format
*/
char *get_sensor_parse_format(uint8_t sensor_id);

/**
* @brief Function to get sensor axis name
* @param[in] sensor_id : Sensor ID
* @return String represents sensor axis name
*/
char *get_sensor_axis_names(uint8_t sensor_id);

/**
* @brief Function to get KLIO error
* @param[in] error  : Error code
* @return String represents KLIO error
*/
char *get_klio_error(bhy_klio_param_driver_error_state_t error);

/**
* @brief Function to set up interface
* @param[in] reset_power : Option to enable/disable power reset
* @param[in] intf        : Type of interface
* @param[in] com_port    : COM port
*/
void setup_interfaces(bool reset_power, enum bhy_intf intf, const char *com_port);

/**
* @brief Function to close interface
* @param[in] intf : Type of interface
*/
void close_interfaces(enum bhy_intf intf);

/**
* @brief Function to read data via SPI
* @param[in] reg_addr  : Register address
* @param[out] reg_data : Pointer to register data
* @param[in] length    : Length of data to read
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
* @brief Function to write data via SPI
* @param[in] reg_addr  : Register address
* @param[in] reg_data  : Pointer to register data to write
* @param[in] length    : Length of data to write
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
* @brief Function to read data via I2C
* @param[in] reg_addr  : Register address
* @param[out] reg_data : Pointer to register data
* @param[in] length    : Length of data to read
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
* @brief Function to write data via I2C
* @param[in] reg_addr  : Register address
* @param[in] reg_data  : Pointer to register data to write
* @param[in] length    : Length of data to write
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
* @brief Function to introduce a delay in microseconds
* @param[in] us           : Number of time to delay (microseconds)
* @param[in] private_data : Pointer to private data
*/
void bhydev_delay_us(uint32_t us, void *private_data);

/**
* @brief Function to get interrupt status
* @return Interrupt status
*/
bool get_interrupt_status(void);

#endif /* _COMMON_H_ */
