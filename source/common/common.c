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
 * @file    common.c
 * @brief   Common source file for the BHy260 examples
 *
 */

/*lint -e750*/
#include "common.h"
#include "bhy_defs.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

#define BHA260_SHUTTLE_ID       0x139
#define BHI260_SHUTTLE_ID       0x119

#define ROBERT_BOSCH_USB_VID    (0x108C)
#define ARDUINO_USB_VID         (0x2341)
#define BST_APP31_CDC_USB_PID   (0xAB38)
#define BST_APP30_CDC_USB_PID   (0xAB3C)
#define BST_APP20_CDC_USB_PID   (0xAB2C)
#define BST_HEAR3X_CDC_USB_PID  (0x4B3C)
#define ARDUINO_NICLA_USB_PID   (0x0060)

#ifndef PC
static uint8_t verb_buff[256] = { 0 };
#endif

/**
* @brief Function to write verbose information
* @param[in] buffer : Pointer to buffer which stored information
* @param[in] length : Length of data
*/
void verbose_write(uint8_t *buffer, uint16_t length);

/**
* @brief Function to print information
* @param[in] format : Format specifier
*/
void PRINT(const char * restrict format, ...)
{
    va_list args = { 0 };

    va_start(args, format);

#ifndef PC
    (void)vsprintf((char *)verb_buff, format, args);
    verbose_write(verb_buff, (uint16_t)(strlen((char *)verb_buff)));
#else
    vfprintf(stdout, format, args);
#endif

    va_end(args);
}

#ifdef MCU_APP20
static enum coines_multi_io_pin cs_pin = BHY260_APP20_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY260_APP20_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY260_APP20_RESET_PIN;
#else
static enum coines_multi_io_pin cs_pin = BHY260_APP3X_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY260_APP3X_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY260_APP3X_RESET_PIN;
#endif

typedef struct
{
    int16_t key;
    char *value;
} bhy_str_lookup_entry;

/**
 * @brief Function to look up a string in a lookup table
 * @param[in] table        : Pointer to lookup table
 * @param[in] table_size   : Size of lookup table
 * @param[in] key          : Key to look up
 * @param[in] default_value: Default value if key is not found
 * @return Corresponding string if found, otherwise default_value
 */
static char *bhy_str_lut_lookup(const bhy_str_lookup_entry *table, size_t table_size, int16_t key, char *default_value)
{
    for (size_t i = 0; i < table_size; i++)
    {
        if (table[i].key == key)
        {
            return table[i].value;
        }
    }

    return default_value;
}

/**
* @brief Function to get sensor name from Accelerometer sensor ID
* @param[in] sensor_id  : Sensor ID
* @return String represents corresponding Accelerometer sensor
*/
char *get_accel_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry accel_sensor_lut[] = {
        { BHY_SENSOR_ID_ACC_PASS, "Accelerometer passthrough" },
        { BHY_SENSOR_ID_ACC_RAW, "Accelerometer uncalibrated" }, { BHY_SENSOR_ID_ACC, "Accelerometer corrected" },
        { BHY_SENSOR_ID_ACC_BIAS, "Accelerometer offset" }, { BHY_SENSOR_ID_ACC_WU, "Accelerometer corrected wake up" },
        { BHY_SENSOR_ID_ACC_RAW_WU, "Accelerometer uncalibrated wake up" },
        { BHY_SENSOR_ID_ACC_BIAS_WU, "Accelerometer offset wake up" },
    };

    return bhy_str_lut_lookup(accel_sensor_lut, sizeof(accel_sensor_lut) / sizeof(accel_sensor_lut[0]), sensor_id, " ");
}

/**
* @brief Function to get sensor name from Gyroscope sensor ID
* @param[in] sensor_id  : Sensor ID
* @return String represents corresponding Gyroscope sensor
*/
char *get_gyro_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry gyro_sensor_lut[] = {
        { BHY_SENSOR_ID_GYRO_PASS, "Gyroscope passthrough" }, { BHY_SENSOR_ID_GYRO_RAW, "Gyroscope uncalibrated" },
        { BHY_SENSOR_ID_GYRO, "Gyroscope corrected" }, { BHY_SENSOR_ID_GYRO_BIAS, "Gyroscope offset" },
        { BHY_SENSOR_ID_GYRO_WU, "Gyroscope wake up" }, { BHY_SENSOR_ID_GYRO_RAW_WU, "Gyroscope uncalibrated wake up" },
        { BHY_SENSOR_ID_GYRO_BIAS_WU, "Gyroscope offset wake up" },
    };

    return bhy_str_lut_lookup(gyro_sensor_lut, sizeof(gyro_sensor_lut) / sizeof(gyro_sensor_lut[0]), sensor_id, " ");
}

/**
* @brief Function to get sensor name from Magnetometer sensor ID
* @param[in] sensor_id  : Sensor ID
* @return String represents corresponding Magnetometer sensor
*/
char *get_mag_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry mag_sensor_lut[] = {
        { BHY_SENSOR_ID_MAG_PASS, "Magnetometer passthrough" }, { BHY_SENSOR_ID_MAG_RAW, "Magnetometer uncalibrated" },
        { BHY_SENSOR_ID_MAG, "Magnetometer corrected" }, { BHY_SENSOR_ID_MAG_BIAS, "Magnetometer offset" },
        { BHY_SENSOR_ID_MAG_WU, "Magnetometer wake up" },
        { BHY_SENSOR_ID_MAG_RAW_WU, "Magnetometer uncalibrated wake up" },
        { BHY_SENSOR_ID_MAG_BIAS_WU, "Magnetometer offset wake up" },
    };

    return bhy_str_lut_lookup(mag_sensor_lut, sizeof(mag_sensor_lut) / sizeof(mag_sensor_lut[0]), sensor_id, " ");
}

/**
* @brief Function to get sensor name from some Motion sensor IDs
* @param[in] sensor_id  : Sensor ID
* @return String represents corresponding Motion sensor
*/
char *get_motion_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry motion_sensor_lut[] = {
        { BHY_SENSOR_ID_GRA, "Gravity vector" }, { BHY_SENSOR_ID_GRA_WU, "Gravity vector wake up" },
        { BHY_SENSOR_ID_LACC, "Linear acceleration" }, { BHY_SENSOR_ID_LACC_WU, "Linear acceleration wake up" },
        { BHY_SENSOR_ID_RV, "Rotation vector" }, { BHY_SENSOR_ID_RV_WU, "Rotation vector wake up" },
        { BHY_SENSOR_ID_GAMERV, "Game rotation vector" }, { BHY_SENSOR_ID_GAMERV_WU, "Game rotation vector wake up" },
        { BHY_SENSOR_ID_GEORV, "Geo-magnetic rotation vector" },
        { BHY_SENSOR_ID_GEORV_WU, "Geo-magnetic rotation vector wake up" }, { BHY_SENSOR_ID_ORI, "Orientation" },
        { BHY_SENSOR_ID_ORI_WU, "Orientation wake up" }, { BHY_SENSOR_ID_SIG, "Significant motion" },
        { BHY_SENSOR_ID_STD, "Step detector" }, { BHY_SENSOR_ID_STD_WU, "Step detector wake up" },
        { BHY_SENSOR_ID_AIR_QUALITY, "Air Quality" }, { BHY_SENSOR_ID_TEMP, "Temperature" },
        { BHY_SENSOR_ID_BARO, "Barometer" }, { BHY_SENSOR_ID_HUM, "Humidity" }, { BHY_SENSOR_ID_GAS, "Gas" },
    };

    return bhy_str_lut_lookup(motion_sensor_lut,
                              sizeof(motion_sensor_lut) / sizeof(motion_sensor_lut[0]),
                              sensor_id,
                              " ");
}

/**
* @brief Function to get sensor name from all miscellaneous sensor IDs
* @param[in] sensor_id  : Sensor ID
* @return String represents corresponding sensor name
*/
char *get_misc_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry misc_sensor_lut[] = {
        { BHY_SENSOR_ID_SIG_LP_WU, "Low Power Significant motion wake up" },
        { BHY_SENSOR_ID_STD_LP, "Low Power Step detector" },
        { BHY_SENSOR_ID_STD_LP_WU, "Low Power Step detector wake up" },
        { BHY_SENSOR_BMP_TEMPERATURE_WU, "BMP Temperature wake up" },
        { BHY_SENSOR_ID_ANY_MOTION_LP_WU, "Low Power Any motion wake up" },
        { BHY_SENSOR_ID_NO_MOTION_LP_WU, "Low Power No Motion wake up" },
        { BHY_SENSOR_ID_AR_WEAR_WU, "Activity recognition wake up" },
        { BHY_SENSOR_ID_WRIST_WEAR_LP_WU, "Low Power Wrist Wear wake up" },
        { BHY_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, "Low Power Wrist Gesture wake up" },
        { BHY_SENSOR_ID_MULTI_TAP, "Multi Tap Detector" },
        { BHY_SENSOR_ID_HEAD_ORI_MIS_ALG, "Head Misalignment Calibrator" },
        { BHY_SENSOR_ID_IMU_HEAD_ORI_Q, "IMU Head Orientation Quaternion" },
        { BHY_SENSOR_ID_NDOF_HEAD_ORI_Q, "NDOF Head Orientation Quaternion" },
        { BHY_SENSOR_ID_IMU_HEAD_ORI_E, "IMU Head Orientation Euler" },
        { BHY_SENSOR_ID_NDOF_HEAD_ORI_E, "NDOF Head Orientation Euler" },
        { BHY_SENSOR_ID_HEAD_GESTURE, "Head Gesture" }, { BHY_SENSOR_ID_TEMP_WU, "Temperature wake up" },
        { BHY_SENSOR_ID_BARO_WU, "Barometer wake up" }, { BHY_SENSOR_ID_PRESSURE_WU, "BMP Pressure wake up" },
        { BHY_SENSOR_ID_PRESSURE, "BMP Pressure" }, { BHY_SENSOR_ID_HUM_WU, "Humidity wake up" },
        { BHY_SENSOR_ID_GAS_WU, "Gas wake up" }, { BHY_SENSOR_ID_KLIO, "Klio" },
        { BHY_SENSOR_ID_KLIO_GENERIC, "Klio Generic" }, { BHY_SENSOR_ID_KLIO_LOG, "Klio log" },
        { BHY_SENSOR_ID_STC_LP, "Low Power Step counter" },
        { BHY_SENSOR_ID_STC_LP_WU, "Low Power Step counter wake up" },
        { BHY_SENSOR_BMP_TEMPERATURE, "BMP Temperature" },
    };

    return bhy_str_lut_lookup(misc_sensor_lut, sizeof(misc_sensor_lut) / sizeof(misc_sensor_lut[0]), sensor_id, " ");

}

/**
* @brief Function to get some sensor axis name (part 1)
* @param[in] sensor_id : Sensor ID
* @return String represents sensor axis name
*/
char *get_sensor_axis_name(uint8_t sensor_id)
{
    char *ret = " ";

    if (((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_LACC_WU)) ||
        (sensor_id == BHY_SENSOR_ID_ACC_BIAS_WU) || (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = "x,y,z";
    }
    else if ((sensor_id >= BHY_SENSOR_ID_RV) && (sensor_id <= BHY_SENSOR_ID_GEORV_WU))
    {
        ret = "x,y,z,w,ar";
    }
    else if ((sensor_id == BHY_SENSOR_ID_ORI) || (sensor_id == BHY_SENSOR_ID_ORI_WU))
    {
        ret = "h,p,r";
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO)
    {
        ret = "lin,lid,lpr,lcr,rin,rid,rc,rsc";
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO_GENERIC)
    {
        ret = "gid,gsc,gc,fc,fsc";
    }

    return ret;
}

/**
* @brief Function to get interrupt status
* @return Interrupt status
*/
bool get_interrupt_status(void)
{
    int16_t coines_rslt;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_get_pin_config(int_pin, &pin_direction, &pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        PRINT("Error getting interrupt pin status\r\n.%s\r\n", get_coines_error(coines_rslt));
    }

    return pin_value == COINES_PIN_VALUE_HIGH;
}

/**
* @brief Function to get COINES error
* @param[in] rslt  : result value
* @return String corresponds to COINES error
*/
char *get_coines_error(int16_t rslt)
{
    static const bhy_str_lookup_entry coines_error_lut[] = {
        { COINES_SUCCESS, "" }, { COINES_E_FAILURE, "[COINES Error] Generic failure" },
        { COINES_E_COMM_IO_ERROR, "[COINES Error] Communication IO failed. Check connections with the sensor" },
        { COINES_E_COMM_INIT_FAILED, "[COINES Error] Communication initialization failed" },
        { COINES_E_UNABLE_OPEN_DEVICE, "[COINES Error] Unable to open device. Check if the board is in use" },
        { COINES_E_DEVICE_NOT_FOUND, "[COINES Error] Device not found. Check if the board is powered on" },
        { COINES_E_UNABLE_CLAIM_INTF, "[COINES Error] Unable to claim interface. Check if the board is in use" },
        { COINES_E_MEMORY_ALLOCATION, "[COINES Error] Error allocating memory" },
        { COINES_E_NOT_SUPPORTED, "[COINES Error] Feature not supported" },
        { COINES_E_NULL_PTR, "[COINES Error] Null pointer error" },
        { COINES_E_COMM_WRONG_RESPONSE, "[COINES Error] Unexpected response" },
        { COINES_E_SPI16BIT_NOT_CONFIGURED, "[COINES Error] 16-Bit SPI not configured" },
        { COINES_E_SPI_INVALID_BUS_INTF, "[COINES Error] Invalid SPI bus interface" },
        { COINES_E_SPI_CONFIG_EXIST, "[COINES Error] SPI already configured" },
        { COINES_E_SPI_BUS_NOT_ENABLED, "[COINES Error] SPI bus not enabled" },
        { COINES_E_SPI_CONFIG_FAILED, "[COINES Error] SPI configuration failed" },
        { COINES_E_I2C_INVALID_BUS_INTF, "[COINES Error] Invalid I2C bus interface" },
        { COINES_E_I2C_BUS_NOT_ENABLED, "[COINES Error] I2C bus not enabled" },
        { COINES_E_I2C_CONFIG_FAILED, "[COINES Error] I2C configuration failed" },
        { COINES_E_I2C_CONFIG_EXIST, "[COINES Error] I2C already configured" },
    };

    return bhy_str_lut_lookup(coines_error_lut,
                              sizeof(coines_error_lut) / sizeof(coines_error_lut[0]),
                              rslt,
                              "[COINES Error] Unknown error code");
}

/**
* @brief Function to get API error
* @param[in] error_code  : Error code
* @return String corresponds to error code
*/
char *get_api_error(int8_t error_code)
{
    static const bhy_str_lookup_entry api_error_lut[] = {
        { BHY_OK, "[API Error] No error" }, { BHY_E_NULL_PTR, "[API Error] Null pointer" },
        { BHY_E_INVALID_PARAM, "[API Error] Invalid parameter" }, { BHY_E_IO, "[API Error] IO error" },
        { BHY_E_MAGIC, "[API Error] Invalid firmware" }, { BHY_E_TIMEOUT, "[API Error] Timed out" },
        { BHY_E_BUFFER, "[API Error] Invalid buffer" }, { BHY_E_INVALID_FIFO_TYPE, "[API Error] Invalid FIFO type" },
        { BHY_E_INVALID_EVENT_SIZE, "[API Error] Invalid Event size" },
        { BHY_E_PARAM_NOT_SET, "[API Error] Parameter not set" },
        { BHY_E_INSUFFICIENT_MAX_SIMUL_SENSORS, "[API Error] Insufficient max simultaneous sensors" },
        { BHY_E_FUNCTION_NOT_FOUND, "[API Error] Function not found" },
    };

    return bhy_str_lut_lookup(api_error_lut,
                              sizeof(api_error_lut) / sizeof(api_error_lut[0]),
                              error_code,
                              "[API Error] Unknown API error code");
}

/**
* @brief Function to set up interface
* @param[in] reset_power : Option to enable/disable power reset
* @param[in] intf        : Type of interface
* @param[in] com_port    : COM port
*/
void setup_interfaces(bool reset_power, enum bhy_intf intf, const char *com_port)
{
    int16_t coines_rslt;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;
    struct coines_board_info board_info;

#ifndef PC
    (void)com_port;
    struct coines_ble_config ble_config;
    ble_config.name = "bhycli";
    ble_config.tx_power = COINES_TX_POWER_8_DBM;
    (void)coines_ble_config(&ble_config);
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);
#else
    struct coines_serial_com_config scom_config;

    if (com_port)
    {
        scom_config.baud_rate = 9600;
        scom_config.vendor_id = ROBERT_BOSCH_USB_VID;
        scom_config.product_id = BST_APP30_CDC_USB_PID;
        scom_config.com_port_name = strdup(com_port);
        scom_config.rx_buffer_size = 2048;
        coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, &scom_config);
        free(scom_config.com_port_name); /* free memory */
    }
    else
    {
        coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
    }

#endif
    if (coines_rslt)
    {
        PRINT("%s\n", get_coines_error(coines_rslt));
    }

    coines_rslt = coines_get_board_info(&board_info);
    if (coines_rslt == COINES_SUCCESS)
    {
#ifdef PC
        if (com_port)
        {
            if (board_info.board == 3) /* Application Board 2.0 */
            {
                scom_config.product_id = BST_APP20_CDC_USB_PID;
            }
            else if (board_info.board == 9) /* Application Board 3.1 */
            {
                scom_config.product_id = BST_APP31_CDC_USB_PID;
            }
            else if (board_info.board == 10) /* Hear 3x board*/
            {
                scom_config.product_id = BST_HEAR3X_CDC_USB_PID;
            }
        }

#endif
        if ((board_info.board == 5) || (board_info.board == 9)) /* Application Board 3.x */
        {
            cs_pin = BHY260_APP3X_CS_PIN;
            int_pin = BHY260_APP3X_INT_PIN;
            reset_pin = BHY260_APP3X_RESET_PIN;
        }
        else if (board_info.board == 10) /* Hear 3x board*/
        {
            cs_pin = COINES_HEARABLE_SHUTTLE_PIN_8;
            int_pin = COINES_HEARABLE_SHUTTLE_PIN_15;
            reset_pin = COINES_HEARABLE_SHUTTLE_PIN_1;
        }
    }
    else
    {
        PRINT("%s\r\n", get_coines_error(coines_rslt));
    }

    if (reset_power)
    {
        coines_rslt = coines_set_shuttleboard_vdd_vddio_config(0, 0);
        if (coines_rslt != COINES_SUCCESS)
        {
            PRINT("%s\r\n", get_coines_error(coines_rslt));
        }

        pin_direction = COINES_PIN_DIRECTION_OUT;
        pin_value = COINES_PIN_VALUE_LOW;
        coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
        if (coines_rslt != COINES_SUCCESS)
        {
            PRINT("%s\r\n", get_coines_error(coines_rslt));
        }

        coines_delay_msec(10);
    }

    if (intf == BHY_SPI_INTERFACE)
    {
        PRINT("Host Interface : SPI\r\n");
        coines_rslt = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE0);
        if (coines_rslt != COINES_SUCCESS)
        {
            PRINT("Error configuring to SPI.\r\n%s\r\n", get_coines_error(coines_rslt));
        }
    }
    else
    {
        PRINT("Host Interface : I2C\r\n");
        coines_rslt = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
        if (coines_rslt != COINES_SUCCESS)
        {
            PRINT("Error configuring to I2C.\r\n%s\r\n", get_coines_error(coines_rslt));
        }
    }

    coines_rslt = coines_set_shuttleboard_vdd_vddio_config(1800, 1800);
    if (coines_rslt != COINES_SUCCESS)
    {
        PRINT("Error setting Vdd and Vddio to 1.8V.\r\n%s\r\n", get_coines_error(coines_rslt));
    }

    pin_direction = COINES_PIN_DIRECTION_OUT;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        PRINT("Error setting the reset pin\r\n.%s\r\n", get_coines_error(coines_rslt));
    }

    /* Configure as a pull-down. The BHy260 operates the interrupt pin as an active high, level, push-pull by default */
    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_LOW;
    coines_rslt = coines_set_pin_config(int_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        PRINT("Error configuring the interrupt pin\r\n.%s\r\n", get_coines_error(coines_rslt));
    }

    coines_delay_msec(50);
}

/**
* @brief Function to close interface
* @param[in] intf : Type of interface
*/
void close_interfaces(enum bhy_intf intf)
{
    if (intf == BHY_I2C_INTERFACE)
    {
        (void)coines_deconfig_i2c_bus(COINES_I2C_BUS_0);
    }
    else
    {
        (void)coines_deconfig_spi_bus(COINES_SPI_BUS_0);
    }

    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    (void)fflush(stdout);

    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(100);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(100);
}

/**
* @brief Function to read data via SPI
* @param[in] reg_addr  : Register address
* @param[out] reg_data : Pointer to register data
* @param[in] length    : Length of data to read
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, reg_data, (uint16_t)length);
}

/**
* @brief Function to write data via SPI
* @param[in] reg_addr  : Register address
* @param[in] reg_data  : Pointer to register data to write
* @param[in] length    : Length of data to write
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

/**
* @brief Function to read data via I2C
* @param[in] reg_addr  : Register address
* @param[out] reg_data : Pointer to register data
* @param[in] length    : Length of data to read
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, reg_data, (uint16_t)length);
}

/**
* @brief Function to write data via I2C
* @param[in] reg_addr  : Register address
* @param[in] reg_data  : Pointer to register data to write
* @param[in] length    : Length of data to write
* @param[in] intf_ptr  : Pointer to interface
* @return API error codes
*/
int8_t bhydev_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

/**
* @brief Function to introduce a delay in microseconds
* @param[in] us           : Number of time to delay (microseconds)
* @param[in] private_data : Pointer to private data
*/
void bhydev_delay_us(uint32_t us, void *private_data)
{
    (void)private_data;
    coines_delay_usec(us);
}

/**
* @brief Function to get sensor error related to bootloader
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_bootloader_error_text(uint8_t sensor_error)
{
    static const bhy_str_lookup_entry bootloader_error_lut[] = {
        { 0x00, "No error" }, { 0x1A, "[Sensor error] ROM Version Mismatch" },
        { 0x10, "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch" },
        { 0x11, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC" },
        { 0x12, "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch" },
        { 0x13, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC" },
        { 0x14, "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed" },
        { 0x15, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC" },
        { 0x16, "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required" },
        { 0x17, "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing" },
        { 0x19, "[Sensor error] Bootloader reports: Unexpected Watchdog Reset" },
        { 0x1B, "[Sensor error] Bootloader reports: Fatal Firmware Error" },
        { 0x1F, "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid" },
        { 0x44, "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available" },
        { 0xC0, "[Sensor error] Bootloader reports: Command Error" },
        { 0xC1, "[Sensor error] Bootloader reports: Command Too Long" },
        { 0xC2, "[Sensor error] Bootloader reports: Command Buffer Overflow" },
    };

    return bhy_str_lut_lookup(bootloader_error_lut,
                              sizeof(bootloader_error_lut) / sizeof(bootloader_error_lut[0]),
                              sensor_error,
                              " ");
}

/**
* @brief Function to get some sensor errors in text format
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_errors_text(uint8_t sensor_error)
{
    static const bhy_str_lookup_entry sensor_error_lut[] = {
        { 0x77, "[Sensor error] Host Download Channel Empty" }, { 0x78, "[Sensor error] DMA Error" },
        { 0x79, "[Sensor error] Corrupted Input Block Chain" }, { 0x7A, "[Sensor error] Corrupted Output Block Chain" },
        { 0x7B, "[Sensor error] Buffer Block Manager Error" },
        { 0x7C, "[Sensor error] Input Channel Not Word Aligned" }, { 0x7D, "[Sensor error] Too Many Flush Events" },
        { 0x7E, "[Sensor error] Unknown Host Channel Error" }, { 0x81, "[Sensor error] Decimation Too Large" },
        { 0x90, "[Sensor error] Master SPI/I2C Queue Overflow" }, { 0x91, "[Sensor error] SPI/I2C Callback Error" },
        { 0xA0, "[Sensor error] Timer Scheduling Error" }, { 0xB0, "[Sensor error] Invalid GPIO for Host IRQ" },
        { 0xB1, "[Sensor error] Error Sending Initialized Meta Events" },
        { 0xD0, "[Sensor error] User Mode Error: Sys Call Invalid" },
        { 0xD1, "[Sensor error] User Mode Error: Trap Invalid" },
        { 0xE1, "[Sensor error] Firmware Upload Failed: Firmware header corrupt" },
        { 0xE2, "[Sensor error] Sensor Data Injection: Invalid input stream" },
    };

    return bhy_str_lut_lookup(sensor_error_lut,
                              sizeof(sensor_error_lut) / sizeof(sensor_error_lut[0]),
                              sensor_error,
                              " ");
}

/**
* @brief Function to get sensor Algorithm errors
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_algo_error_text(uint8_t sensor_error)
{
    static const bhy_str_lookup_entry sensor_algo_error_lut[] = {
        { 0x2D, "[Sensor error] Firmware Too Large" }, { 0x2F, "[Sensor error] Invalid RAM Banks" },
        { 0x30, "[Sensor error] Math Error" }, { 0x40, "[Sensor error] Memory Error" },
        { 0x41, "[Sensor error] SWI3 Error" }, { 0x42, "[Sensor error] SWI4 Error" },
        { 0x43, "[Sensor error] Illegal Instruction Error" }, { 0x45, "[Sensor error] Invalid Memory Access" },
        { 0x50, "[Sensor error] Algorithm Error: BSX Init" }, { 0x51, "[Sensor error] Algorithm Error: BSX Do Step" },
        { 0x52, "[Sensor error] Algorithm Error: Update Sub" }, { 0x53, "[Sensor error] Algorithm Error: Get Sub" },
        { 0x54, "[Sensor error] Algorithm Error: Get Phys" },
        { 0x55, "[Sensor error] Algorithm Error: Unsupported Phys Rate" },
        { 0x56, "[Sensor error] Algorithm Error: Cannot find BSX Driver" },
    };

    return bhy_str_lut_lookup(sensor_algo_error_lut,
                              sizeof(sensor_algo_error_lut) / sizeof(sensor_algo_error_lut[0]),
                              sensor_error,
                              " ");
}

/**
* @brief Function to get sensor Self-Test errors
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_self_test_error_text(uint8_t sensor_error)
{
    static const bhy_str_lookup_entry sensor_self_test_error_lut[] = {
        { 0x60, "[Sensor error] Sensor Self-Test Failure" }, { 0x61, "[Sensor error] Sensor Self-Test X Axis Failure" },
        { 0x62, "[Sensor error] Sensor Self-Test Y Axis Failure" },
        { 0x64, "[Sensor error] Sensor Self-Test Z Axis Failure" }, { 0x65, "[Sensor error] FOC Failure" },
        { 0x66, "[Sensor error] Sensor Busy" }, { 0x6F, "[Sensor error] Self-Test or FOC Test Unsupported" },
        { 0x72, "[Sensor error] No Host Interrupt Set" },
        { 0x73, "[Sensor error] Event ID Passed to Host Interface Has No Known Size" },
        { 0x75, "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)" },
        { 0x76, "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)" },
    };

    return bhy_str_lut_lookup(sensor_self_test_error_lut,
                              sizeof(sensor_self_test_error_lut) / sizeof(sensor_self_test_error_lut[0]),
                              sensor_error,
                              " ");
}

/**
* @brief Function to get miscellaneous sensor errors
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_misc_error_text(uint8_t sensor_error)
{
    static const bhy_str_lookup_entry sensor_misc_error_lut[] = {
        { 0x1C, "[Sensor error] Chained Firmware Error: Next Payload Not Found" },
        { 0x1D, "[Sensor error] Chained Firmware Error: Payload Not Valid" },
        { 0x1E, "[Sensor error] Chained Firmware Error: Payload Entries Invalid" },
        { 0x20, "[Sensor error] Firmware Init Failed" },
        { 0x21, "[Sensor error] Sensor Init Failed: Unexpected Device ID" },
        { 0x22, "[Sensor error] Sensor Init Failed: No Response from Device" },
        { 0x23, "[Sensor error] Sensor Init Failed: Unknown" }, { 0x24, "[Sensor error] Sensor Error: No Valid Data" },
        { 0x25, "[Sensor error] Slow Sample Rate" }, { 0x26, "[Sensor error] Data Overflow (saturated sensor data)" },
        { 0x27, "[Sensor error] Stack Overflow" }, { 0x28, "[Sensor error] Insufficient Free RAM" },
        { 0x29, "[Sensor error] Sensor Init Failed: Driver Parsing Error" },
        { 0x2A, "[Sensor error] Too Many RAM Banks Required" }, { 0x2B, "[Sensor error] Invalid Event Specified" },
        { 0x2C, "[Sensor error] More than 32 On Change" },
    };

    return bhy_str_lut_lookup(sensor_misc_error_lut,
                              sizeof(sensor_misc_error_lut) / sizeof(sensor_misc_error_lut[0]),
                              sensor_error,
                              "[Sensor error] Unknown error code");
}

/**
* @brief Function to get sensor error in text format
* @param[in] sensor_error  : Sensor code
* @return String corresponds to sensor error
*/
char *get_sensor_error_text(uint8_t sensor_error)
{
    char *ret;

    ret = get_sensor_bootloader_error_text(sensor_error);

    if ((strcmp(ret, " ") == 0) && (sensor_error != 0x00))
    {
        ret = get_sensor_errors_text(sensor_error);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_algo_error_text(sensor_error);

            if (strcmp(ret, " ") == 0)
            {
                ret = get_sensor_self_test_error_text(sensor_error);

                if (strcmp(ret, " ") == 0)
                {
                    ret = get_sensor_misc_error_text(sensor_error);
                }
            }
        }
    }

    return ret;
}

/**
* @brief Function to get physical sensor name
* @param[in] sensor_id  : Sensor ID
* @return String represents physical sensor name
*/
char *get_physical_sensor_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry physical_sensor_names[] = {
        { BHY_PHYS_SENSOR_ID_ACCELEROMETER, "Accelerometer" }, { BHY_PHYS_SENSOR_ID_GYROSCOPE, "Gyroscope" },
        { BHY_PHYS_SENSOR_ID_MAGNETOMETER, "Magnetometer" }, { BHY_PHYS_SENSOR_ID_TEMP_GYRO, "Temperature Gyroscope" },
        { BHI360_PHYS_SENSOR_ID_BME_TEMP, "BME Temperature" }, { BHY_PHYS_SENSOR_ID_POSITION, "Position" },
        { BHY_PHYS_SENSOR_ID_HUMIDITY, "Humidity" }, { BHY_PHYS_SENSOR_ID_TEMPERATURE, "Temperature" },
        { BHY_PHYS_SENSOR_ID_GAS_RESISTOR, "Gas Resistor" },
        { BHY_PHYS_SENSOR_ID_MAGNETOMETER_DUMMY, "Magnetometer dummy" },
        { BHY_PHYS_SENSOR_ID_PHYS_STEP_COUNTER, "Step Counter" },
        { BHY_PHYS_SENSOR_ID_PHYS_STEP_DETECTOR, "Step Detector" },
        { BHY_PHYS_SENSOR_ID_PHYS_SIGN_MOTION, "Significant Motion" },
        { BHY_PHYS_SENSOR_ID_PHYS_ANY_MOTION, "Any Motion" }, { BHI360_PHYS_SENSOR_ID_FEATURE_CORE, "Feature Core" },
        { BHY_PHYS_SENSOR_ID_GPS, "GPS" }, { BHY_PHYS_SENSOR_ID_LIGHT, "Light" },
        { BHY_PHYS_SENSOR_ID_PROXIMITY, "Proximity" }, { BHY_PHYS_SENSOR_ID_ACT_REC, "Activity Recognition" },
        { BHY_PHYS_SENSOR_ID_PHYS_NO_MOTION, "No Motion" },
        { BHY_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT, "Wrist Gesture Detector" },
        { BHY_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP, "Wrist Wear Wakeup" },
        { BHY_PHYS_SENSOR_ID_BMP_TEMPERATURE, "BMP Temperature" }, { BHY_PHYS_SENSOR_ID_BMP_PRESSURE, "BMP Pressure" },
    };

    return bhy_str_lut_lookup(physical_sensor_names,
                              sizeof(physical_sensor_names) / sizeof(physical_sensor_names[0]),
                              sensor_id,
                              "Undefined physical sensor ID");
}

/**
* @brief Function to get physical sensor ID
* @param[in] virt_sensor_id  : Virtual sensor ID
* @return Physical sensor ID
*/
uint8_t get_physical_sensor_id(uint8_t virt_sensor_id)
{
    uint8_t ret;

    if ((virt_sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (virt_sensor_id <= BHY_SENSOR_ID_ACC_RAW_WU))
    {
        ret = BHY_PHYS_SENSOR_ID_ACCELEROMETER;
    }
    else if (((virt_sensor_id >= BHY_SENSOR_ID_GYRO_PASS) && (virt_sensor_id <= BHY_SENSOR_ID_GYRO_RAW_WU)) ||
             (virt_sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = BHY_PHYS_SENSOR_ID_GYROSCOPE;
    }
    else if (((virt_sensor_id >= BHY_SENSOR_ID_MAG_PASS) && (virt_sensor_id <= BHY_SENSOR_ID_MAG_RAW_WU)) ||
             (virt_sensor_id == BHY_SENSOR_ID_MAG_BIAS_WU))
    {
        ret = BHY_PHYS_SENSOR_ID_MAGNETOMETER;
    }
    else
    {
        ret = BHY_PHYS_SENSOR_ID_NOT_SUPPORTED;
    }

    return ret;
}

/**
* @brief Function to get sensor name
* @param[in] sensor_id  : Sensor ID
* @return String represents sensor name
*/
char *get_sensor_name(uint8_t sensor_id)
{
    char *ret;

    ret = get_accel_sensor_name(sensor_id);

    if (strcmp(ret, " ") != 0)
    {
        return ret;
    }

    ret = get_gyro_sensor_name(sensor_id);

    if (strcmp(ret, " ") != 0)
    {
        return ret;
    }

    ret = get_mag_sensor_name(sensor_id);

    if (strcmp(ret, " ") != 0)
    {
        return ret;
    }

    ret = get_motion_sensor_name(sensor_id);

    if (strcmp(ret, " ") != 0)
    {
        return ret;
    }

    ret = get_misc_sensor_name(sensor_id);

    if (strcmp(ret, " ") != 0)
    {
        return ret;
    }

    if ((sensor_id >= BHY_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY_SENSOR_ID_CUSTOM_END))
    {
        ret = "Custom sensor ID ";
    }
    else
    {
        ret = "Undefined sensor ID ";
    }

    return ret;
}

/**
* @brief Function to get dynamic scaling factor
* @param[in] sensor_id     : Sensor ID
* @param[in] dynamic_range : Dynamic range
* @return Physical sensor ID
*/
float get_sensor_dynamic_range_scaling(uint8_t sensor_id, float dynamic_range)
{
    float scaling = -1.0f;

    if (((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_ACC_RAW_WU)) ||
        ((sensor_id >= BHY_SENSOR_ID_GYRO_PASS) && (sensor_id <= BHY_SENSOR_ID_GYRO_RAW_WU)) ||
        (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU) ||
        ((sensor_id >= BHY_SENSOR_ID_MAG_PASS) && (sensor_id <= BHY_SENSOR_ID_MAG_RAW_WU)) ||
        (sensor_id == BHY_SENSOR_ID_MAG_BIAS_WU))
    {
        scaling = dynamic_range / 32768.0f;
    }
    else
    {
        printf("Sensor ID not supported for dynamic range scaling\r\n");
        scaling = -1.0f; /* Do not apply the scaling factor */
    }

    return scaling;
}

/**
* @brief Function to get sensor SI unit
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor SI unit
*/
char *get_sensor_si_unit(uint8_t sensor_id)
{
    char *ret = " ";

    if ((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_ACC_RAW_WU))
    {
        ret = "Earth g-s";
    }
    else if (((sensor_id >= BHY_SENSOR_ID_GYRO_PASS) && (sensor_id <= BHY_SENSOR_ID_GYRO_RAW_WU)) ||
             (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = "degrees/second";
    }
    else if (((sensor_id >= BHY_SENSOR_ID_MAG_PASS) && (sensor_id <= BHY_SENSOR_ID_MAG_RAW_WU)) ||
             (sensor_id == BHY_SENSOR_ID_MAG_BIAS_WU))
    {
        ret = "microtesla";
    }
    else
    {
        ret = "";
    }

    return ret;
}

/**
* @brief Function to get some sensor parse format (part 1)
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor parse format
*/
char *get_sensor_parse_format_text(uint8_t sensor_id)
{
    char *ret = " ";

    if ((sensor_id == BHY_SENSOR_ID_SI_ACCEL) || (sensor_id == BHY_SENSOR_ID_SI_GYROS))
    {
        ret = "f,f,f";
    }
    else if (((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_LACC_WU)) ||
             (sensor_id == BHY_SENSOR_ID_ACC_BIAS_WU) || (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = "s16,s16,s16";
    }
    else if ((sensor_id >= BHY_SENSOR_ID_RV) && (sensor_id <= BHY_SENSOR_ID_GEORV_WU))
    {
        ret = "s16,s16,s16,s16,u16";
    }
    else if ((sensor_id == BHY_SENSOR_ID_ORI) || (sensor_id == BHY_SENSOR_ID_ORI_WU))
    {
        ret = "s16,s16,s16";
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO)
    {
        ret = "u8,u8,u8,u8,u8,u8,f,f";
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO_GENERIC)
    {
        ret = "u8,u8,f,u8,f";
    }
    else if ((sensor_id == BHY_SENSOR_ID_LIGHT) || (sensor_id == BHY_SENSOR_ID_LIGHT_WU))
    {
        ret = "s16";
    }

    return ret;
}

/**
* @brief Function to get some sensor parse format (part 2)
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor parse format
*/
char *get_sensor_parse_format_data(uint8_t sensor_id)
{
    char *ret = " ";

    if ((sensor_id == BHY_SENSOR_ID_DEVICE_ORI) || (sensor_id == BHY_SENSOR_ID_DEVICE_ORI_WU) ||
        (sensor_id == BHY_SENSOR_ID_HUM) || (sensor_id == BHY_SENSOR_ID_HUM_WU) || (sensor_id == BHY_SENSOR_ID_PROX) ||
        (sensor_id == BHY_SENSOR_ID_PROX_WU) || (sensor_id == BHY_SENSOR_ID_EXCAMERA) ||
        (sensor_id == BHY_SENSOR_ID_MULTI_TAP) || (sensor_id == BHY_SENSOR_ID_HEAD_GESTURE))
    {
        ret = "u8";
    }
    else if ((sensor_id == BHY_SENSOR_ID_TEMP) || (sensor_id == BHY_SENSOR_ID_TEMP_WU) ||
             (sensor_id == BHY_SENSOR_BMP_TEMPERATURE) || (sensor_id == BHY_SENSOR_BMP_TEMPERATURE_WU))
    {
        ret = "s16";
    }
    else if ((sensor_id == BHY_SENSOR_ID_GAS) || (sensor_id == BHY_SENSOR_ID_GAS_WU) ||
             (sensor_id == BHY_SENSOR_ID_STC) || (sensor_id == BHY_SENSOR_ID_STC_WU) ||
             (sensor_id == BHY_SENSOR_ID_STC_LP) ||
             (sensor_id == BHY_SENSOR_ID_STC_LP_WU))
    {
        ret = "u32";
    }

    return ret;
}

/**
* @brief Function to get some sensor parse format (part 3)
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor parse format
*/
char *get_sensor_parse_format_rep(uint8_t sensor_id)
{
    char *ret = " ";

    if ((sensor_id == BHY_SENSOR_ID_TILT_DETECTOR) || (sensor_id == BHY_SENSOR_ID_STD) ||
        ((sensor_id >= BHY_SENSOR_ID_SIG) && (sensor_id <= BHY_SENSOR_ID_PICKUP_GESTURE)) ||
        (sensor_id == BHY_SENSOR_ID_STD_WU) || (sensor_id == BHY_SENSOR_ID_SIG_LP_WU) ||
        (sensor_id == BHY_SENSOR_ID_STD_LP) || (sensor_id == BHY_SENSOR_ID_STD_LP_WU) ||
        (sensor_id == BHY_SENSOR_ID_WRIST_TILT_GESTURE) || (sensor_id == BHY_SENSOR_ID_STATIONARY_DET) ||
        (sensor_id == BHY_SENSOR_ID_ANY_MOTION_LP_WU) || (sensor_id == BHY_SENSOR_ID_NO_MOTION_LP_WU) ||
        (sensor_id == BHY_SENSOR_ID_MOTION_DET) || (sensor_id == BHY_SENSOR_ID_WRIST_WEAR_LP_WU))
    {
        ret = "";
    }
    else if ((sensor_id == BHY_SENSOR_ID_AR) || (sensor_id == BHY_SENSOR_ID_AR_WEAR_WU))
    {
        ret = "u16";
    }
    else if (sensor_id == BHY_SENSOR_ID_GPS)
    {
        ret = "st";
    }

    return ret;
}

/**
* @brief Function to check if the sensor id output is of the barometer
* @param[in] sensor_id : Sensor ID
* @return true if sensor is barometer else false
*/
static bool is_baro_sensor(uint8_t sensor_id)
{
    return (sensor_id == BHY_SENSOR_ID_BARO) || (sensor_id == BHY_SENSOR_ID_BARO_WU) ||
           (sensor_id == BHY_SENSOR_ID_PRESSURE) || (sensor_id == BHY_SENSOR_ID_PRESSURE_WU);
}

/**
* @brief Function to get sensor parse format
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor parse format
*/
char *get_sensor_parse_format(uint8_t sensor_id)
{
    char *ret;

    ret = get_sensor_parse_format_text(sensor_id);

    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_parse_format_data(sensor_id);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_parse_format_rep(sensor_id);

            if (strcmp(ret, " ") == 0)
            {
                if (is_baro_sensor(sensor_id))
                {
                    ret = "u24";
                }
                else if (sensor_id == BHY_SENSOR_ID_WRIST_GEST_DETECT_LP_WU)
                {
                    ret = "u8";
                }
                else if (sensor_id == BHY_SENSOR_ID_AIR_QUALITY)
                {
                    ret = "f,f,f,f,f,f,f,u8";
                }
                else if ((sensor_id == BHY_SENSOR_ID_HEAD_ORI_MIS_ALG) || (sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_Q) ||
                         (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_Q))
                {
                    ret = "s16,s16,s16,s16";
                }
                else if ((sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_E) || (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_E))
                {
                    ret = "s16,s16,s16";
                }
                else if ((sensor_id == BHY_SENSOR_ID_GAS) || (sensor_id == BHY_SENSOR_ID_GAS_WU))
                {
                    ret = "u32";
                }
                else if ((sensor_id == BHY_SENSOR_ID_STC) || (sensor_id == BHY_SENSOR_ID_STC_WU) ||
                         (sensor_id == BHY_SENSOR_ID_STC_LP) || (sensor_id == BHY_SENSOR_ID_STC_LP_WU) ||
                         (sensor_id == BHY_SENSOR_ID_EXCAMERA))
                {
                    ret = "c";
                }
                else
                {
                    ret = "";
                }
            }
        }
    }

    return ret;
}

/**
* @brief Function to get some sensor axis name (part 2)
* @param[in] sensor_id : Sensor ID
* @return String represents sensor axis name
*/
char *get_sensor_axes_name(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry sensor_axes_name_lut[] = {
        { BHY_SENSOR_ID_TEMP, "t" }, { BHY_SENSOR_ID_TEMP_WU, "t" }, { BHY_SENSOR_BMP_TEMPERATURE, "t" },
        { BHY_SENSOR_BMP_TEMPERATURE_WU, "t" }, { BHY_SENSOR_ID_TILT_DETECTOR, "e" }, { BHY_SENSOR_ID_STD, "e" },
        { BHY_SENSOR_ID_SIG, "e" }, { BHY_SENSOR_ID_WAKE_GESTURE, "e" }, { BHY_SENSOR_ID_GLANCE_GESTURE, "e" },
        { BHY_SENSOR_ID_PICKUP_GESTURE, "e" }, { BHY_SENSOR_ID_STD_WU, "e" }, { BHY_SENSOR_ID_SIG_LP_WU, "e" },
        { BHY_SENSOR_ID_STD_LP, "e" }, { BHY_SENSOR_ID_STD_LP_WU, "e" }, { BHY_SENSOR_ID_WRIST_TILT_GESTURE, "e" },
        { BHY_SENSOR_ID_STATIONARY_DET, "e" }, { BHY_SENSOR_ID_ANY_MOTION_LP_WU, "e" },
        { BHY_SENSOR_ID_NO_MOTION_LP_WU, "e" }, { BHY_SENSOR_ID_MOTION_DET, "e" },
        { BHY_SENSOR_ID_WRIST_WEAR_LP_WU, "e" },
    };

    return bhy_str_lut_lookup(sensor_axes_name_lut,
                              sizeof(sensor_axes_name_lut) / sizeof(sensor_axes_name_lut[0]),
                              sensor_id,
                              " ");
}

/**
* @brief Function to get some sensor axis name (part 3)
* @param[in] sensor_id : Sensor ID
* @return String represents sensor axis name
*/
char *get_sensor_axis_name_format(uint8_t sensor_id)
{
    static const bhy_str_lookup_entry sensor_axis_name_format_lut[] = {
        { BHY_SENSOR_ID_AR, "a" }, { BHY_SENSOR_ID_AR_WEAR_WU, "a" },
        { BHY_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, "wrist_gesture" }, { BHY_SENSOR_ID_MULTI_TAP, "taps" },
        { BHY_SENSOR_ID_AIR_QUALITY, "t,h,g,i,si,c,v,a" }, { BHY_SENSOR_ID_HEAD_ORI_MIS_ALG, "x,y,z,w" },
        { BHY_SENSOR_ID_IMU_HEAD_ORI_Q, "x,y,z,w" }, { BHY_SENSOR_ID_NDOF_HEAD_ORI_Q, "x,y,z,w" },
        { BHY_SENSOR_ID_IMU_HEAD_ORI_E, "h,p,r" }, { BHY_SENSOR_ID_NDOF_HEAD_ORI_E, "h,p,r" },
        { BHY_SENSOR_ID_BARO, "p" }, { BHY_SENSOR_ID_BARO_WU, "p" }, { BHY_SENSOR_ID_PRESSURE, "p" },
        { BHY_SENSOR_ID_PRESSURE_WU, "p" }, { BHY_SENSOR_ID_HUM, "h" }, { BHY_SENSOR_ID_HUM_WU, "h" },
        { BHY_SENSOR_ID_HEAD_GESTURE, "g" }, { BHY_SENSOR_ID_STC, "sc" }, { BHY_SENSOR_ID_STC_WU, "sc" },
        { BHY_SENSOR_ID_STC_LP, "sc" }, { BHY_SENSOR_ID_STC_LP_WU, "sc" },
    };

    return bhy_str_lut_lookup(sensor_axis_name_format_lut,
                              sizeof(sensor_axis_name_format_lut) / sizeof(sensor_axis_name_format_lut[0]),
                              sensor_id,
                              " ");
}

/**
* @brief Function to get sensor axis name
* @param[in] sensor_id     : Sensor ID
* @return String represents sensor axis name
*/
char *get_sensor_axis_names(uint8_t sensor_id)
{
    char *ret;

    ret = get_sensor_axis_name(sensor_id);

    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_axes_name(sensor_id);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_axis_name_format(sensor_id);
        }
    }

    return ret;
}

char *get_klio_error(bhy_klio_param_driver_error_state_t error)
{
    static const bhy_str_lookup_entry klio_error_codes[] = {
        { BHY_KLIO_DRIVER_ERROR_NONE, "No error" }, { BHY_KLIO_DRIVER_ERROR_INVALID_PARAMETER, "Invalid parameter" },
        { BHY_KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE, "Parameter out of range" },
        { BHY_KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION, "Invalid pattern operation" },
        { BHY_KLIO_DRIVER_ERROR_NOT_IMPLEMENTED, "Not implemented" }, { BHY_KLIO_DRIVER_ERROR_BUFSIZE, "Buffer size" },
        { BHY_KLIO_DRIVER_ERROR_INTERNAL, "Internal" }, { BHY_KLIO_DRIVER_ERROR_UNDEFINED, "Undefined" },
        { BHY_KLIO_DRIVER_ERROR_OPERATION_PENDING, "Operation pending" },
    };

    return bhy_str_lut_lookup(klio_error_codes,
                              sizeof(klio_error_codes) / sizeof(klio_error_codes[0]),
                              error,
                              "Unknown error code");
}

#ifndef PC

/**
* @brief Default function to write verbose information
* @param[in] buffer : Pointer to buffer which stored information
* @param[in] length : Length of data
*/
void default_verbose_write(uint8_t *buffer, uint16_t length)
{
    coines_write_intf(COINES_COMM_INTF_USB, buffer, length);
}

#endif
