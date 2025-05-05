/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    bhycli_callbacks.c
 * @brief   Source file for the command line utility callbacks
 *
 */

#define BHY2CLI_VER_MAJOR       "0"
#define BHY2CLI_VER_MINOR       "5"
#define BHY2CLI_VER_BUGFIX      "1"

#ifdef __STDC_ALLOC_LIB__
#define __STDC_WANT_LIB_EXT2__  1
#endif

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE         200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <ctype.h>
#include <sys/stat.h>
#include <math.h>
#if defined(PC)
#include <dirent.h>
#endif

#include "coines.h"
#include "bhycli_callbacks.h"
#include "common_callbacks.h"
#include "common.h"
#include "parse.h"
#include "verbose.h"
#include "dinject.h"
#include "post_mortem.h"
#include "bhy_system_param.h"
#include "bhy_virtual_sensor_conf_param.h"
#include "bhy_activity_param.h"
#include "bhy_param_defs.h"
#include "bhy_bsx_algo_param.h"
#include "bhy_bsec_param.h"
#include "bhy_virtual_sensor_info_param.h"

#define BHYCLI_MAX_STRING_LENGTH  UINT16_C(32)

#define MAX_FILENAME_LENGTH       255

/* Contains all parameters of the of custom virtual sensors
 * required for parsing */
typedef struct BHY_PACKED custom_driver_information
{
    char sensor_name[BHYCLI_MAX_STRING_LENGTH];
    uint16_t sensor_payload : 7;
    uint16_t sensor_id : 8;
    uint16_t is_registered : 1;
    char output_formats[BHYCLI_MAX_STRING_LENGTH];
} custom_driver_information_t;

/* *INDENT-OFF* */
/* Contains information about klio capabilities and current state, as well as runtime configuration */
/*
static struct
{
    uint8_t max_cyclic_patterns;
    uint8_t max_cyclic_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
    uint8_t max_generic_patterns;
    uint16_t max_generic_pattern_blob_size;
}klio_vars;
*/
/* *INDENT-ON* */

static int8_t assert_rslt;
static uint8_t fifo_buffer[2048];
static bool sensors_active[256] = { false };
static bool first_run = true;

/* Added to report the final swim data to mobile APP when swim is disabled */
/*static bhy_event_data_swim_output_t swim_data; */

/* Static global table that contains the payloads of present custom virtual sensors, derived by a parameter read */
static custom_driver_information_t custom_driver_information[(BHY_SENSOR_ID_CUSTOM_END - BHY_SENSOR_ID_CUSTOM_START) +
                                                             1];

/* static bool klio_enabled = false; */

struct data_inject dinject;
static struct bhy_virtual_sensor_conf_param_conf sensor_conf;

/**
* @brief Function to convert input string to integer
* @param[in] str : Input string
* @return Integer corresponds to string
*/
uint32_t string_to_int(const char *str)
{
    char int_str[32] = { 0 };

    strncpy(int_str, str, strlen(str));
    int_str[strlen(str)] = '\0';

    return (uint32_t)strtol(int_str, NULL, 0);
}

/* BEGIN GenAI Github Copilot */

/**
* @brief Function to checks if a file exists
* @param[in] file_name : The name of the file to check.
* @return 1 if the file exists, 0 otherwise.
*/
uint8_t file_exists(const char *file_name)
{
    FILE *file;

    /* Try to open the file in read mode */
    file = fopen(file_name, "r");
    if (file != NULL)
    {
        /* File exists, close the file */
        fclose(file);

        return 1;
    }

    /* File does not exist */
    return 0;
}

/* END GenAI Github Copilot */

static cli_callback_table_t bhy_cli_callbacks[] = {
    { 0, "", 0, NULL, NULL }, /* Empty characters creates a new line */
    { 'h', "help", 0, help_callback, help_help }, /* Print all available commands */
    { 0, "version", 0, version_callback, version_help }, /* Prints the HW, SW versions and build date*/
    { 'v', "verb", 1, verbose_callback, verb_help }, /* Change verbose of received outputs */
    { 'b', "ramb", 1, ramb_callback, ramb_help }, /* Reset, Load firmware to RAM and boot */
    { 'n', "reset", 0, reset_callback, reset_help }, /* Trigger a soft reset to the sensor */
    { 'a', "addse", 1, addse_callback, addse_help }, /* Add a custom sensor */
    { 'g', "boot", 1, boot_callback, boot_help }, /* Boot from RAM or Flash */
    { 'c', "actse", 1, actse_callback, actse_help }, /* Activate/De-activate a sensor */
    { 0, "schema", 0, schema_callback, schema_help }, /* Get schema information of the loaded sensors */
    { 0, "hexse", 1, hexse_callback, hexse_help }, /* Stream sensor data in hex */
    { 0, "dactse", 0, dactse_callback, dactse_help }, /* Deactivate all the active sensors */
    { 0, "lsactse", 0, lsactse_callback, lsactse_help }, /* List all the active sensors */
    { 0, "dmode", 1, dmode_callback, dmode_help }, /* Switch to Data Injection mode */
    { 0, "dinject", 1, dinject_callback, dinject_help }, /* Compute virtual sensor output from raw IMU data */
    #if 0
    { 'e', "erase", 0, erase_callback, erase_help }, /* Erase the external Flash */
    { 'd', "flb", 1, flb_callback, flb_help }, /* Reset, Load firmware to Flash and boot */
    { 0, "efd", 0, efd_callback, efd_help }, /* Erase the Flash descriptor */
    { 'f', "fl", 1, fl_callback, fl_help }, /* Upload firmware to Flash */
    #endif
    { 'r', "rd", 1, rd_callback, rd_help }, /* Read registers */
    { 'u', "ram", 1, ram_callback, ram_help }, /* Upload firmware to RAM */
    { 'w', "wr", 1, wr_callback, wr_help }, /* Write registers */
    { 'i', "info", 0, info_callback, info_help }, /* Get information of the state of the device and loaded sensors */
    { 's', "rdp", 1, rdp_callback, rdp_help }, /* Read a parameter */
    { 't', "wrp", 1, wrp_callback, wrp_help }, /* Write a parameter */
    { 'p', "physeninfo", 1, physeninfo_callback, physeninfo_help }, /* Read Physical Sensor Information */
    { 'm', "postm", 1, pm_callback, pm_help }, /* Get Post Mortem Data */
    { 0, "logse", 1, logse_callback, logse_help }, /* Log sensor data in binary */
    { 0, "attlog", 1, attlog_callback, attlog_help }, /* Attach a log file for logging */
    { 0, "detlog", 1, detlog_callback, detlog_help }, /* Detach a log file for logging */
    { 0, "setvirtsenconf", 3, setvirtsenconf_callback, setvirtsenconf_help }, /* Setting virtual sensor configuration */
    { 0, "getvirtsenconf", 1, getvirtsenconf_callback, getvirtsenconf_help }, /* Getting virtual sensor configuration */

    #if 0
    { 0, "kstatus", 0, kstatus_callback, kstatus_help }, /* Get Klio status */
    { 0, "ksetstate", 4, ksetstate_callback, ksetstate_help }, /* Set state of cyclic Klio */
    { 0, "kgetstate", 0, kgetstate_callback, kgetstate_help }, /* Get state of cyclic Klio */
    { 0, "ksetgstate", 1, ksetgstate_callback, ksetgstate_help }, /* Set state of generic Klio */
    { 0, "kgetgstate", 0, kgetgstate_callback, kgetgstate_help }, /* Get state of generic Klio */
    { 0, "kreset", 0, kreset_callback, kreset_help }, /* Reset all Klio state */
    { 0, "ksetgestconf", 2, ksetgestconf_callback, ksetgestconf_help }, /* Set config for generic Klio gesture */
    { 0, "ksettimeconf", 1, ksettimeconf_callback, ksettimeconf_help }, /* Set expected timing for gestures */
    { 0, "kldpatt", 2, kldpatt_callback, kldpatt_help }, /* Load Klio pattern for cyclic recognition */
    { 0, "kldgpatt", 2, kldgpatt_callback, kldgpatt_help }, /* Load Klio pattern for generic recognition */
    { 0, "kenpatt", 1, kenpatt_callback, kenpatt_help }, /* Enable cyclic Klio pattern */
    { 0, "kdispatt", 1, kdispatt_callback, kdispatt_help }, /* Disable cyclic Klio pattern */
    { 0, "kdisapatt", 1, kdisapatt_callback, kdisapatt_help }, /* Disable cyclic Klio adaptive pattern */
    { 0, "kswpatt", 1, kswpatt_callback, kswpatt_help }, /* Switch cyclic Klio pattern between left/right hand */
    { 0, "kautldpatt", 2, kautldpatt_callback, kautldpatt_help }, /* Auto-load cyclic Klio patterns */
    { 0, "kgetparam", 1, kgetparam_callback, kgetparam_help }, /* Get cyclic Klio parameters */
    { 0, "ksetparam", 2, ksetparam_callback, ksetparam_help }, /* Set cyclic Klio parameters */
    { 0, "ksimscore", 2, ksimscore_callback, ksimscore_help }, /* Get cyclic Klio Similarity score */
    { 0, "kmsimscore", 2, kmsimscore_callback, kmsimscore_help }, /* Get Multiple cyclic Klio Similarity score */
    { 0, "swim", 3, swim_callback, swim_help }, /* Configure the Swim recognition */
    { 0, "swimver", 0, swimver_callback, swimver_help }, /* Get the Swim Version */
    { 0, "swimgetfreq", 0, swimgetfreq_callback, swimgetfreq_help }, /* Get the Swim frequency */
    { 0, "swimsetfreq", 2, swimsetfreq_callback, swimsetfreq_help }, /* Set the Swim frequency */
    { 0, "swimgetaxes", 0, swimgetaxes_callback, swimgetaxes_help }, /* Get the Swim orientation sensor */
    { 0, "swimsetaxes", 1, swimsetaxes_callback, swimsetaxes_help }, /* Set the Swim orientation sensor */
    { 0, "swimsetlogging", 1, swimsetlogging_callback, swimsetlogging_help }, /* Set the Swim logging sensor */
    #endif
    { 0, "mtapen", 1, mtapen_callback, mtapen_help }, /* Enable/Disable Multi Tap Sensor */
    { 0, "mtapinfo", 0, mtapinfo_callback, mtapinfo_help }, /* Get Multi Tap Sensor Info */
    { 0, "mtapsetcnfg", 3, mtapsetcnfg_callback, mtapsetcnfg_help }, /* Set the Multi Tap Configuration */
    { 0, "mtapgetcnfg", 0, mtapgetcnfg_callback, mtapgetcnfg_help }, /* Get the Multi Tap Configuration */
    { 0, "accsetfoc", 3, accsetfoc_callback, accsetfoc_help }, /* Set the Accelerometer Fast Offset Calibration */
    { 0, "accgetfoc", 0, accgetfoc_callback, accgetfoc_help }, /* Get the Accelerometer Fast Offset Calibration */
    { 0, "accsetpwm", 1, accsetpwm_callback, accsetpwm_help }, /* Set the Accelerometer Power Mode */
    { 0, "accgetpwm", 0, accgetpwm_callback, accgetpwm_help }, /* Get the Accelerometer Power Mode */
    { 0, "accsetar", 6, accsetar_callback, accsetar_help }, /* Set the Accelerometer axis remapping */
    { 0, "accgetar", 0, accgetar_callback, accgetar_help }, /* Set the Accelerometer axis remapping */
    { 0, "acctrignvm", 0, acctrignvm_callback, acctrignvm_help }, /* Trigger a NVM writing for Accelerometer */
    { 0, "accgetnvm", 0, accgetnvm_callback, accgetnvm_help }, /* Get NVM writing status for Accelerometer */
    { 0, "gyrosetfoc", 3, gyrosetfoc_callback, gyrosetfoc_help }, /* Set the Gyroscope Fast Offset Calibration */
    { 0, "gyrogetfoc", 0, gyrogetfoc_callback, gyrogetfoc_help }, /* Get the Gyroscope Fast Offset Calibration */
    { 0, "gyrosetois", 1, gyrosetois_callback, gyrosetois_help }, /* Set the Gyroscope OIS Mode*/
    { 0, "gyrogetois", 0, gyrogetois_callback, gyrogetois_help }, /* Get the Gyroscope OIS Mode*/
    { 0, "gyrosetfs", 1, gyrosetfs_callback, gyrosetfs_help }, /* Set the Gyroscope Fast Startup Mode */
    { 0, "gyrogetfs", 0, gyrogetfs_callback, gyrogetfs_help }, /* Get the Gyroscope Fast Startup Mode*/
    { 0, "gyrosetcrt", 0, gyrosetcrt_callback, gyrosetcrt_help }, /* Set the Gyroscope CRT state*/
    { 0, "gyrogetcrt", 0, gyrogetcrt_callback, gyrogetcrt_help }, /* Get the Gyroscope CRT status*/
    { 0, "gyrosetpwm", 1, gyrosetpwm_callback, gyrosetpwm_help }, /* Set the Gyroscope Power Mode */
    { 0, "gyrogetpwm", 0, gyrogetpwm_callback, gyrogetpwm_help }, /* Get the Gyroscope Power Mode */
    { 0, "gyrosettat", 1, gyrosettat_callback, gyrosettat_help }, /* Set the Gyroscope Timer Auto Trim state*/
    { 0, "gyrogettat", 0, gyrogettat_callback, gyrogettat_help }, /* Get the Gyroscope Timer Auto Trim status*/
    { 0, "gyrotrignvm", 0, gyrotrignvm_callback, gyrotrignvm_help }, /* Trigger a NVM writing for Gyroscope */
    { 0, "gyrogetnvm", 0, gyrogetnvm_callback, gyrogetnvm_help }, /* Get NVM writing status for Gyroscope */
    { 0, "magsetpwm", 1, magsetpwm_callback, magsetpwm_help }, /* Set the Magnetometer Power Mode */
    { 0, "maggetpwm", 0, maggetpwm_callback, maggetpwm_help }, /* Get the Magnetometer Power Mode */
    { 0, "wwwsetcnfg", 8, wwwsetcnfg_callback, wwwsetcnfg_help }, /* Set the Wrist Wear Wakeup Configuration */
    { 0, "wwwgetcnfg", 0, wwwgetcnfg_callback, wwwgetcnfg_help }, /* Get the Wrist Wear Wakeup Configuration */
    { 0, "amsetcnfg", 1, amsetcnfg_callback, amsetcnfg_help }, /* Set the Any Motion Configuration */
    { 0, "amgetcnfg", 0, amgetcnfg_callback, amgetcnfg_help }, /* Get the Any Motion Configuration */
    { 0, "nmsetcnfg", 1, nmsetcnfg_callback, nmsetcnfg_help }, /* Set the No Motion Configuration */
    { 0, "nmgetcnfg", 0, nmgetcnfg_callback, nmgetcnfg_help }, /* Get the No Motion Configuration */
    { 0, "wgdsetcnfg", 10, wgdsetcnfg_callback, wgdsetcnfg_help }, /* Set the Wrist Gesture Detection Configuration */
    { 0, "wgdgetcnfg", 0, wgdgetcnfg_callback, wgdgetcnfg_help }, /* Get the Wrist Gesture Detection Configuration */
    { 0, "baro1setcnfg", 3, baro1setcnfg_callback, baro1setcnfg_help }, /* Set the Barometer pressure type 1
                                                                         * Configuration */
    { 0, "baro1getcnfg", 0, baro1getcnfg_callback, baro1getcnfg_help }, /* Get the Barometer pressure type 1
                                                                         * Configuration */
    { 0, "baro2setcnfg", 5, baro2setcnfg_callback, baro2setcnfg_help }, /* Set the Barometer pressure type 2
                                                                         * Configuration */
    { 0, "baro2getcnfg", 0, baro2getcnfg_callback, baro2getcnfg_help }, /* Get the Barometer pressure type 2
                                                                         * Configuration */
    { 0, "scsetcnfg", 27, scsetcnfg_callback, scsetcnfg_help }, /* Set the Step Counter Configuration */
    { 0, "scgetcnfg", 0, scgetcnfg_callback, scgetcnfg_help }, /* Get the Step Counter Configuration */
    { 0, "hmctrig", 0, hmctrig_callback, hmctrig_help }, /* Trigger Head Misalignment Calibration */
    { 0, "hmcsetcnfg", 4, hmcsetcnfg_callback, hmcsetcnfg_help }, /* Set Head Misalignment Configuration */
    { 0, "hmcgetcnfg", 0, hmcgetcnfg_callback, hmcgetcnfg_help }, /* Get Head Misalignment Configuration */
    { 0, "hmcsetdefcnfg", 0, hmcsetdefcnfg_callback, hmcsetdefcnfg_help }, /* Set Default Head Misalignment
                                                                            * Configuration */
    { 0, "hmcver", 0, hmcver_callback, hmcver_help }, /* Get Head Misalignment Calibrator Version */
    { 0, "hmcsetcalcorrq", 4, hmcsetcalcorrq_callback, hmcsetcalcorrq_help }, /* Set Head Misalignment Quaternion
                                                                               * Calibration Correction */
    { 0, "hmcgetcalcorrq", 0, hmcgetcalcorrq_callback, hmcgetcalcorrq_help }, /* Get Head Misalignment Quaternion
                                                                               * Calibration Correction */
    { 0, "hmcsetmode", 4, hmcsetmode_callback, hmcsetmode_help }, /* Set Misalignment Mode and Vector X value */
    { 0, "hmcgetmode", 0, hmcgetmode_callback, hmcgetmode_help }, /* Get Misalignment Mode and Vector X value */
    { 0, "hosetheadcorrq", 1, hosetheadcorrq_callback, hosetheadcorrq_help }, /* Get Head Orientation
                                                                                        * Quaternion Initial Head
                                                                                        * Correction */
    { 0, "hogetheadcorrq", 0, hogetheadcorrq_callback, hogetheadcorrq_help }, /* Get Head Orientation
                                                                                        * Quaternion Initial Head
                                                                                        * Correction */
    { 0, "hover", 0, hover_callback, hover_help }, /* Get Head Orientation Version */
    { 0, "hosetheadcorre", 1, hosetheadcorre_callback, hosetheadcorre_help }, /* Get Head Orientation Euler
                                                                                     * Initial Head Correction */
    { 0, "hogetheadcorre", 0, hogetheadcorre_callback, hogetheadcorre_help }, /* Get Head Orientation Euler Initial Head
                                                                               * Correction */
    { 0, "foc", 1, foc_callback, foc_help }, /* Set FOC configuration */
    { 0, "chipid", 0, getchipid_callback, getchipid_help }, /* Gets chip id of the sensor */
    { 0, "syssetphyseninfo", 0, syssetphyseninfo_callback, syssetphyseninfo_help }, /* Set system param physical sensor
                                                                                     * info */
    { 0, "sysgetphysenlist", 0, sysgetphysenlist_callback, sysgetphysenlist_help }, /* Get list of physical sensor */
    { 0, "sysgetvirsenlist", 0, sysgetvirtsenlist_callback, sysgetvirtsenlist_help }, /* Get list of physical sensor */
    { 0, "sysgettimestamps", 0, sysgettimestamps_callback, sysgettimestamps_help }, /* Get system timestamps */
    { 0, "sysgetfwversion", 0, sysgetfwversion_callback, sysgetfwversion_help }, /* Get system firmware version */
    { 0, "sysgetfifoctrl", 0, sysgetfifoctrl_callback, sysgetfifoctrl_help }, /* Get fifo control */
    { 0, "syssetwkffctrl", 1, syssetwkffctrl_callback, syssetwkffctrl_help }, /* Set watermark for wake-up fifo control
                                                                               * */
    { 0, "syssetnwkffctrl", 1, syssetnwkffctrl_callback, syssetnwkffctrl_help }, /* Set watermark for nonwake-up fifo
                                                                                  * control */
    { 0, "sysgetmectrl", 1, sysgetmectrl_callback, sysgetmectrl_help }, /* Get meta event control  for
                                                                         * wake-up/nonwake-up fifo control */
    { 0, "syssetmectrl", 3, syssetmectrl_callback, syssetmectrl_help }, /* Set meta event control  for
                                                                         * wake-up/nonwake-up fifo control */
    { 0, "bsecsetalstate", 163, bsecsetalstate_callback, bsecsetalstate_help }, /* Sets BSEC algorithm state */
    { 0, "bsecgetalstate", 0, bsecgetalstate_callback, bsecgetalstate_help }, /* Gets BSEC algorithm state */
    { 0, "bsecsettempoff", 1, bsecsettempoff_callback, bsecsettempoff_help }, /* Sets BSEC temperature offset */
    { 0, "bsecgettempoff", 0, bsecgettempoff_callback, bsecgettempoff_help }, /* Gets BSEC temperature offset */
    { 0, "bsecsetsamrate", 1, bsecsetsamrate_callback, bsecsetsamrate_help }, /* Sets BSEC sample rate */
    { 0, "bsecgetsamrate", 0, bsecgetsamrate_callback, bsecgetsamrate_help }, /* Gets BSEC sample rate */
    { 0, "sethearactvcnfg", 6, sethearactvcnfg_callback, sethearactvcnfg_help },
    { 0, "gethearactvcnfg", 0, gethearactvcnfg_callback, gethearactvcnfg_help },
    { 0, "setwearactvcnfg", 5, setwearactvcnfg_callback, setwearactvcnfg_help },
    { 0, "getwearactvcnfg", 0, getwearactvcnfg_callback, getwearactvcnfg_help },
    { 0, "virtseinfo", 1, virtseinfo_callback, virtseinfo_help }, /* Gets virtual sensor information
                                                                                  * parameters */
    { 0, "setbsxparam", 1, setbsxparam_callback, setbsxparam_help }, /* Set bsx algo calibration
                                                                                        * states */
    { 0, "getbsxparam", 1, getbsxparam_callback, getbsxparam_help }, /* Get bsx algo calibration
                                                                                        * states */
    { 0, "getbsxver", 0, getbsxver_callback, getbsxver_help }, /* Set bsx algo SIC matrix */
    { 0, "phyrangeconf", 2, phyrangeconf_callback, phyrangeconf_help }, /* Set physical range configuration */
    { 0, "logandstream", 1, logandstream_callback, logandstream_help }, /* Set logging and streaming together */
#ifndef PC
    { 0, "echo", 1, echo_callback, echo_help }, /* Toggle the echo setting */
    { 0, "heart", 1, heartbeat_callback, heartbeat_help }, /* Toggle the heartbeat message setting */
    { 0, "mklog", 1, mklog_callback, mklog_help }, /* Make a log file */
    { 0, "rm", 1, rm_callback, rm_help }, /* Remove a file */
    { 0, "ls", 0, ls_callback, ls_help }, /* List files */
    { 0, "wrfile", 2, wrfile_callback, wrfile_help }, /* Write content to a file */
    { 0, "rdfile", 1, rdfile_callback, rdfile_help }, /* Read content from a file */
    { 0, "slabel", 1, slabel_callback, slabel_help }, /* Write a binary label into the log file */
    { 0, "cls", 0, cls_callback, cls_help }, /* Clear screen */
    { 0, "strbuf", 1, streambuff_callback, streambuff_help }, /* Enable streaming buffer */
#endif
};

/**
* @brief Function to reset Fuser2 core
* @param[in] bhy : Device instance
*/
static void reset_hub(struct bhy_dev *bhy);

/**
* @brief Function to upload firmware to RAM
* @param[in] filepath : Path to firmware file
* @param[in] bhy      : Device instance
* @return Status for uploading
*/
static bool upload_to_ram(const char *filepath, struct bhy_dev *bhy);

/**
* @brief Function to print boot status
* @param[in] boot_status : Boot status value
*/
static void print_boot_status(uint8_t boot_status);

/**
* @brief Function to boot from RAM
* @param[in] bhy : Device instance
*/
static void boot_ram(struct bhy_dev *bhy);

/**
* @brief Function to print information about state of device and sensors
* @param[in] bhy : Device instance
*/
static void show_info(struct bhy_dev *bhy);

#if 0

/**
* @brief Function to boot from Flash
* @param[in] bhy : Device instance
*/
static void boot_flash(struct bhy_dev *bhy);

/**
* @brief Function to upload firmware to Flash
* @param[in] filepath : Path to firmware file
* @param[in] bhy      : Device instance
* @return Status for uploading
*/
static bool upload_to_flash(const char *filepath, struct bhy_dev *bhy);

#endif

/**
* @brief Function to write value to register
* @param[in] payload : Data to write, including register address
* @param[in] bhy     : Device instance
*/
static void wr_regs(const char *payload, struct bhy_dev *bhy);

/**
* @brief Function to read value from register
* @param[in] payload : Data includes register address and length to read
* @param[in] bhy     : Device instance
*/
static void rd_regs(const char *payload, struct bhy_dev *bhy);

/**
* @brief Function to read value from parameter
* @param[in] payload : Data includes parameter ID
* @param[in] bhy     : Device instance
*/
static void rd_param(const char *payload, struct bhy_dev *bhy);

/**
* @brief Function to write value to parameter
* @param[in] payload : Data to write, including parameter ID
* @param[in] bhy     : Device instance
*/
static void wr_param(const char *payload, struct bhy_dev *bhy);

/**
* @brief Function to read physical sensor information
* @param[in] payload : Data includes sensor ID
* @param[in] bhy     : Device instance
*/
static void rd_phy_sensor_info(const char *payload, struct bhy_dev *bhy);

#if 0

/**
* @brief Function to erase flash
* @param[in] end_addr : End address
* @param[in] bhy      : Device instance
*/
static void erase_flash(uint32_t end_addr, struct bhy_dev *bhy);

#endif

/**
* @brief Function to configure sensor
* @param[in] sen_cfg    : Sensor configuration
* @param[in] sen_id     : Sensor ID
* @param[in] parse_flag : Parse flag
* @param[in] ref        : Reference to device and command line
*/
static void configure_sensor(struct bhy_virtual_sensor_conf_param_conf sen_cfg,
                             uint8_t sen_id,
                             uint8_t parse_flag,
                             struct bhy_cli_ref *ref);

/**
* @brief Function to activate sensor
* @param[in] sensor_parameters : Sensor parameters
* @param[in] parse_flag        : Parse flag
* @param[in] ref               : Reference to device and command line
*/
static void activate_sensor(const char *sensor_parameters, uint8_t parse_flag, struct bhy_cli_ref *ref);

/**
* @brief Default function to parse for custom sensor
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_custom_sensor_default(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for custom sensor
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_custom_sensor(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to add a sensor
* @param[in] payload : Data contains sensor information
* @param[in] ref     : Reference to device and command line
*/
static void add_sensor(const char *payload, struct bhy_cli_ref *cli_ref);

#if 0

/**
* @brief Function to enable Klio
* @param[in] bhy : Device instance
*/
static void klio_enable(struct bhy_dev *bhy);

/**
* @brief Function to get Klio status
* @param[in] bhy : Device instance
*/
static void klio_status(struct bhy_dev *bhy);

/**
* @brief Function to set state of cyclic Klio
* @param[in] arg1 : Argument for learning status
* @param[in] arg2 : Argument for learning reset
* @param[in] arg3 : Argument for recognition status
* @param[in] arg4 : Argument for recognition reset
* @param[in] bhy  : Device instance
*/
static void klio_set_state(const char *arg1, const char *arg2, const char *arg3, const char *arg4,
                           struct bhy_dev *bhy);

/**
* @brief Function to get state of cyclic Klio
* @param[in] bhy  : Device instance
*/
static void klio_get_state(struct bhy_dev *bhy);

/**
* @brief Function to set state of generic recognition
* @param[in] arg  : Argument for gestures status
* @param[in] bhy  : Device instance
*/
static void klio_set_generic_recognition_state(const char *arg, struct bhy_dev *bhy);

/**
* @brief Function to get state of generic recognition
* @param[in] bhy  : Device instance
*/
static void klio_get_generic_recognition_state(struct bhy_dev *bhy);

/**
* @brief Function to load cyclic pattern
* @param[in] arg1 : Argument for pattern index
* @param[in] arg2 : Argument for pattern/adaptive pattern
* @param[in] bhy  : Device instance
*/
static void klio_load_cyclic_pattern(const char *arg1, const char *arg2, struct bhy_dev *bhy);

/**
* @brief Function to load generic pattern
* @param[in] arg1 : Argument for pattern index
* @param[in] arg2 : Argument for pattern as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_load_generic_pattern(const char *arg1, const char *arg2, struct bhy_dev *bhy);

/**
* @brief Function to set gesture configuration
* @param[in] arg1 : Argument for configuration index
* @param[in] arg2 : Argument for configuration as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_set_gesture_config(const char *arg1, const char *arg2, struct bhy_dev *bhy);

/**
* @brief Function to set timing configuration
* @param[in] arg : Argument for configuration as bare hex bytestring
* @param[in] bhy : Device instance
*/
static void klio_set_timing_config(const char *arg, struct bhy_dev *bhy);

/**
* @brief Function to get Klio parameter
* @param[in] arg : Argument for parameter ID
* @param[in] bhy : Device instance
*/
static void klio_get_parameter(const uint16_t *arg, struct bhy_dev *bhy);

/**
* @brief Function to set Klio parameter
* @param[in] arg1 : Argument for parameter ID
* @param[in] arg2 : Argument for value to set
* @param[in] bhy  : Device instance
*/
static void klio_set_parameter(const char *arg1, char *arg2, struct bhy_dev *bhy);

/**
* @brief Function to get Klio Similarity score
* @param[in] arg1 : Argument for first pattern as bare hex bytestring
* @param[in] arg2 : Argument for second pattern as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_similarity_score(const uint8_t *arg1, const uint8_t *arg2, struct bhy_dev *bhy);

/**
* @brief Function to get Multiple Klio Similarity score
* @param[in] arg1 : Argument for base index
* @param[in] arg2 : Argument for comparison indices
* @param[in] bhy  : Device instance
*/
static void klio_similarity_score_multiple(const char *arg1, const char *arg2, struct bhy_dev *bhy);

/**
* @brief Function to control pattern operation
* @param[in] operation : Argument for pattern operation
* @param[in] arg1      : Argument for pattern indices
* @param[in] bhy       : Device instance
*/
static void klio_pattern_state_operation(const uint8_t operation, const char *arg1, struct bhy_dev *bhy);

/**
* @brief Function to parse cyclic Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse generic Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio_generic(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse log Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio_log(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for swim
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_swim(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

#endif

/**
* @brief Function to parse for Accelerometer and Gyroscope sensors
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_acc_gyro(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_multitap(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_wrist_gesture_detect(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Air quality
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_air_quality(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_hmc(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_oc(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_ec(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to log sensor data
* @param[in] sid           : Sensor ID
* @param[in] tns           : Timestamp in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
* @param[in] logdev        : Device instance for log
*/
static void log_data(uint8_t sid,
                     uint64_t tns,
                     uint8_t event_size,
                     const uint8_t *event_payload,
                     struct logbin_dev *logdev);

/**
* @brief Function to trigger FOC
* @param[in] payload : Sensor ID
* @param[in] bhy     : Device instance
*/
static void trigger_foc(const char *payload, struct bhy_dev *bhy);

/**
* @brief Function to write meta information
* @param[in] log : Device instance for log
* @param[in] bhy : Device instance
*/
static void write_meta_info(struct logbin_dev *log, struct bhy_dev *bhy);

/**
* @brief Function to stream HEX data
* @param[in] sid           : Sensor ID
* @param[in] ts            : Timestamp in seconds
* @param[in] tns           : Timestamp in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
*/
static void stream_hex_data(uint8_t sid, uint32_t ts, uint32_t tns, uint8_t event_size, const uint8_t *event_payload);

/**
* @brief Function to print schema information
* @param[in] bhy : Device instance
*/
static void schema_info(struct bhy_dev *bhy);

/**
* @brief Function to get default scaling factor for some sensors
* @param[in] sensor_id : Sensor ID
* @param[in] bhy       : Device instance
*/
static float get_sensor_default_scaling_value(uint8_t sensor_id, struct bhy_dev *bhy);

/**
* @brief Function to get default scaling factor for all sensors
* @param[in] sensor_id : Sensor ID
* @param[in] bhy       : Device instance
*/
static float get_sensor_default_scaling(uint8_t sensor_id, struct bhy_dev *bhy);

/**
* @brief Function to print BSX Parameter information
* @param[in] bsx_state : Pointer to BSX state structure
*/
static void print_bsx_algo_param_states(bhy_bsx_algo_param_state_exg *bsx_state);

/**
* @brief Function to get all command lines callback
* @return Reference to callback table
*/
cli_callback_table_t *bhy_get_cli_callbacks(void)
{
    return bhy_cli_callbacks;
}

/**
* @brief Function to get number of command line callbacks
* @return Number of command line callbacks
*/
uint8_t bhy_get_n_cli_callbacks(void)
{
    return sizeof(bhy_cli_callbacks) / sizeof(cli_callback_table_t);
}

/**
* @brief Function to initialize interface and sensor callbacks
* @param[in] cli_ref  : Reference to command line
*/
void bhy_callbacks_init(struct bhy_cli_ref *cli_ref)
{
    struct bhy_dev *bhy = &cli_ref->bhy;
    uint8_t expected_data;
    uint8_t chip_id;
    uint8_t feat_status;

    for (uint8_t i = BHY_SENSOR_ID_CUSTOM_START;
         i <= BHY_SENSOR_ID_CUSTOM_END; i++)
    {
        custom_driver_information[i - BHY_SENSOR_ID_CUSTOM_START].is_registered = 0;
        strcpy(custom_driver_information[i - BHY_SENSOR_ID_CUSTOM_START].sensor_name, "Undefined custom sensor");
    }

    /* Print Copyright build date */
    PRINT("Copyright (c) 2024 Bosch Sensortec GmbH\r\n");
    PRINT("Version %s.%s.%s Build date: " __DATE__ "\r\n", BHY2CLI_VER_MAJOR, BHY2CLI_VER_MINOR, BHY2CLI_VER_BUGFIX);
#ifdef BHY_USE_I2C
    (void)(bhy_init(BHY_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, bhy));
#else
    (void)(bhy_init(BHY_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, bhy));
#endif

    /* Install virtual sensor callbacks */
    bhy_install_callbacks(&cli_ref->bhy, &cli_ref->parse_table);

    coines_delay_msec(100); /* Wait for flash firmware to load if applicable */

    (void)(bhy_get_chip_id(&chip_id, bhy));

    switch (chip_id)
    {
        #if 0
        case BHY_CHIP_ID_BHI260AP_BHI260AB_0:
        case BHY_CHIP_ID_BHI260AP_BHI260AB_1:
        #endif
        case BHI3_CHIP_ID_BHI360:
        #if 0
        case BHI3_CHIP_ID_BHI380:
        case BHY_CHIP_ID_BHA260AB_0:
        case BHY_CHIP_ID_BHA260AB_1:
        #endif
            (void)(bhy_get_feature_status(&feat_status, bhy));
            PRINT("Device found. Chip ID read 0x%X\r\n", chip_id);
            if (feat_status & BHY_FEAT_STATUS_OPEN_RTOS_MSK)
            {
                INFO("RTOS based firmware running\r\n");

                (void)(bhy_update_virtual_sensor_list(bhy));
            }

            break;
        default:
            ERROR("Device not found, Check connections and power. Chip ID read 0x%x\r\n", chip_id);
#ifdef PC
            exit(1);
#endif
    }

    /* Config status channel */
    (void)(bhy_set_host_intf_ctrl(BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy));
    (void)(bhy_get_host_intf_ctrl(&expected_data, bhy));
    if (!(expected_data & BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }
}

/**
* @brief Function to check whether any sensor is active or not
* @return Sensor active status
*/
bool bhy_are_sensors_active(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        if (sensors_active[i])
        {
            return true;
        }
    }

    return false;
}

/**
* @brief Function to deactivate all sensors and release some resources
* @param[in] cli_ref  : Reference to command line
*/
void bhy_exit(struct bhy_cli_ref *cli_ref)
{
    struct parse_ref *parse_table = &(cli_ref->parse_table);
    struct parse_sensor_details *sensor_details;

    sensor_conf.sample_rate = 0.0f;
    sensor_conf.latency = 0;
    for (uint16_t i = 0; i <= 255; i++)
    {
        if (sensors_active[i])
        {
            (void)bhy_virtual_sensor_conf_param_set_cfg((uint8_t)i, &sensor_conf, &cli_ref->bhy);
            sensors_active[i] = false;
            sensor_details = parse_add_sensor_details((uint8_t)i, parse_table);
            sensor_details->parse_flag = PARSE_FLAG_NONE;
        }
    }

    for (uint8_t i = 0; i < 10; i++)
    {
        /* Process meta events over a period of 10ms*/
        (void)(bhy_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &cli_ref->bhy));
        cli_ref->bhy.hif.delay_us(10000, NULL);
    }

    if (cli_ref->parse_table.logdev.logfile)
    {
        fclose(cli_ref->parse_table.logdev.logfile);
        cli_ref->parse_table.logdev.logfile = NULL;
        memset(cli_ref->parse_table.logdev.logfilename, 0, sizeof(cli_ref->parse_table.logdev.logfilename));
    }
}

/**
* @brief Function to parse the callback
* @param[in] cli_ref  : Reference to command line
*/
void bhy_data_parse_callback(struct bhy_cli_ref *cli_ref)
{
    if (get_interrupt_status())
    {
        (void)(bhy_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &cli_ref->bhy));
    }
}

/**
* @brief Function to get the parse callback for some sensors (part 1)
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy_fifo_parse_callback_t bhy_get_parse_callback(uint8_t sensor_id)
{
    bhy_fifo_parse_callback_t callback = NULL;

    if (((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_ACC_RAW_WU)) ||
        (sensor_id == BHY_SENSOR_ID_ACC_BIAS_WU) || (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU) ||
        ((sensor_id >= BHY_SENSOR_ID_GYRO_PASS) && (sensor_id <= BHY_SENSOR_ID_GYRO_RAW_WU)) ||
        ((sensor_id >= BHY_SENSOR_ID_MAG_PASS) && (sensor_id <= BHY_SENSOR_ID_LACC_WU)) ||
        (sensor_id == BHY_SENSOR_ID_MAG_BIAS) || (sensor_id == BHY_SENSOR_ID_MAG_BIAS_WU))
    {
        callback = parse_3axis_s16;
    }
    else if ((sensor_id == BHY_SENSOR_ID_ORI) || (sensor_id == BHY_SENSOR_ID_ORI_WU))
    {
        callback = parse_euler;
    }
    else if ((sensor_id >= BHY_SENSOR_ID_RV) && (sensor_id <= BHY_SENSOR_ID_GEORV_WU))
    {
        callback = parse_quaternion;
    }
    else if ((sensor_id == BHY_SENSOR_ID_HUM) || (sensor_id == BHY_SENSOR_ID_HUM_WU))
    {
        callback = parse_scalar_u8;
    }
    else if ((sensor_id == BHY_SENSOR_ID_GAS) || (sensor_id == BHY_SENSOR_ID_GAS_WU))
    {
        callback = parse_scalar_u32;
    }

    return callback;
}

/**
* @brief Function to get the parse callback for some sensors (part 2)
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy_fifo_parse_callback_t bhy_get_func_callback(uint8_t sensor_id)
{
    bhy_fifo_parse_callback_t callback = NULL;

    if ((sensor_id == BHY_SENSOR_ID_DEVICE_ORI) || (sensor_id == BHY_SENSOR_ID_DEVICE_ORI_WU))
    {
        callback = parse_device_ori;
    }
    else if ((sensor_id == BHY_SENSOR_ID_TEMP) || (sensor_id == BHY_SENSOR_ID_TEMP_WU))
    {
        callback = parse_s16_as_float;
    }
    else if ((sensor_id == BHY_SENSOR_ID_BARO) || (sensor_id == BHY_SENSOR_ID_BARO_WU))
    {
        callback = parse_u24_as_float;
    }

    #if 0
    else if (sensor_id == BHY_SENSOR_ID_KLIO)
    {
        callback = parse_klio;
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO_GENERIC)
    {
        callback = parse_klio_generic;
    }
    else if (sensor_id == BHY_SENSOR_ID_KLIO_LOG)
    {
        callback = parse_klio_log;
    }
    else if (sensor_id == BHY_SENSOR_ID_SWIM)
    {
        callback = parse_swim;
    }
    #endif
    else if ((sensor_id == BHY_SENSOR_ID_SI_ACCEL) || (sensor_id == BHY_SENSOR_ID_SI_GYROS))
    {
        callback = parse_acc_gyro;
    }

    #if 0
    else if ((sensor_id == BHY_SENSOR_ID_LIGHT) || (sensor_id == BHY_SENSOR_ID_LIGHT_WU) ||
             (sensor_id == BHY_SENSOR_BMP_TEMPERATURE) || (sensor_id == BHY_SENSOR_BMP_TEMPERATURE_WU))
    {
        callback = parse_s16_as_float;
    }
    else if ((sensor_id == BHY_SENSOR_ID_PROX) || (sensor_id == BHY_SENSOR_ID_PROX_WU) ||
             (sensor_id == BHY_SENSOR_ID_EXCAMERA))
    {
        callback = parse_scalar_u8;
    }

    #endif

    return callback;
}

/**
* @brief Function to get the parse callback for some sensors (part 3)
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy_fifo_parse_callback_t bhy_get_motion_callback(uint8_t sensor_id)
{
    bhy_fifo_parse_callback_t callback = NULL;

    if ((sensor_id == BHY_SENSOR_ID_STC) || (sensor_id == BHY_SENSOR_ID_STC_WU) ||
        (sensor_id == BHY_SENSOR_ID_STC_LP) || (sensor_id == BHY_SENSOR_ID_STC_LP_WU))
    {
        callback = parse_scalar_u32;
    }
    else if ((sensor_id == BHY_SENSOR_ID_STD) || (sensor_id == BHY_SENSOR_ID_STD_WU) ||
             (sensor_id == BHY_SENSOR_ID_TILT_DETECTOR) || (sensor_id == BHY_SENSOR_ID_SIG_LP_WU) ||
             (sensor_id == BHY_SENSOR_ID_STD_LP) || (sensor_id == BHY_SENSOR_ID_STD_LP_WU) ||
             (sensor_id == BHY_SENSOR_ID_WRIST_TILT_GESTURE) || (sensor_id == BHY_SENSOR_ID_STATIONARY_DET) ||
             (sensor_id == BHY_SENSOR_ID_ANY_MOTION_LP_WU) || (sensor_id == BHI3_SENSOR_ID_NO_MOTION_LP_WU) ||
             (sensor_id == BHY_SENSOR_ID_MOTION_DET) || (sensor_id == BHI3_SENSOR_ID_WRIST_WEAR_LP_WU) ||
             ((sensor_id >= BHY_SENSOR_ID_SIG) && (sensor_id <= BHY_SENSOR_ID_PICKUP_GESTURE)))
    {
        callback = parse_scalar_event;
    }

    return callback;
}

/**
* @brief Function to get the parse callback for some sensors (part 4)
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy_fifo_parse_callback_t bhy_get_misc_callback(uint8_t sensor_id)
{
    bhy_fifo_parse_callback_t callback = NULL;

    if ((sensor_id == BHY_SENSOR_ID_AR) || (sensor_id == BHI3_SENSOR_ID_AR_WEAR_WU))
    {
        callback = parse_activity;
    }
    else if (sensor_id == BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU)
    {
        callback = parse_wrist_gesture_detect;
    }

    #if 0
    else if (sensor_id == BHY_SENSOR_ID_GPS)
    {
        callback = parse_gps;
    }
    #endif

    else if (sensor_id == BHI3_SENSOR_ID_MULTI_TAP)
    {
        callback = parse_multitap;
    }
    else if (sensor_id == BHY_SENSOR_ID_AIR_QUALITY)
    {
        callback = parse_air_quality;
    }
    else if (sensor_id == BHY_SENSOR_ID_HEAD_ORI_MIS_ALG)
    {
        callback = parse_hmc;
    }
    else if ((sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_Q) || (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_Q))
    {
        callback = parse_oc;
    }
    else if ((sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_E) || (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_E))
    {
        callback = parse_ec;
    }
    else
    {
        callback = parse_generic;
    }

    return callback;
}

/**
* @brief Function to get the parse callback
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy_fifo_parse_callback_t bhy_get_callback(uint8_t sensor_id)
{
    bhy_fifo_parse_callback_t callback;

    callback = bhy_get_parse_callback(sensor_id);

    if (callback == NULL)
    {
        callback = bhy_get_func_callback(sensor_id);

        if (callback == NULL)
        {
            callback = bhy_get_motion_callback(sensor_id);

            if (callback == NULL)
            {
                callback = bhy_get_misc_callback(sensor_id);
            }
        }
    }

    return callback;
}

#if 0

/**
* @brief Function to print help for kstatus command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kstatus_help(void * const ref)
{
    (void)ref;

    PRINT("  kstatus\r\n");
    PRINT("        = Get and reset current klio driver status\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kstatus command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kstatus_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    klio_enable(&cli_ref->bhy);
    klio_status(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksetstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetstate_help(void *ref)
{
    (void)ref;
    PRINT("  ksetstate <le> <lr> <re> <rr>\r\n");
    PRINT("        = Set cyclic klio state\r\n");
    PRINT("         <le> learning enable (0/1)\r\n");
    PRINT("         <lr> learning reset (0/1)\r\n");
    PRINT("         <re> recognition enable (0/1)\r\n");
    PRINT("         <rr> recognition reset (0/1)\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksetstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s %s %s %s\r\n",
         (char *)argv[0],
         (char *)argv[1],
         (char *)argv[2],
         (char *)argv[3],
         (char *)argv[4]);
    klio_enable(&cli_ref->bhy);
    klio_set_state((char *)argv[1], (char *)argv[2], (char *)argv[3], (char *)argv[4], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksetgstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetgstate_help(void *ref)
{
    (void)ref;

    PRINT("  ksetgstate <en>\r\n");
    PRINT("        = Enable/disable generic gesture recognition\r\n");
    PRINT("         <en> recognition enabled (0/1)\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksetgstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetgstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_set_generic_recognition_state((char *)argv[1], &cli_ref->bhy);

    return CLI_OK;
}

/**
* @brief Function to print help for kgetgstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetgstate_help(void *ref)
{
    (void)ref;

    PRINT("  kgetgstate\r\n");
    PRINT("        = Get current generic Klio state\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kgetgstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetgstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    klio_get_generic_recognition_state(&cli_ref->bhy);

    return CLI_OK;
}

/**
* @brief Function to print help for kgetstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetstate_help(void *ref)
{
    (void)ref;

    PRINT("  kgetstate\r\n");
    PRINT("        = Get current cyclic Klio state\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kreset command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kreset_help(void *ref)
{
    (void)ref;

    PRINT("  kreset\r\n");
    PRINT("        = Reset all Klio state\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kreset command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kreset_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", (char *)argv[0]);
    (void)bhy_klio_param_reset(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksetgestconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetgestconf_help(void *ref)
{
    (void)ref;

    PRINT("  ksetgestconf <index> <config>\r\n");
    PRINT("        = Load a pattern config for the generic recognition algorithm.\r\n");
    PRINT("          Each pattern must have a config associated with it.\r\n");
    PRINT("         <index> config index to write to, must match pattern index\r\n");
    PRINT("         <config> config as bare hex bytestring\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksetgestconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetgestconf_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", (char *)argv[0], (char *)argv[1]);
    klio_set_gesture_config((char *)argv[1], (char *)argv[2], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksettimeconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksettimeconf_help(void *ref)
{
    (void)ref;

    PRINT("  ksettimeconf <config>\r\n");
    PRINT("        = Load expected timing between gestures for the generic\r\n");
    PRINT("          recognition algorithm.\r\n");
    PRINT("         <config> config as bare hex bytestring\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksettimeconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksettimeconf_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", (char *)argv[0]);
    klio_set_timing_config((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kldpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kldpatt_help(void *ref)
{
    (void)ref;

    PRINT("  kldpatt <index> <pattern>\r\n");
    PRINT("        = Load a pattern/adaptive pattern for cyclic recognition. If loading an\r\n");
    PRINT("          adaptive pattern, a regular pattern must have been previously\r\n");
    PRINT("          loaded on the given index\r\n");
    PRINT("         <index> pattern index to write to\r\n");
    PRINT("         <pattern> pattern/adaptive pattern as bare hex bytestring\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kldgpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kldgpatt_help(void *ref)
{
    (void)ref;

    PRINT("  kldgpatt <index> <pattern>\r\n");
    PRINT("        = Load a pattern for generic recognition.\r\n");
    PRINT("         <index> pattern index to write to\r\n");
    PRINT("         <pattern> pattern as bare hex bytestring\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kenpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kenpatt_help(void *ref)
{
    (void)ref;

    PRINT("  kenpatt <patterns>\r\n");
    PRINT("        = Enable pattern ids for cyclic recognition\r\n");
    PRINT("         <patterns> pattern indices to enable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kdispatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kdispatt_help(void *ref)
{
    (void)ref;

    PRINT("  kdispatt <patterns>\r\n");
    PRINT("        = Disable pattern ids for cyclic recognition\r\n");
    PRINT("         <patterns> pattern indices to disable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kdisapatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kdisapatt_help(void *ref)
{
    (void)ref;

    PRINT("  kdisapatt <patterns>\r\n");
    PRINT("        = Disable pattern adaptation for given pattern ids\r\n");
    PRINT("         <patterns> pattern indices to disable, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kswpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kswpatt_help(void *ref)
{
    (void)ref;

    PRINT("  kswpatt <patterns>\r\n");
    PRINT("        = Switch pattern between left/right hand\r\n");
    PRINT("         <patterns> pattern indices to switch, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kautldpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kautldpatt_help(void *ref)
{
    (void)ref;

    PRINT("  kautldpatt <enable> <index>\r\n");
    PRINT("        = Automatically use learnt patterns for recognition\r\n");
    PRINT("         <enable> enable or disable (1/0)\r\n");
    PRINT("         <index> pattern index to start loading into (normally 0)\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kgetparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetparam_help(void *ref)
{
    (void)ref;

    PRINT("  kgetparam <param>\r\n");
    PRINT("        = Print klio parameter\r\n");
    PRINT("         <param> parameter id, see documentation\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kgetstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    klio_enable(&cli_ref->bhy);
    klio_get_state(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kldpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kldpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_load_cyclic_pattern((char *)argv[1], (char *)argv[2], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kldgpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kldgpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_load_generic_pattern((char *)argv[1], (char *)argv[2], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kenpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kenpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_ENABLE, (char *)argv[1], &cli_ref->bhy);
    PRINT("Enabled pattern %s for recognition\r\n", argv[1]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for kdispatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kdispatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_DISABLE, (char *)argv[1], &cli_ref->bhy);
    PRINT("Disabled pattern %s from recognition\r\n", argv[1]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for kdisapatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kdisapatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_AP_DISABLE, (char *)argv[1], &cli_ref->bhy);
    PRINT("Disable adaptation for pattern %s\r\n", argv[1]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for kswpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kswpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);
    klio_pattern_state_operation(KLIO_PATTERN_STATE_SWITCH_HAND, (char *)argv[1], &cli_ref->bhy);
    PRINT("Switched pattern to opposite hand for pattern %s\r\n", argv[1]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for kautldpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kautldpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy);
    klio_vars.auto_load_pattern = (uint8_t) atoi((char *)argv[1]);
    klio_vars.auto_load_pattern_write_index = (uint8_t) atoi((char *)argv[2]);
    PRINT("Klio auto load pattern %s, starting from index %s\r\n",
          (klio_vars.auto_load_pattern == 0) ? "Disabled" : "Enabled",
          argv[2]);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kgetparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetparam_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    klio_enable(&cli_ref->bhy);

    /*lint -e826*/
    klio_get_parameter((uint16_t *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksetparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetparam_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy);
    klio_set_parameter((char *)argv[1], (char *)argv[2], &cli_ref->bhy);
    PRINT("Set value %s for parameter id %s\r\n", argv[2], argv[1]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for ksimscore command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksimscore_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy);
    klio_similarity_score(argv[1], argv[2], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for kmsimscore command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kmsimscore_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0], argv[1], argv[2]);
    klio_enable(&cli_ref->bhy);
    klio_similarity_score_multiple((const char *)argv[1], (const char *)argv[2], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksetparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetparam_help(void *ref)
{
    (void)ref;

    PRINT("  ksetparam <param> <value>\r\n");
    PRINT("        = Set klio parameter\r\n");
    PRINT("         <param> parameter id, see documentation\r\n");
    PRINT("         <value> depends on parameter id, see documentation\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ksimscore command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksimscore_help(void *ref)
{
    (void)ref;

    PRINT("  ksimscore <pattern1> <pattern2>\r\n");
    PRINT("        = Print similarity score for two patterns\r\n");
    PRINT("         <pattern1> first pattern as bare hex bytestring\r\n");
    PRINT("         <pattern2> second pattern as bare hex bytestring\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for kmsimscore command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kmsimscore_help(void *ref)
{
    (void)ref;

    PRINT("  kmsimscore <base index> <comparison indices>\r\n");
    PRINT("        = Print similarity score for one or more stored patterns\r\n");
    PRINT("         <base index> compare the patterns in <comparison indices> with this pattern index\r\n");
    PRINT(
        "         <comparison indices> pattern indices to compare with pattern in base index, specified as 0,1,4 etc\r\n");

    return CLI_OK;
}

#endif

/**
* @brief Function to print help for version command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t version_help(void *ref)
{
    (void)ref;

    PRINT("  version\r\n");
    PRINT("        = Prints the version\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for version command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t version_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    struct coines_board_info board_info;

    INFO("Executing %s\r\n", argv[0]);

    (void)coines_get_board_info(&board_info);

    PRINT("HW info:: Board: %u, HW ID: %X, Shuttle ID: %X, SW ID: %X\r\n",
          board_info.board,
          board_info.hardware_id,
          board_info.shuttle_id,
          board_info.software_id);
    PRINT("SW Version: %s.%s.%s\r\nBuild date: " __DATE__ "\r\n\r\n\r\n",
          BHY2CLI_VER_MAJOR,
          BHY2CLI_VER_MINOR,
          BHY2CLI_VER_BUGFIX);

    return CLI_OK;
}

/**
* @brief Function to print help for help command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t help_help(void *ref)
{
    (void)ref;

    PRINT("Usage:\r\n");
    PRINT("bhy2cli [<port>] [<port_name>] [<options>]\r\n");
    PRINT("<port>: optional input parameter, trigger keyword\r\n");
    PRINT(
        "<port_name>: optional input parameter, use it when want to support running multiple applications in parallel\r\n");
    PRINT("Options:\r\n");
    PRINT("  -h OR help\t= Print this usage message\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for help command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t help_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    (void)cli_help(ref, &(cli_ref->cli_dev));

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for info command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t info_help(void *ref)
{
    (void)ref;

    PRINT("  -i OR info\t= Show device information: Device ID,\r\n");
    PRINT("    \t  ROM version, RAM version, Power state,\r\n");
    PRINT("    \t  list of available sensors,\r\n");
    PRINT("    \t  content of Boot Status register,\r\n");
    PRINT("    \t  content of Error value register\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for info command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t info_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    show_info(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ramb command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ramb_help(void *ref)
{
    (void)ref;

    PRINT("  -b OR ramb <firmware path>\r\n");
    PRINT("    \t= Reset, upload specified firmware to RAM and boot from RAM\r\n");
    PRINT("    \t  [equivalent to using \"reset ram <firmware> boot r\" successively]\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ramb command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ramb_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    reset_hub(&cli_ref->bhy);
    if (upload_to_ram((char *)argv[1], &cli_ref->bhy))
    {
        boot_ram(&cli_ref->bhy);
        (void)bhy_system_param_get_virtual_sensor_present(&cli_ref->bhy);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

#if 0

/**
* @brief Function to print help for flb command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t flb_help(void *ref)
{
    (void)ref;

    PRINT("   -d OR flb <firmware path>\r\n");
    PRINT("    \t= Reset, upload specified firmware to Flash and boot from Flash\r\n");
    PRINT("    \t  [equivalent to using \"reset fl <firmware path> boot f\" successively]\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for flb command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t flb_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    reset_hub(&cli_ref->bhy);
    if (upload_to_flash((char *)argv[1], &cli_ref->bhy))
    {
        boot_flash(&cli_ref->bhy);
        (void)bhy_system_param_get_virtual_sensor_present(&cli_ref->bhy);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}
#endif

/**
* @brief Function to print help for reset command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t reset_help(void *ref)
{
    (void)ref;

    PRINT("  -n OR reset\t= Reset sensor hub\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for reset command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t reset_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);
    reset_hub(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for addse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t addse_help(void *ref)
{
    (void)ref;

    PRINT("  -a OR addse <sensor id>:<sensor name>:<total output payload in bytes>:\r\n");
    PRINT("     <output_format_0>:<output_format_1>\r\n");
    PRINT("    \t= Register the expected payload of a new custom virtual sensor\r\n");
    PRINT("    \t -Valid output_formats: u8: Unsigned 8 Bit, u16: Unsigned 16 Bit, u32:\r\n");
    PRINT("    \t  Unsigned 32 Bit, s8: Signed 8 Bit, s16: Signed 16 Bit, s32: Signed 32 Bit,\r\n");
    PRINT("    \t  f: Float, c: Char \r\n");
    PRINT("    \t -e.g.: addse 160:\"Lean Orientation\":2:c:c \r\n");
    PRINT("    \t -Note that the corresponding virtual sensor has to be enabled in the same function\r\n");
    PRINT("    \t  call (trailing actse option), since the registration of the sensor is temporary. \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for addse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t addse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    add_sensor((char *)argv[1], ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for rd command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rd_help(void *ref)
{
    (void)ref;

    PRINT("  -r OR rd <adr>[:<len>]\r\n");
    PRINT("    \t= Read from register address <adr> for length <len> bytes\r\n");
    PRINT("    \t -If input <len> is not provided, the default read length is 1 byte\r\n");
    PRINT("    \t -When reading registers with auto-increment, the provided register as well as\r\n");
    PRINT("    \t  the following registers will be read\r\n");
    PRINT("    \t -e.g rd 0x08:3 will read the data of registers 0x08, 0x09 and 0x0a\r\n");
    PRINT("    \t  max. 53 bytes can be read at once\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for rd command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rd_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_regs((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for wr command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wr_help(void *ref)
{
    (void)ref;

    PRINT("  -w OR wr <adr>=<val1>[,<val2>]...\r\n");
    PRINT("    \t= Write to register address <adr> with comma separated values <val>\r\n");
    PRINT("    \t -If more values provided <val>, the additional\r\n");
    PRINT("    \t  values will be written to the following addresses\r\n");
    PRINT("    \t -When writing to registers with auto-increment, the provided register as well as\r\n");
    PRINT("    \t  the following registers will be written\r\n");
    PRINT("    \t -e.g wr 0x08=0x02,0x03,0x04 will write the provided data to registers 0x08, 0x09\r\n");
    PRINT("    \t  and 0x0a. Max. 46 bytes can be written at once\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wr command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wr_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    wr_regs((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for rdp command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rdp_help(void *ref)
{
    (void)ref;

    PRINT("  -s OR rdp <param id>\r\n");
    PRINT("    \t= Display read_param response of parameter <param id>\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for rdp command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rdp_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_param((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for wrp command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wrp_help(void *ref)
{
    (void)ref;

    PRINT("  -t OR wrp <param id>=<val1>[,<val2>]...\r\n");
    PRINT("    \t= Write data to parameter <param id> with the bytes to be written, <val1>[,<val2>]... \r\n");
    PRINT("    \t -e.g. 0x103=5,6 will write 0x05 to the first byte and 0x06 to the second byte\r\n");
    PRINT("    \t  of the parameter \"Fifo Control\"\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wrp command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wrp_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    wr_param((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for physeninfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t physeninfo_help(void *ref)
{
    (void)ref;

    PRINT("  -p OR physeninfo <physical sensor id>\r\n");
    PRINT("    \t= Display Physical Sensor Information of <physical sensor id>\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for physeninfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t physeninfo_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    rd_phy_sensor_info((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for ram command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ram_help(void *ref)
{
    (void)ref;

    PRINT("  ram <firmware path>\r\n");
    PRINT("    \t= Upload firmware to RAM\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ram command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ram_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    (void)upload_to_ram((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

#if 0

/**
* @brief Function to print help for fl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t fl_help(void *ref)
{
    (void)ref;

    PRINT("  fl <firmware path>\r\n");
    PRINT("    \t= Upload firmware to external-flash\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for fl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t fl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    (void)upload_to_flash((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}
#endif

/**
* @brief Function to print help for boot command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t boot_help(void *ref)
{
    (void)ref;

    PRINT("  -g OR boot <medium>\r\n");
    PRINT("    \t= Boot from the specified <medium>: \"f\" for FLASH, \"r\" for RAM\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for boot command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t boot_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    if ((argv[1][0]) == 'r')
    {
        boot_ram(&cli_ref->bhy);
        (void)bhy_system_param_get_virtual_sensor_present(&cli_ref->bhy);
    }

#if 0
    else if ((argv[1][0]) == 'f')
    {
        boot_flash(&cli_ref->bhy);
        (void)bhy_system_param_get_virtual_sensor_present(&cli_ref->bhy);
    }
#endif
    else
    {
        ERROR("Invalid boot medium: %s\r\n", argv[1]);

        return CLI_E_INVALID_PARAM;
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

#if 0

/**
* @brief Function to print help for erase command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t erase_help(void *ref)
{
    (void)ref;

    PRINT("  -e OR erase\t= Erase external-flash\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for erase command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t erase_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s \r\n", argv[0]);
    erase_flash(BHY2_FLASH_SIZE_4MB, &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for efd command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t efd_help(void *ref)
{
    (void)ref;

    PRINT("  efd\t= Erase the flash descriptor\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for efd command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t efd_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_dev *bhy = &cli_ref->bhy;
    int8_t rslt;
    uint8_t boot_status;

    (void)(bhy_get_boot_status(&boot_status, bhy));
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Reset the sensor before erasing the flash descriptor\r\n");

            return CLI_OK;
        }
    }

    PRINT("Erasing flash descriptor. This might hang if a reset command before this was not issued\r\n");
    rslt = bhy_erase_flash(0, 0xFFF, bhy); /* 0xFFF is hopefully within the first sector */
    if (rslt != BHY_OK)
    {
        ERROR("Erasing flash descriptor failed, status: %02d\r\n", rslt);

        return CLI_OK;
    }

    PRINT("Erasing flash descriptor successful\r\n");

    return CLI_OK;
}
#endif

/**
* @brief Function to print help for actse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t actse_help(void *ref)
{
    (void)ref;

    PRINT("  -c OR actse <sensor id>:<frequency>[:<latency>][:<downsampling>]\r\n");
    PRINT("    \t= Activate sensor <sensor id> at specified sample rate <frequency>,\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -<downsampling> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple actse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");
    PRINT("    \t -downsampling(Hz): sensor downsampling value, it should be smaller than frequency to take effect\r\n");
    PRINT("    \t -Eg:\r\n");
    PRINT("    \t -actse 3:50::10\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for actse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t actse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_STREAM, (struct bhy_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for schema command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t schema_help(void *ref)
{
    (void)ref;

    PRINT("  schema\t= Show schema information: \r\n");
    PRINT("    \t  ID: Name: Event size: Parsing format: Axis names: Scaling\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for schema command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t schema_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    schema_info(&cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for hexse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hexse_help(void *ref)
{
    (void)ref;

    PRINT("  hexse <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT("    \t= Stream sensor <sensor id> at specified sample rate <frequency>, in hex format\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple logse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hexse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hexse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_HEXSTREAM, (struct bhy_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for logse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t logse_help(void *ref)
{
    (void)ref;

    PRINT("  logse <sensor id>:<frequency>[:<latency>]\r\n");
    PRINT("    \t= Log sensor <sensor id> at specified sample rate <frequency>,\r\n");
    PRINT("    \t -latency <latency>, duration time <time>, sample counts <count>\r\n");
    PRINT("    \t -At least <frequency> is a must input parameter\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -One or more sensors can be active by passing multiple logse options\r\n");
    PRINT("    \t -id: sensor id\r\n");
    PRINT("    \t -frequency(Hz): sensor ODR\r\n");
    PRINT("    \t -latency(ms): sensor data outputs with a latency\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for logse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t logse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    activate_sensor((char *)argv[1], PARSE_FLAG_LOG, (struct bhy_cli_ref *)ref);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for attlog command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t attlog_help(void *ref)
{
    (void)ref;

    PRINT("  attlog <filename.ext>\r\n");
    PRINT("    \t= Attach (and create if required) a log file (write-only),");
    PRINT(" where data can be logged to\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for attlog command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t attlog_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    if (cli_ref->parse_table.logdev.logfile == NULL)
    {
        INFO("Executing %s %s\r\n", argv[0], argv[1]);

        cli_ref->parse_table.logdev.logfile = fopen((char *)argv[1], "wb");
        memcpy(cli_ref->parse_table.logdev.logfilename, (char *)argv[1], strlen((char *)argv[1]));

        if (cli_ref->parse_table.logdev.logfile)
        {
            PRINT("File %s was created\r\n", cli_ref->parse_table.logdev.logfilename);
            write_meta_info(&cli_ref->parse_table.logdev, &cli_ref->bhy);
        }
        else
        {
            ERROR("File %s could not be found/created\r\n", cli_ref->parse_table.logdev.logfilename);
        }
    }
    else
    {
        ERROR("File %s is open. Please use 'detlog' to detach the open file\r\n",
              cli_ref->parse_table.logdev.logfilename);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for detlog command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t detlog_help(void *ref)
{
    (void)ref;

    PRINT("  detlog <filename.ext>\r\n");
    PRINT("    \t= Detach the log file \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for detlog command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t detlog_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        INFO("Executing %s\r\n", argv[0]);

        if (!strcmp(cli_ref->parse_table.logdev.logfilename, (char *)argv[1]))
        {
            fclose(cli_ref->parse_table.logdev.logfile);
            cli_ref->parse_table.logdev.logfile = NULL;
            PRINT("File %s was detached for logging\r\n", cli_ref->parse_table.logdev.logfilename);
            memset(cli_ref->parse_table.logdev.logfilename, 0, sizeof(cli_ref->parse_table.logdev.logfilename));
        }
        else
        {
            ERROR("Passed Filename does not match with Open File\r\n");
        }
    }
    else
    {
        ERROR("No file to detach\r\n");
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for setvirtsenconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setvirtsenconf_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    uint8_t sensor_id = (uint8_t) atoi((char *)argv[1]);

    sensor_conf.sample_rate = (bhy_float) atof((char *)argv[2]);
    sensor_conf.latency = (uint32_t) atoi((char *)argv[3]);

    rslt = bhy_virtual_sensor_conf_param_set_cfg(sensor_id, &sensor_conf, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Virtual sensor configuration set failed \r\n");

        return rslt;
    }

    PRINT("Virtual sensor configuration set successfully \r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for setvirtsenconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setvirtsenconf_help(void *ref)
{
    (void)ref;

    PRINT("  setvirtsenconf <sensor_id> <sample_rate> <latency>\r\n");
    PRINT("    \t= Setting virtual sensor configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for getvirtsenconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getvirtsenconf_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    uint8_t sensor_id = (uint8_t) atoi((char *)argv[1]);

    rslt = bhy_virtual_sensor_conf_param_get_cfg(sensor_id, &sensor_conf, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Virtual sensor configuration set failed \r\n");

        return rslt;
    }

    PRINT("Virtual sensor configuration get successfully \r\n");
    PRINT("Custom sensor ID=%d, rate=%.2fHz,latency=%ld, range=%d\r\n",
          sensor_id,
          sensor_conf.sample_rate,
          sensor_conf.latency,
          sensor_conf.range);

    return CLI_OK;
}

/**
* @brief Function to print help for getvirtsenconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getvirtsenconf_help(void *ref)
{
    (void)ref;

    PRINT("  getvirtsenconf <sensor_id>\r\n");
    PRINT("    \t= Getting virtual sensor configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for slabel command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t slabel_help(void *ref)
{
    (void)ref;

    PRINT("  slabel <label string>\r\n");
    PRINT("    \t= Set a string label in the log file %u characters long\r\n", LOGBIN_LABEL_SIZE);

    return CLI_OK;
}

/**
* @brief Function to implement callback for slabel command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t slabel_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t label[LOGBIN_LABEL_SIZE] = { 0 };
    uint64_t timestamp_ns;

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        if (strlen((char *)argv[1]) <= LOGBIN_LABEL_SIZE)
        {
            strcpy((char *)label, (char *)argv[1]);
        }
        else
        {
            memcpy(label, argv[1], LOGBIN_LABEL_SIZE);
        }

        INFO("Executing %s %s %s\r\n", argv[0], argv[1], label);
        (void)bhy_get_hw_timestamp_ns(&timestamp_ns, &cli_ref->bhy);

        /* System IDs start at 224 */
        log_data(LOGBIN_META_ID_LABEL, timestamp_ns, LOGBIN_LABEL_SIZE, label, &cli_ref->parse_table.logdev);
    }
    else
    {
        ERROR("No open log file to set labels\r\n");
    }

    return CLI_OK;
}

#if 0

/**
* @brief Function to print help for swim command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swim_help(void *ref)
{
    (void)ref;

    PRINT("  swim <e/d> <r/l> <pool length>\r\n");
    PRINT("    \t<e/d>= Enable / Disable\r\n");
    PRINT("    \t<r/l>= Right / Left\r\n");
    PRINT("    \t<length>= Length of the pool as an integer\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimver_help(void *ref)
{
    (void)ref;

    PRINT("  swimver\r\n");
    PRINT("    \t= Get the algorithm version\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimgetfreq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimgetfreq_help(void *ref)
{
    (void)ref;

    PRINT("  swimgetfreq\r\n");
    PRINT("    \t To Get the Swim sampling frequency\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimsetfreq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetfreq_help(void *ref)
{
    (void)ref;

    PRINT("  swimsetfreq <Freq> <latency>\r\n");
    PRINT("    \t To SET the Swim sampling frequency\r\n");
    PRINT("    \t <Freq> = Frequency (in Hz) to set \r\n");
    PRINT("    \t <Latency> = latency (ms) to set \r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimgetaxes command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimgetaxes_help(void *ref)
{
    (void)ref;

    PRINT("  swimgetaxes\r\n");
    PRINT("    \t= Get the orientation of Physical sensor set for swim algorithm\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimsetaxes command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetaxes_help(void *ref)
{
    (void)ref;

    PRINT("  swimsetaxes <orientation_matrix>\r\n");
    PRINT("    \t= Set the orientation of Physical sensor set for swim algorithm\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for swimsetlogging command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetlogging_help(void *ref)
{
    (void)ref;

    PRINT("  swimsetlogging status\r\n");
    PRINT("    \t= Enable logging for swim algorithm\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimgetfreq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimgetfreq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    assert_rslt = bhy_virtual_sensor_conf_param_get_cfg(BHY_SENSOR_ID_SWIM, &sensor_conf, &cli_ref->bhy);
    if (assert_rslt)
    {
        PRINT("SWIMFREQ GET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("SWIMFREQ %.2f\r\n\r\n\r\n", sensor_conf.sample_rate);
    }

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimsetfreq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetfreq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    sensor_conf.sample_rate = 0.0F;
    sensor_conf.latency = 0;

    INFO("Executing %s\r\n", argv[0]);

    sensor_conf.sample_rate = (bhy_float) atof((const char *)argv[1]);
    sensor_conf.latency = (uint32_t) atoi((const char *)argv[2]);

    assert_rslt = bhy_virtual_sensor_conf_param_set_cfg(BHY_SENSOR_ID_SWIM, &sensor_conf, &cli_ref->bhy);

    if (assert_rslt)
    {
        PRINT("SWIMFREQ SET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("SWIMFREQ SET Success\r\n\r\n\r\n");
    }

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimver_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_swim_param_version_t swim_algo_ver;
    bhy_swim_param_config_t config;

    INFO("Executing %s\r\n", argv[0]);

    if (bhy_swim_param_get_version(&swim_algo_ver, &cli_ref->bhy))
    {
        PRINT("SWIMVER 0.0.0\r\n\r\n\r\n");
    }
    else
    {
        if (bhy_swim_param_get_config(&config, &cli_ref->bhy))
        {
            PRINT("SWIMVER %u.%u.%u\r\n\r\n\r\n", swim_algo_ver.major, swim_algo_ver.minor, swim_algo_ver.platform);
        }
        else
        {
            PRINT("SWIMVER %u.%u.%u\r\nSWIMCONF %s %u\r\n\r\n\r\n",
                  swim_algo_ver.major,
                  swim_algo_ver.minor,
                  swim_algo_ver.platform,
                  config.dev_on_left_hand ? "LEFT" : "RIGHT",
                  config.pool_length_integral);
        }
    }

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimgetaxes command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimgetaxes_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_orient_matrix orient_matrix;
    uint8_t loop;

    INFO("Executing %s\r\n", argv[0]);
    (void)(bhy_get_orientation_matrix(BHY_PHYS_SENSOR_ID_ACCELEROMETER, &orient_matrix, &cli_ref->bhy));
    PRINT("Acc ");

    for (loop = 0; loop < 8; loop++)
    {
        PRINT("%d,", orient_matrix.c[loop]);
    }

    PRINT("%d\r\n\r\n", orient_matrix.c[loop]);

    memset(&orient_matrix.c[0], 0x0, sizeof(orient_matrix.c) / sizeof(orient_matrix.c[0]));
    (void)(bhy_get_orientation_matrix(BHY_PHYS_SENSOR_ID_GYROSCOPE, &orient_matrix, &cli_ref->bhy));
    PRINT("Gyro ");

    for (loop = 0; loop < 8; loop++)
    {
        PRINT("%d,", orient_matrix.c[loop]);
    }

    PRINT("%d\r\n\r\n", orient_matrix.c[loop]);

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimsetaxes command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetaxes_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_orient_matrix orient_matrix = { { 0 } };
    uint8_t loop = 0;
    char delimiter[] = ",";
    char *axes = (char *)argv[1];
    char *token = strtok(axes, delimiter);

    while ((token != NULL))
    {
        orient_matrix.c[loop] = (int8_t) atoi(token);
        token = strtok(NULL, delimiter);
        loop++;
    }

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    (void)(bhy_system_param_set_physical_sensor_info(BHY_PHYS_SENSOR_ID_ACCELEROMETER, &orient_matrix, &cli_ref->bhy));
    (void)(bhy_system_param_set_physical_sensor_info(BHY_PHYS_SENSOR_ID_GYROSCOPE, &orient_matrix, &cli_ref->bhy));

    PRINT("Set the orientation matrix for the Physical Sensors successfully");

    return CLI_OK;
}

/**
* @brief Function to implement callback for swimsetlogging command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetlogging_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t state;

    INFO("Executing %s\r\n", argv[0]);

    state = (uint8_t) atoi((const char *)argv[1]);

    assert_rslt = bhy_swim_param_set_logging(state, &cli_ref->bhy);

    if (assert_rslt)
    {
        PRINT("SWIMSETLOGGING SET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("SWIMSETLOGGING SET Success\r\n\r\n\r\n");
    }

    return CLI_OK;
}

/**
* @brief Function to implement callback for swim command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swim_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_swim_param_config_t config;

    INFO("Executing %s %s %s %s\r\n", argv[0], argv[1], argv[2], argv[3]);

    config.update_swim_config = (uint8_t)((argv[1][0] == 'e') ? BHY_SWIM_ENABLE_CONFIG : BHY_SWIM_DISABLE_CONFIG);
    config.dev_on_left_hand =
        (uint8_t)((argv[2][0] == 'r') ? BHY_SWIM_DEVICE_ON_RIGHT_HAND : BHY_SWIM_DEVICE_ON_LEFT_HAND);
    config.pool_length_integral = (uint8_t) (atoi((char *)argv[3]));
    config.pool_length_floating = 0;

    (void)(bhy_swim_param_set_config(&config, &cli_ref->bhy));

    PRINT("Setting swim config: %s, %s, %u\r\n\r\n",
          (argv[1][0] == 'e') ? "Enable" : "Disable",
          (argv[2][0] == 'r') ? "Right" : "Left",
          atoi((char *)argv[3]));

    /*! Writing the final swim output after disabling swim sensor*/
    if (config.update_swim_config == BHY_SWIM_DISABLE_CONFIG)
    {
        PRINT("Summary D: %u; C: %u; FRS: %u; BRS: %u; BTS: %u; BKS: %u; STC: %u\r\n",
              swim_data.total_distance,
              swim_data.length_count,
              swim_data.lengths_freestyle,
              swim_data.lengths_breaststroke,
              swim_data.lengths_butterfly,
              swim_data.lengths_backstroke,
              swim_data.stroke_count);
    }

    return CLI_OK;
}
#endif

/**
* @brief Function to convert time in tick to seconds and nanoseconds
* @param[in] time_ticks : Time in ticks
* @param[out] s         : Second part of time
* @param[out] ns        : Nanosecond part of time
* @param[out] tns       : Total time in nanoseconds
*/
static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

#if 0

/**
* @brief Function to parse cyclic Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    bhy_event_data_klio_t data;
    uint8_t parse_flag;
    uint32_t klio_driver_status;
    uint8_t tmp_buf[252];
    uint16_t bufsize = sizeof(tmp_buf);
    bhy_klio_param_sensor_state_t klio_sensor_state;
    struct bhy_dev *bhy;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    bhy = parse_table->bhy;

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (data.learn.index != -1)
    {
        /* Read out learnt pattern */
        (void)(bhy_klio_param_read_pattern(0, tmp_buf, &bufsize, bhy));
        (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

        DATA("SID: %u; T: %lu.%09lu; Pattern learnt: ", callback_info->sensor_id, s, ns);
        for (uint16_t i = 0; i < bufsize; i++)
        {
            PRINT_D("%02x", tmp_buf[i]);
        }

        PRINT_D("\r\n");

        /* write back learnt pattern for recognition */
        if (klio_vars.auto_load_pattern && klio_vars.auto_load_pattern_write_index < klio_vars.max_cyclic_patterns)
        {
            (void)(bhy_klio_param_write_pattern(klio_vars.auto_load_pattern_write_index, tmp_buf, bufsize, bhy));
            (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

            (void)(bhy_klio_param_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                     &klio_vars.auto_load_pattern_write_index, 1, bhy));
            (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

            klio_vars.auto_load_pattern_write_index++;

            /* write klio state (enable recognition, and also make sure learning is not disabled) */
            klio_sensor_state.learning_enabled = 1;
            klio_sensor_state.learning_reset = 0;
            klio_sensor_state.recognition_enabled = 1;
            klio_sensor_state.recognition_reset = 0;
            (void)(bhy_klio_param_set_state(&klio_sensor_state, bhy));
            (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));
        }
    }

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%f]\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.learn.index,
             data.learn.progress,
             data.learn.change_reason,
             data.recognize.index,
             data.recognize.count);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse generic Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio_generic(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    bhy_klio_param_generic_sensor_frame_t data;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA(
            "SID: %u; T: %lu.%09lu; SubgestId: %d; SubgestCount: %d; SubgestScore: %f; FullGestCount: %d; FullGestScore: %f\r\n",
            callback_info->sensor_id,
            s,
            ns,
            data.subgest_id,
            data.subgest_count,
            data.subgest_score,
            data.full_gest_count,
            data.full_gest_score);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse log Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_klio_log(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    bhy_klio_param_log_frame_t data;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; ax: %.9g, ay: %.9g, az: %.9g, gx: %.9g, gy: %.9g, gz: %.9g\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.accel[0],
             data.accel[1],
             data.accel[2],
             data.gyro[0],
             data.gyro[1],
             data.gyro[2]);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

#endif

/**
* @brief Function to parse for Accelerometer and Gyroscope sensors
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_acc_gyro(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float *sensor_data = (float *)callback_info->data_ptr;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; X: %f; Y: %f; Z: %f;\r\n",
             callback_info->sensor_id,
             s,
             ns,
             sensor_data[0],
             sensor_data[1],
             sensor_data[2]);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Air quality
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_air_quality(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    struct parse_sensor_details *sensor_details;
    bhy_event_data_iaq_output_t air_quality = { 0 };

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhy_event_data_parse_air_quality(callback_info->data_ptr, &air_quality);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; I: %u, S: %u, V: %u, C: %u,  A: %u, T: %d, H: %u, G: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             air_quality.iaq,
             air_quality.siaq,
             air_quality.voc,
             air_quality.co2,
             air_quality.iaq_accuracy,
             air_quality.comp_temperature,
             air_quality.comp_humidity,
             air_quality.raw_gas);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

#if 0

/**
* @brief Function to parse for swim
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_swim(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhy_event_data_swim_parsing(callback_info->data_ptr, &swim_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; D: %u; C: %u; FRS: %u; BRS: %u; BTF: %u; BKS: %u; STC: %u\r\n",
             callback_info->sensor_id,
             s,
             ns,
             swim_data.total_distance,
             swim_data.length_count,
             swim_data.lengths_freestyle,
             swim_data.lengths_breaststroke,
             swim_data.lengths_butterfly,
             swim_data.lengths_backstroke,
             swim_data.stroke_count);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

#endif

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_multitap(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    bhy_event_data_multi_tap multitap_data;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhy_event_data_multi_tap_parsing(callback_info->data_ptr, (uint8_t *)&multitap_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; %s; \r\n", callback_info->sensor_id, s, ns,
             bhy_event_data_multi_tap_string_out[multitap_data]);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_wrist_gesture_detect(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    struct bhy_event_data_wrist_gesture_detect wrist_gesture_detect_data;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhy_event_data_wrist_gesture_detect_parsing(callback_info->data_ptr, &wrist_gesture_detect_data);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; wrist_gesture: %s; \r\n",
             callback_info->sensor_id,
             s,
             ns,
             bhy_event_data_wrist_gesture_detect_output[wrist_gesture_detect_data.wrist_gesture]);
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_hmc(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhy_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhy_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x / 16384.0f,
             data.y / 16384.0f,
             data.z / 16384.0f,
             data.w / 16384.0f);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_oc(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhy_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhy_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f\r\n",
             callback_info->sensor_id,
             s,
             ns,
             data.x / 16384.0f,
             data.y / 16384.0f,
             data.z / 16384.0f,
             data.w / 16384.0f);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_ec(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhy_event_data_head_orientation_eul data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhy_event_data_head_orientation_eul_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f\r\n",
             callback_info->sensor_id,
             s,
             ns,
             (data.heading * 360.0f) / 32768.0f,
             (data.pitch * 360.0f) / 32768.0f,
             (data.roll * 360.0f) / 32768.0f);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {

        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to install virtual sensor callbacks
* @param[in] bhy         : Device instance
* @param[in] parse_table : Pointer to parse table
*/
void bhy_install_callbacks(struct bhy_dev *bhy, struct parse_ref *parse_table)
{
    for (uint8_t i = 0; i < BHY_MAX_SIMUL_SENSORS; i++)
    {
        parse_table->sensor[i].parse_flag = PARSE_FLAG_NONE;
        parse_table->sensor[i].id = 0;
    }

    (void)(bhy_register_fifo_parse_callback(BHY_SYS_ID_META_EVENT, parse_meta_event, parse_table, bhy));
    (void)(bhy_register_fifo_parse_callback(BHY_SYS_ID_META_EVENT_WU, parse_meta_event, parse_table, bhy));
    (void)(bhy_register_fifo_parse_callback(BHY_SYS_ID_DEBUG_MSG, parse_debug_message, parse_table, bhy));
}

/**
* @brief Function to reset Fuser2 core
* @param[in] bhy : Device instance
*/
static void reset_hub(struct bhy_dev *bhy)
{
    uint8_t data = 0, data_exp;

    (void)(bhy_soft_reset(bhy));

    (void)(bhy_get_host_interrupt_ctrl(&data, bhy));
    data &= ~BHY_ICTL_DISABLE_STATUS_FIFO; /* Enable status interrupts */
    data &= ~BHY_ICTL_DISABLE_DEBUG; /* Enable debug interrupts */
    data &= ~BHY_ICTL_EDGE; /* Level */
    data &= ~BHY_ICTL_ACTIVE_LOW; /* Active high */
    data &= ~BHY_ICTL_OPEN_DRAIN; /* Push-pull */
    data_exp = data;
    (void)(bhy_set_host_interrupt_ctrl(data, bhy));
    (void)(bhy_get_host_interrupt_ctrl(&data, bhy));
    if (data != data_exp)
    {
        WARNING("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
    }

    /* Config status channel */
    (void)(bhy_set_host_intf_ctrl(BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy));
    (void)(bhy_get_host_intf_ctrl(&data, bhy));
    if (!(data & BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL))
    {
        WARNING("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n",
                BHY_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    PRINT("Reset successful\r\n");
}

/**
* @brief Function to upload firmware to RAM
* @param[in] filepath : Path to firmware file
* @param[in] bhy      : Device instance
* @return Status for uploading
*/
static bool upload_to_ram(const char *filepath, struct bhy_dev *bhy)
{
    FILE *fw_file;
    struct stat st;
    uint8_t firmware_chunk[BHY2_RD_WR_LEN];
    uint32_t len;
    int8_t rslt = BHY_OK;
    uint8_t boot_status;
    uint32_t start_time_ms;

#ifdef PC
    uint8_t progress = 0, new_progress = 0;
#endif
    (void)(bhy_get_boot_status(&boot_status, bhy));
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        #if 0
        if ((boot_status & BHY_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY_BST_FLASH_VERIFY_DONE))
        {
        #endif
        if (boot_status & BHY_BST_HOST_FW_VERIFY_DONE)
        {
            ERROR("Seems like a firmware is running. Please reset the sensor before uploading firmware\r\n");

            return false;
        }

#ifdef PC
        fw_file = fopen(filepath, "rb"); /* Without the b, the file is read incorrectly */
#else
        fw_file = fopen(filepath, "r");
#endif

        if (!fw_file)
        {
            ERROR("Cannot open file: %s\r\n", filepath);

            return false;
        }

        (void)stat(filepath, &st);
        len = (uint32_t) st.st_size;

        /* 256 KB */
        if (len > 262144)
        {
            ERROR("Invalid RAM Size of %lu bytes\r\n", len);

            return false;
        }

        PRINT("Uploading %lu bytes of firmware to RAM\r\n", len);
        start_time_ms = coines_get_millis();

        /*lint -e774*/
        uint32_t incr = BHY2_RD_WR_LEN;
        if ((incr % 4) != 0)
        {
            incr = BHY_ROUND_WORD_LOWER(incr);
        }

        for (uint32_t i = 0; (i < len) && (rslt == BHY_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round off to higher 4 bytes */
                {
                    incr = BHY_ROUND_WORD_HIGHER(incr);
                }
            }

            (void)fread(firmware_chunk, 1, incr, fw_file);
            rslt = bhy_upload_firmware_to_ram_partly(firmware_chunk, len, i, incr, bhy);
#ifdef PC
            progress = (float)(i + incr) / (float)len * 100.0f;
            if (progress != new_progress)
            {
                INFO("Completed %u %%\r", progress);
                new_progress = progress;
            }

#endif
        }

        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        fclose(fw_file);

        if (rslt != BHY_OK)
        {
            ERROR("Firmware upload failed. Returned with error code: %d. %s\r\n", rslt, get_api_error(rslt));

            return false;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return false;
    }

    PRINT("Uploading firmware to RAM successful\r\n");

    return true;
}

/**
* @brief Function to print boot status
* @param[in] boot_status : Boot status value
*/
static void print_boot_status(uint8_t boot_status)
{
    PRINT("Boot Status : 0x%02x: ", boot_status);
#if 0
    if (boot_status & BHY_BST_FLASH_DETECTED)
    {
        PRINT("Flash detected. ");
    }

    if (boot_status & BHY_BST_FLASH_VERIFY_DONE)
    {
        PRINT("Flash verify done. ");
    }

    if (boot_status & BHY_BST_FLASH_VERIFY_ERROR)
    {
        PRINT("Flash verification failed. ");
    }

    if (boot_status & BHY_BST_NO_FLASH)
    {
        PRINT("No flash installed. ");
    }
#endif
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        PRINT("Host interface ready. ");
    }

    if (boot_status & BHY_BST_HOST_FW_VERIFY_DONE)
    {
        PRINT("Firmware verification done. ");
    }

    if (boot_status & BHY_BST_HOST_FW_VERIFY_ERROR)
    {
        PRINT("Firmware verification error. ");
    }

    if (boot_status & BHY_BST_HOST_FW_IDLE)
    {
        PRINT("Firmware halted. ");
    }

    PRINT("\r\n");
}

/**
* @brief Function to boot from RAM
* @param[in] bhy : Device instance
*/
static void boot_ram(struct bhy_dev *bhy)
{
    int8_t rslt;
    uint8_t feat_status;
    uint8_t boot_status, error_val;
    uint16_t tries = 100;

    PRINT("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy->hif.delay_us(10000, NULL);
        (void)(bhy_get_boot_status(&boot_status, bhy));
        if (boot_status & BHY_BST_HOST_FW_VERIFY_DONE)
        {
            break;
        }
    } while (tries--);

    print_boot_status(boot_status);
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        if (boot_status & BHY_BST_HOST_FW_VERIFY_DONE)
        {
            INFO("Booting from RAM\r\n");
            rslt = bhy_boot_from_ram(bhy);
            if (rslt != BHY_OK)
            {
                ERROR("Booting from RAM failed. API error code %d.\r\n%s\r\n", rslt, get_api_error(rslt));
                (void)(bhy_get_error_value(&error_val, bhy));
                if (error_val)
                {
                    ERROR("Sensor reports error 0x%X.\r\n%s", error_val, get_sensor_error_text(error_val));
                }

                return;
            }

            (void)(bhy_get_feature_status(&feat_status, bhy));
            if (feat_status & BHY_FEAT_STATUS_OPEN_RTOS_MSK)
            {
                (void)(bhy_update_virtual_sensor_list(bhy));

                for (uint16_t i = 0; i < 10; i++)
                {
                    /* Process meta events over a period of 100ms*/
                    (void)(bhy_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), bhy));
                    bhy->hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Feature status failed, booting from RAM failed\r\n");

                return;
            }
        }
        else
        {
            ERROR("Upload firmware to RAM before boot\r\n");

            return;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return;
    }

    PRINT("Booting from RAM successful\r\n");
}

/**
* @brief Function to print information about state of device and sensors
* @param[in] bhy : Device instance
*/
static void show_info(struct bhy_dev *bhy)
{
    uint16_t kernel_version = 0, user_version = 0;
    uint16_t rom_version = 0;
    uint8_t product_id = 0;
    uint8_t host_status = 0, feat_status = 0;
    uint8_t val = 0;
    uint8_t sensor_error;
    struct bhy_virtual_sensor_info_param_info info;

    /* Get product_id */
    (void)(bhy_get_product_id(&product_id, bhy));

    /* Get Kernel version */
    (void)(bhy_get_kernel_version(&kernel_version, bhy));

    /* Get User version */
    (void)(bhy_get_user_version(&user_version, bhy));

    /* Get ROM version */
    (void)(bhy_get_rom_version(&rom_version, bhy));

    (void)(bhy_get_host_status(&host_status, bhy));

    (void)(bhy_get_feature_status(&feat_status, bhy));

    PRINT("Product ID     : %02x\r\n", product_id);
    PRINT("Kernel version : %04u\r\n", kernel_version);
    PRINT("User version   : %04u\r\n", user_version);
    PRINT("ROM version    : %04u\r\n", rom_version);
    PRINT("Power state    : %s\r\n", (host_status & BHY_HST_POWER_STATE) ? "sleeping" : "active");
    PRINT("Host interface : %s\r\n", (host_status & BHY_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
    PRINT("Feature status : 0x%02x\r\n", feat_status);

    /* Read boot status */
    (void)(bhy_get_boot_status(&val, bhy));
    print_boot_status(val);

    /* Read error value */
    (void)(bhy_get_error_value(&sensor_error, bhy));
    if (sensor_error)
    {
        ERROR("%s\r\n", get_sensor_error_text(sensor_error));
    }

    if (feat_status & BHY_FEAT_STATUS_OPEN_RTOS_MSK)
    {
        (void)bhy_update_virtual_sensor_list(bhy);

        /* Get present virtual sensor */
        (void)bhy_system_param_get_virtual_sensor_present(bhy);

        PRINT("Virtual sensor list.\r\n");
        PRINT("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\r\n");
        PRINT("----------+--------------------------------------+-----+-----+-----------+-----------|\r\n");
        for (uint8_t i = 0; i < BHY_SENSOR_ID_MAX; i++)
        {
            if (bhy_is_sensor_available(i, bhy))
            {
                if (i < BHY_SENSOR_ID_CUSTOM_START)
                {
                    PRINT(" %8u | %36s ", i, get_sensor_name(i));
                }
                else
                {
                    PRINT(" %8u | %36s ", i, custom_driver_information[i - BHY_SENSOR_ID_CUSTOM_START].sensor_name);
                }

                (void)(bhy_get_sensor_info(i, &info, bhy));
                PRINT("| %3u | %3u | %9.4f | %9.4f |\r\n",
                      info.driver_id,
                      info.driver_version,
                      info.min_rate.f_val,
                      info.max_rate.f_val);
            }
        }
    }
}

#if 0

/**
* @brief Function to boot from Flash
* @param[in] bhy : Device instance
*/
static void boot_flash(struct bhy_dev *bhy)
{
    int8_t rslt;
    uint8_t boot_status, feat_status;
    uint8_t error_val = 0;
    uint16_t tries = 300; /* Wait for up to little over 3s */

    PRINT("Waiting for firmware verification to complete\r\n");
    do
    {
        bhy->hif.delay_us(10000, NULL);
        (void)(bhy_get_boot_status(&boot_status, bhy));
        if (boot_status & BHY_BST_FLASH_VERIFY_DONE)
        {
            break;
        }
    } while (tries--);

    (void)(bhy_get_boot_status(&boot_status, bhy));
    print_boot_status(boot_status);
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        if (boot_status & BHY_BST_FLASH_DETECTED)
        {

            /* If no firmware is running, boot from Flash */
            PRINT("Booting from flash\r\n");
            rslt = bhy_boot_from_flash(bhy);
            if (rslt != BHY_OK)
            {
                ERROR("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                (void)(bhy_get_regs(BHY_REG_ERROR_VALUE, &error_val, 1, bhy));
                if (error_val)
                {
                    ERROR("%s\r\n", get_sensor_error_text(error_val));
                }

                return;
            }

            (void)(bhy_get_boot_status(&boot_status, bhy));
            print_boot_status(boot_status);

            if (!(boot_status & BHY_BST_HOST_INTERFACE_READY))
            {
                /* hub is not ready, need reset hub */
                PRINT("Host interface is not ready, triggering a reset\r\n");

                (void)(bhy_soft_reset(bhy));
            }

            (void)(bhy_get_feature_status(&feat_status, bhy));
            if (feat_status & BHY_FEAT_STATUS_OPEN_RTOS_MSK)
            {
                (void)(bhy_update_virtual_sensor_list(bhy));
                for (uint16_t i = 0; i < 10; i++) /* Process meta events */
                {
                    (void)(bhy_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), bhy));
                    bhy->hif.delay_us(10000, NULL);
                }
            }
            else
            {
                ERROR("Reading Feature status failed, booting from flash failed\r\n");

                return;
            }
        }
        else
        {
            ERROR("Can't detect external flash\r\n");

            return;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");

        return;
    }

    PRINT("Booting from flash successful\r\n");
}

/**
* @brief Function to upload firmware to Flash
* @param[in] filepath : Path to firmware file
* @param[in] bhy      : Device instance
* @return Status for uploading
*/
static bool upload_to_flash(const char *filepath, struct bhy_dev *bhy)
{
    FILE *fw_file;
    struct stat st;
    uint8_t firmware_chunk[BHY2_RD_WR_LEN];
    uint32_t len;
    int8_t rslt = BHY_OK;
    uint8_t boot_status;
    uint32_t start_time_ms;

#ifdef PC
    uint8_t progress = 0, new_progress = 0;
#endif
    (void)(bhy_get_boot_status(&boot_status, bhy));
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Reset the sensor before uploading firmware to external flash\r\n");

            return false;
        }

#ifdef PC
        fw_file = fopen(filepath, "rb"); /* Without the b, the file is read incorrectly */
#else
        fw_file = fopen(filepath, "r");
#endif
        if (!fw_file)
        {
            ERROR("Cannot open file: %s\r\n", filepath);

            return false;
        }

        (void)stat(filepath, &st);
        len = (uint32_t) st.st_size;

        /* 8 MB */
        if (len > 8388608)
        {
            ERROR("Invalid firmware size of %lu bytes\r\n", len);

            return false;
        }

        PRINT("Erasing first %lu bytes of flash\r\n", len);
        rslt = bhy_erase_flash(BHY_FLASH_SECTOR_START_ADDR, BHY_FLASH_SECTOR_START_ADDR + len, bhy);
        if (rslt != BHY_OK)
        {
            ERROR("Erasing flash failed with error code %d. %s\r\n", rslt, get_api_error(rslt));
        }

        PRINT("Uploading %lu bytes of firmware to flash\r\n", len);
        start_time_ms = coines_get_millis();
        uint32_t incr = BHY_RD_WR_LEN;
        if ((incr % 4) != 0) /* Round of to lower 4 bytes */
        {
            incr = BHY_ROUND_WORD_LOWER(incr);
        }

        for (uint32_t i = 0; (i < len) && (rslt == BHY_OK); i += incr)
        {
            if (incr > (len - i)) /* Last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round of to higher 4bytes */
                {
                    incr = BHY_ROUND_WORD_HIGHER(incr);
                }
            }

            (void)fread(firmware_chunk, 1, incr, fw_file);
            rslt = bhy_upload_firmware_to_flash_partly(firmware_chunk, i, incr, bhy);
#ifdef PC
            progress = (float)(i + incr) / (float)len * 100.0f;
            if (progress != new_progress)
            {
                INFO("Completed %u %%\r", progress);
                new_progress = progress;
            }

#endif
        }

        INFO("Firmware upload took %.2f seconds\r\n", (float)(coines_get_millis() - start_time_ms) / 1000.0f);
        fclose(fw_file);
        if (rslt != BHY_OK)
        {
            ERROR("%s. Firmware upload failed\r\n", get_api_error(rslt));

            return false;
        }
    }
    else
    {
        ERROR("Host interface is not ready\r\n");
    }

    PRINT("Uploading firmware to flash successful\r\n");

    return true;
}
#endif

/**
* @brief Function to write value to register
* @param[in] payload : Data to write, including register address
* @param[in] bhy     : Device instance
*/
static void wr_regs(const char *payload, struct bhy_dev *bhy)
{
    char *start;
    char *end;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024] = { 0 };
    char *strtok_ptr;
    char *byte_delimiter = ",";
    uint16_t len = 0;

    /* Parse register address */
    start = (char *)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write address format error\r\n");

        return;
    }

    strncpy(str_reg, start, (size_t)(end - start));
    str_reg[end - start] = '\0';
    reg = (uint8_t)strtol(str_reg, NULL, 0);

    /* Parse values to be written */
    start = end + 1;
    strtok_ptr = strtok(start, byte_delimiter);

    while (strtok_ptr != NULL)
    {
        val[len] = (uint8_t)strtol(strtok_ptr, NULL, 0);
        strtok_ptr = strtok(NULL, byte_delimiter);
        ++len;
    }

    /* Execution of bus write function */
    (void)(bhy_set_regs(reg, val, len, bhy));

    PRINT("Writing address successful\r\n");
}

/**
* @brief Function to read value from register
* @param[in] payload : Data includes register address and length to read
* @param[in] bhy     : Device instance
*/
static void rd_regs(const char *payload, struct bhy_dev *bhy)
{
    char *start;
    char *end;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024];
    uint16_t len;
    uint16_t i = 0;
    uint16_t j = 0;

    start = (char *)payload;
    end = strchr(start, ':');
    if (end == NULL)
    {
        end = start + strlen(start);
    }

    /* Parse register address */
    strncpy(str_reg, start, (size_t)(end - start));
    str_reg[end - start] = '\0';
    reg = (uint8_t)strtol(str_reg, NULL, 0);

    /* Parse read length, rest of the string */
    start = end + 1;
    len = (uint16_t)strtol(start, NULL, 0);

    /* Default read length is 1 */
    if (len < 1)
    {
        len = 1;
    }

    /* Execution of bus read function */
    (void)(bhy_get_regs(reg, val, len, bhy));

    /* Print register data to console */

    /* Registers after the status channel are auto increment,
     * reading more than 1 byte will lead to reading
     * the subsequent register addresses */
    if (reg <= BHY_REG_CHAN_STATUS)
    {
        PRINT("Reading from register address 0x%02x:\r\n", reg);
        PRINT("Byte hex       dec | Data\r\n");
        PRINT("-------------------------------------------\r\n");
        for (i = 0; i < len; i++)
        {
            if (j == 0)
            {
                PRINT(" ");
                PRINT("0x%06x %8d |", i, i);
            }

            PRINT(" %02x", val[i]);
            ++j;
            if (j >= 8)
            {
                PRINT("\r\n");
                j = 0;
            }
        }

        if ((len % 8) == 0)
        {
            PRINT("\r\n");
        }
    }
    else
    {
        PRINT("Register address: Data\r\n");
        PRINT("----------------------\r\n");
        for (i = 0; i < len; i++)
        {
            PRINT("0x%02x            : %02x \r\n", reg + i, val[i]);
        }
    }

    PRINT("Read complete\r\n");
}

/**
* @brief Function to read value from parameter
* @param[in] payload : Data includes parameter ID
* @param[in] bhy     : Device instance
*/
static void rd_param(const char *payload, struct bhy_dev *bhy)
{
    char str_param_id[8] = { 0 };
    uint8_t tmp_buf[1024] = { 0 };
    uint16_t param_id;
    uint32_t ret_len = 0;
    uint16_t i;
    uint16_t j = 0;

    strncpy(str_param_id, payload, strlen(payload));
    str_param_id[strlen(payload)] = '\0';
    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    (void)(bhy_get_parameter(param_id, tmp_buf, sizeof(tmp_buf), &ret_len, bhy));

    PRINT("Byte hex      dec | Data\r\n");
    PRINT("-------------------------------------------\r\n");
    for (i = 0; i < ret_len; i++)
    {
        if (j == 0)
        {
            PRINT("0x%06x %8d |", i, i);
        }

        PRINT("%02x ", tmp_buf[i]);
        j++;
        if (j >= 8)
        {
            PRINT("\r\n");
            j = 0;
        }
    }

    if ((ret_len % 8) != 0)
    {
        PRINT("\r\n");
    }

    PRINT("Reading parameter 0x%04X successful\r\n", param_id);
}

/**
* @brief Function to write value to parameter
* @param[in] payload : Data to write, including parameter ID
* @param[in] bhy     : Device instance
*/
static void wr_param(const char *payload, struct bhy_dev *bhy)
{
    char *start, *end;
    char str_param_id[8] = { 0 };
    char str_data[8] = { 0 };
    uint8_t data_buf[1024] = { 0 };
    uint16_t param_id;
    uint8_t val;
    uint16_t buf_size = 0;
    uint8_t break_out = 0;

    start = (char *)payload;
    end = strchr(start, '=');
    if (end == NULL)
    {
        ERROR("Write parameter I/O format error\r\n");

        return;
    }

    strncpy(str_param_id, start, (size_t)(end - start));
    str_param_id[end - start] = '\0';
    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    /* Parse write data */
    do
    {
        start = end + 1;
        end = strchr(start, ',');
        if (end == NULL)
        {
            end = start + strlen(start);
            break_out++;
        }

        strncpy(str_data, start, (size_t)(end - start));
        str_data[end - start] = '\0';
        val = (uint8_t)strtol(str_data, NULL, 0);
        data_buf[buf_size] = val;
        buf_size++;
    } while (!break_out);

    /* Make sure write buffer size is always multiples of 4 */
    if (buf_size % 4)
    {
        buf_size = (uint16_t)((buf_size / 4 + 1) * 4);
    }

    (void)(bhy_set_parameter(param_id, data_buf, buf_size, bhy));
    PRINT("Writing parameter successful\r\n");

}

/**
* @brief Function to read physical sensor information
* @param[in] payload : Data includes sensor ID
* @param[in] bhy     : Device instance
*/
static void rd_phy_sensor_info(const char *payload, struct bhy_dev *bhy)
{
    uint16_t param_id;
    uint8_t sens_id;
    struct bhy_system_param_phys_sensor_info psi = { 0 };

    sens_id = (uint8_t)atoi((char *)&payload[0]);
    param_id = (uint16_t)(0x0120 | sens_id);

    if (param_id >= 0x0121 && param_id <= 0x0160)
    {
        (void)(bhy_system_param_get_physical_sensor_info(sens_id, &psi, bhy));

        PRINT("Field Name            hex                    | Value (dec)\r\n");
        PRINT("----------------------------------------------------------\r\n");
        PRINT("Physical Sensor ID    %02X                     | %d\r\n", psi.sensor_type, psi.sensor_type);
        PRINT("Driver ID             %02X                     | %d\r\n", psi.driver_id, psi.driver_id);
        PRINT("Driver Version        %02X                     | %d\r\n", psi.driver_version, psi.driver_version);
        PRINT("Current Consumption   %02X                     | %0.3fmA\r\n",
              psi.power_current,
              psi.power_current / 10.f);
        PRINT("Dynamic Range         %04X                   | %d\r\n", psi.curr_range.u16_val, psi.curr_range.u16_val);

        const char *irq_status[2] = { "Disabled", "Enabled" };
        const char *master_intf[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
        const char *power_mode[8] =
        { "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
          "Low Power Active", "Active" };

        PRINT("Flags                 %02X                     | IRQ status       : %s\r\n", psi.flags,
              irq_status[psi.flags & 0x01]);
        PRINT("                                             | Master interface : %s\r\n",
              master_intf[(psi.flags >> 1) & 0x0F]);
        PRINT("                                             | Power mode       : %s\r\n",
              power_mode[(psi.flags >> 5) & 0x07]);
        PRINT("Slave Address         %02X                     | %d\r\n", psi.slave_address, psi.slave_address);
        PRINT("GPIO Assignment       %02X                     | %d\r\n", psi.gpio_assignment, psi.gpio_assignment);
        PRINT("Current Rate          %08X               | %.3fHz\r\n", psi.curr_rate.u32_val, psi.curr_rate.f_val);
        PRINT("Number of axes        %02X                     | %d\r\n", psi.num_axis, psi.num_axis);

        #define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))
        struct bhy_system_param_orient_matrix ort_mtx;
        ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
        ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] & 0xF0);
        ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
        ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] & 0xF0);
        ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
        ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] & 0xF0);
        ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
        ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] & 0xF0);
        ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);

        PRINT("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\r\n",
              psi.orientation_matrix[0],
              psi.orientation_matrix[1],
              psi.orientation_matrix[2],
              psi.orientation_matrix[3],
              psi.orientation_matrix[4],
              ort_mtx.c[0],
              ort_mtx.c[1],
              ort_mtx.c[2]);
        PRINT("                                             | %+02d %+02d %+02d |\r\n",
              ort_mtx.c[3],
              ort_mtx.c[4],
              ort_mtx.c[5]);
        PRINT("                                             | %+02d %+02d %+02d |\r\n",
              ort_mtx.c[6],
              ort_mtx.c[7],
              ort_mtx.c[8]);
        PRINT("Reserved              %02X                     | %d\r\n", psi.reserved, psi.reserved);
        PRINT("\r\n");

    }
}

#if 0

/**
* @brief Function to erase flash
* @param[in] end_addr : End address
* @param[in] bhy      : Device instance
*/
static void erase_flash(uint32_t end_addr, struct bhy_dev *bhy)
{
    int8_t rslt;
    uint8_t boot_status;

    (void)(bhy_get_boot_status(&boot_status, bhy));
    if (boot_status & BHY_BST_HOST_INTERFACE_READY)
    {
        if ((boot_status & BHY_BST_HOST_FW_VERIFY_DONE) || (boot_status & BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Seems like a firmware is running. Reset the sensor before erasing external flash\r\n");

            return;
        }
    }

    PRINT("Erasing flash. May take a while\r\n");
    rslt = bhy_erase_flash(BHY_FLASH_SECTOR_START_ADDR, BHY_FLASH_SECTOR_START_ADDR + end_addr, bhy);
    if (rslt != BHY_OK)
    {
        ERROR("Erasing flash failed, status: %02d\r\n", rslt);

        return;
    }

    PRINT("Erasing flash successful\r\n");
}

#endif

#define MAXIMUM_VIRTUAL_SENSOR_LIST  UINT16_C(256)
static uint16_t sensor_list[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0 };
static struct bhy_virtual_sensor_conf_param_conf sen_cfg_list[MAXIMUM_VIRTUAL_SENSOR_LIST] = { { 0 } };
static bhy_float ds_list[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0.0f };
static uint16_t num_sensor = 0;
static bool lognstream_inprogress = false;
static char file_name[50] = { '\0' };

bool enable_ds[256] = { false };
uint16_t odr_ds[256] = { 0 };

/**
* @brief Function to activate sensor
* @param[in] sensor_parameters : Sensor parameters
* @param[in] parse_flag        : Parse flag
* @param[in] ref               : Reference to device and command line
*/
static void activate_sensor(const char *sensor_parameters, uint8_t parse_flag, struct bhy_cli_ref *ref)
{
    char sen_id_str[8], sample_rate_str[8], sen_latency_str[8], downstream_str[8];
    uint8_t sen_id;

    sensor_conf.latency = 0;
    char *start, *end;

    /* Parse Sensor ID */
    start = (char *)sensor_parameters;
    end = strchr(start, ':');
    if (end == NULL)
    {
        ERROR("Sensor ID / Sample rate format error\r\n");

        return;
    }

    strncpy(sen_id_str, start, (size_t)(end - start));
    sen_id_str[end - start] = '\0';
    sen_id = (uint8_t)atoi(sen_id_str);

    /* Parse sample rate */
    start = end + 1;
    end = strchr(start, ':');

    if (end == NULL)
    {
        end = start + strlen(start);
    }

    strncpy(sample_rate_str, start, (size_t)(end - start));
    sample_rate_str[end - start] = '\0';
    sensor_conf.sample_rate = (float)atof(sample_rate_str);

    if (sensor_conf.sample_rate < 0)
    {
        sensor_conf.sample_rate = 0.0f;
    }

    /*  Parse Latency */
    if (strlen(end))
    {
        start = end + 1;
        end = strchr(start, ':');

        if (end == NULL)
        {
            end = start + strlen(start);
        }

        strncpy(sen_latency_str, start, (size_t)(end - start));
        sen_latency_str[end - start] = '\0';
        sensor_conf.latency = (uint32_t)atoi(sen_latency_str);
    }

    /* Parse downstream rate*/
    ds_list[sen_id] = 0.0f;
    enable_ds[sen_id] = false;
    odr_ds[sen_id] = 0;

    if (strlen(end))
    {
        start = end + 1;
        end = strchr(start, ':');

        if (end == NULL)
        {
            end = start + strlen(start);
        }

        strncpy(downstream_str, start, (size_t)(end - start));
        downstream_str[end - start] = '\0';
        ds_list[sen_id] = (float)atof(downstream_str);

        if (ds_list[sen_id] > 0.0f && sensor_conf.sample_rate > ds_list[sen_id])
        {
            enable_ds[sen_id] = true;
            odr_ds[sen_id] = round(sensor_conf.sample_rate / ds_list[sen_id]);
        }
    }

    configure_sensor(sensor_conf, sen_id, parse_flag, ref);
}

/**
* @brief Default function to parse for custom sensor
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_custom_sensor_default(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t this_sensor_id;
    uint8_t this_sensor_payload;
    uint8_t parse_flag;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    struct parse_sensor_details *sensor_details;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    this_sensor_id = callback_info->sensor_id;
    this_sensor_payload =
        (uint8_t)custom_driver_information[this_sensor_id - BHY_SENSOR_ID_CUSTOM_START].sensor_payload;

    if (this_sensor_payload > callback_info->data_size)
    {
        ERROR("Mismatch in payload size\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (parse_flag & PARSE_FLAG_STREAM)
    {
        /* Print sensor ID */
        DATA("SID: %u; T: %lu.%09lu; ", this_sensor_id, s, ns);

        for (uint16_t i = 0; i < this_sensor_payload - 1; i++)
        {
            /* Output raw data in hex */
            PRINT_D("%x ", callback_info->data_ptr[i]);
        }

        PRINT_D("\r\n");
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for custom sensor
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_custom_sensor(const struct bhy_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t idx;
    char *strtok_ptr;
    char *parameter_delimiter = ":";
    char tmp_output_formats[BHYCLI_MAX_STRING_LENGTH];
    uint8_t rel_sensor_id; /* Relative sensor ID  */
    uint32_t s, ns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    union bhy_float_conv offset;

    uint8_t tmp_u8 = 0;
    uint16_t tmp_u16 = 0;
    uint32_t tmp_u32 = 0;
    int8_t tmp_s8 = 0;
    int16_t tmp_s16 = 0;
    int32_t tmp_s32 = 0;
    uint8_t tmp_data_c = 0;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    /* Get sensor_id to access correct parsing information from global linked list  */
    rel_sensor_id = callback_info->sensor_id - BHY_SENSOR_ID_CUSTOM_START;

    /* Fetch output_formats string from linked list */
    strcpy(tmp_output_formats, custom_driver_information[rel_sensor_id].output_formats);
    strtok_ptr = strtok(tmp_output_formats, parameter_delimiter);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    if ((custom_driver_information[rel_sensor_id].sensor_payload + 1) != callback_info->data_size)
    {
        ERROR("Mismatch in payload size\r\n");

        return;
    }

    idx = 0;
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        /* Print sensor id and timestamp */
        DATA("%u; %lu.%09lu; ", callback_info->sensor_id, s, ns);

        /* Parse output_formats and output data depending on the individual format of an output */

        while (strtok_ptr != NULL)
        {

            if (strcmp(strtok_ptr, "u8") == 0)
            {
                tmp_u8 = callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%u ", tmp_u8);
            }
            else if (strcmp(strtok_ptr, "u16") == 0)
            {
                tmp_u16 = BHY_LE2U16(&callback_info->data_ptr[idx]);
                idx += 2;

                PRINT_D("%u ", tmp_u16);
            }
            else if (strcmp(strtok_ptr, "u32") == 0)
            {
                tmp_u32 = BHY_LE2U32(&callback_info->data_ptr[idx]);
                idx += 4;

                PRINT_D("%lu ", tmp_u32);
            }
            else if (strcmp(strtok_ptr, "s8") == 0)
            {
                tmp_s8 = (int8_t)callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%d ", tmp_s8);
            }
            else if (strcmp(strtok_ptr, "s16") == 0)
            {
                tmp_s16 = BHY_LE2S16(&callback_info->data_ptr[idx]);
                idx += 2;

                PRINT_D("%d ", tmp_s16);
            }
            else if (strcmp(strtok_ptr, "s32") == 0)
            {
                tmp_s32 = BHY_LE2S32(&callback_info->data_ptr[idx]);
                idx += 4;

                PRINT_D("%ld ", tmp_s32);
            }
            else if (strcmp(strtok_ptr, "c") == 0)
            {
                tmp_data_c = callback_info->data_ptr[idx];
                idx += 1;

                PRINT_D("%c ", tmp_data_c);
            }
            else if (strcmp(strtok_ptr, "f") == 0)
            {
                /* Float values have to be read as unsigned and then interpreted as float */
                offset.u32_val = BHY_LE2U32(&callback_info->data_ptr[idx]);
                idx += 4;

                /* The binary data has to be interpreted as a float */
                PRINT_D("%6.4f ", offset.f_val);
            }

            strtok_ptr = strtok(NULL, parameter_delimiter);
        }

        PRINT_D("\r\n");
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 timestamp,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }

    if (idx != custom_driver_information[rel_sensor_id].sensor_payload)
    {
        ERROR("Provided Output format sizes don't add up to total sensor payload!\r\n");

        return;
    }
}

/**
* @brief Function to add a sensor
* @param[in] payload : Data contains sensor information
* @param[in] ref     : Reference to device and command line
*/
static void add_sensor(const char *payload, struct bhy_cli_ref *cli_ref)
{
    struct bhy_dev *bhy = &cli_ref->bhy;
    struct parse_ref *parse_table = &cli_ref->parse_table;
    char *start;
    char *end;
    char str_sensor_id[BHYCLI_MAX_STRING_LENGTH];
    char str_sensor_payload[BHYCLI_MAX_STRING_LENGTH];
    char output_formats[BHYCLI_MAX_STRING_LENGTH];
    char sensor_name[BHYCLI_MAX_STRING_LENGTH];
    uint8_t sensor_id;
    uint8_t sensor_payload;
    uint8_t len_of_output_formats;
    struct bhy_virtual_sensor_info_param_info sensor_info;

    start = (char *)payload;
    end = strchr(start, ':');

    if (end == NULL)
    {
        ERROR("Add Sensor format error\r\n");

        return;
    }

    /* Parse sensor ID */

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHYCLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor ID!\r\n");

        return;
    }

    strncpy(str_sensor_id, start, (size_t)(end - start));
    str_sensor_id[end - start] = '\0';

    /* Convert string to int */
    sensor_id = (uint8_t)strtol(str_sensor_id, NULL, 10);
    INFO("Sensor ID: %u \r\n", sensor_id);

    /* Parse sensor name */
    start = end + 1;
    end = strchr(start, ':');
    if (end == NULL)
    {
        ERROR("Add Sensor name error\r\n");

        return;
    }

    /* Check length of string */
    if (((uint32_t)(end - start)) > BHYCLI_MAX_STRING_LENGTH)
    {
        ERROR("Too many characters for sensor name. Only %u characters allowed\r\n", BHYCLI_MAX_STRING_LENGTH);

        return;
    }

    strncpy(sensor_name, start, (size_t)(end - start));
    sensor_name[end - start] = '\0';
    INFO("Sensor Name: %s \r\n", sensor_name);

    /* Parse sensor payload */
    start = end + 1;
    end = strchr(start, ':');

    if (end == NULL)
    {
        ERROR("Add Sensor payload error\r\n");

        return;
    }

    strncpy(str_sensor_payload, start, (size_t)(end - start));
    str_sensor_payload[end - start] = '\0';
    sensor_payload = (uint8_t)strtol(str_sensor_payload, NULL, 10);
    INFO("Sensor Payload: %u \r\n", sensor_payload);

    /* Parse output formats string, final parsing of each output is done in the parsing callback function */
    start = end + 1;
    len_of_output_formats = (uint8_t)strlen(start);
    end = start + len_of_output_formats;
    strncpy(output_formats, start, (size_t)((size_t)(end - start)));
    output_formats[end - start] = '\0';

    /* Get the sensor information */
    (void)(bhy_get_sensor_info(sensor_id, &sensor_info, bhy));

    /* Check if supplied payload matches the event size. Note event size includes sensor id in the payload */
    if (sensor_info.event_size != (sensor_payload + 1))
    {
        ERROR("Provided total payload size of sensor ID %u doesn't match the actual payload size!\r\n", sensor_id);

        return;
    }

    /* Store parsed data into the custom driver information table */
    custom_driver_information[sensor_id - BHY_SENSOR_ID_CUSTOM_START].sensor_id = sensor_id;
    custom_driver_information[sensor_id - BHY_SENSOR_ID_CUSTOM_START].sensor_payload = sensor_payload;
    strncpy(custom_driver_information[sensor_id - BHY_SENSOR_ID_CUSTOM_START].output_formats,
            output_formats,
            BHYCLI_MAX_STRING_LENGTH);
    custom_driver_information[sensor_id - BHY_SENSOR_ID_CUSTOM_START].is_registered = 1;
    strncpy(custom_driver_information[sensor_id - BHY_SENSOR_ID_CUSTOM_START].sensor_name,
            sensor_name,
            BHYCLI_MAX_STRING_LENGTH);

    /* Register the custom sensor callback function*/
    (void)(bhy_register_fifo_parse_callback(sensor_id, parse_custom_sensor, parse_table, bhy));

    PRINT("Adding custom driver payload successful\r\n");
}

#if 0

/**
* @brief Function to enable Klio
* @param[in] bhy     : Device instance
*/
static void klio_enable(struct bhy_dev *bhy)
{
    if (!klio_enabled)
    {
        uint8_t buf[255];
        uint16_t size = sizeof(buf);
        uint32_t klio_driver_status;

        int major = -1;
        int minor = -1;
        int version = -1;
        int count = 0;

        (void)(bhy_klio_param_get_parameter(KLIO_PARAM_ALGORITHM_VERSION, buf, &size, bhy));
        (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

        if (size > 0)
        {
            count = sscanf((char *)buf, "%d.%d.%d", &major, &minor, &version);
        }

        if (major < 0 || minor < 0 || version < 0 || count != 3)
        {
            PRINT("Unable to get Klio firmware version.\r\n");
            (void)(BHY_E_MAGIC); /* Invalid firmware error */
        }

        if (major != 3)
        {
            PRINT("The supported Klio firmware is version 3.x.x.\r\n");
            (void)(BHY_E_MAGIC); /* Invalid firmware error */
        }

        (void)(bhy_klio_param_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, buf, &size, bhy));
        (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

        memcpy(&klio_vars.max_cyclic_patterns, buf, 1);

        size = sizeof(buf);

        (void)(bhy_klio_param_get_parameter(KLIO_PARAM_PATTERN_BLOB_SIZE, buf, &size, bhy));
        (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

        memcpy(&klio_vars.max_cyclic_pattern_blob_size, buf, 1);

        klio_vars.auto_load_pattern_write_index = 0;

        if (minor >= 1)
        {
            /* Support for generic gestures was added in version 3.1.0 of Klio. */
            (void)(bhy_klio_param_get_parameter(KLIO_PARAM_MAX_NUM_PATTERNS_GENERIC, buf, &size, bhy));
            (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

            memcpy(&klio_vars.max_generic_patterns, buf, 1);

            (void)(bhy_klio_param_get_parameter(KLIO_PARAM_MAX_BLOB_SIZE_GENERIC, buf, &size, bhy));
            (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));

            memcpy(&klio_vars.max_generic_pattern_blob_size, buf, 2);
        }

        klio_enabled = true;
    }
}

/**
* @brief Function to get Klio status
* @param[in] bhy : Device instance
*/
static void klio_status(struct bhy_dev *bhy)
{
    uint32_t klio_driver_status;

    (void)(bhy_klio_param_read_reset_driver_status(&klio_driver_status, bhy));
    INFO("[kstatus] ");
    PRINT("Status: %lu\r\n", klio_driver_status);
}

/**
* @brief Function to set state of cyclic Klio
* @param[in] arg1 : Argument for learning status
* @param[in] arg2 : Argument for learning reset
* @param[in] arg3 : Argument for recognition status
* @param[in] arg4 : Argument for recognition reset
* @param[in] bhy  : Device instance
*/
static void klio_set_state(const char *arg1, const char *arg2, const char *arg3, const char *arg4, struct bhy_dev *bhy)
{
    bhy_klio_param_sensor_state_t state = { 0 };

    state.learning_enabled = (uint8_t)atoi(arg1);
    state.learning_reset = (uint8_t)atoi(arg2);
    state.recognition_enabled = (uint8_t)atoi(arg3);
    state.recognition_reset = (uint8_t)atoi(arg4);

    INFO("[ksetstate] ");
    PRINT("Learning enabled    : %u\r\n", state.learning_enabled);
    INFO("[ksetstate] ");
    PRINT("Learning reset      : %u\r\n", state.learning_reset);
    INFO("[ksetstate] ");
    PRINT("Recognition enabled : %u\r\n", state.recognition_enabled);
    INFO("[ksetstate] ");
    PRINT("Recognition reset   : %u\r\n", state.recognition_reset);
    (void)(bhy_klio_param_set_state(&state, bhy));
}

/**
* @brief Function to get state of cyclic Klio
* @param[in] bhy  : Device instance
*/
static void klio_get_state(struct bhy_dev *bhy)
{
    bhy_klio_param_sensor_state_t state;

    (void)(bhy_klio_param_get_state(&state, bhy));
    INFO("[kgetstate] ");
    PRINT("Learning enabled    : %u\r\n", state.learning_enabled);
    INFO("[kgetstate] ");
    PRINT("Learning reset      : %u\r\n", state.learning_reset);
    INFO("[kgetstate] ");
    PRINT("Recognition enabled : %u\r\n", state.recognition_enabled);
    INFO("[kgetstate] ");
    PRINT("Recognition reset   : %u\r\n", state.recognition_reset);
}

/**
* @brief Function to set state of generic recognition
* @param[in] arg  : Argument for gestures status
* @param[in] bhy  : Device instance
*/
static void klio_set_generic_recognition_state(const char *arg, struct bhy_dev *bhy)
{
    bhy_klio_param_generic_sensor_state_t state = { 0 };

    state.gestures_enabled = (uint8_t)atoi(arg);

    INFO("[ksetgstate] ");
    PRINT("Generic recognition enabled : %u\r\n", state.gestures_enabled);
    (void)(bhy_klio_param_set_generic_recognition_state(&state, bhy));
}

/**
* @brief Function to get state of generic recognition
* @param[in] bhy  : Device instance
*/
static void klio_get_generic_recognition_state(struct bhy_dev *bhy)
{
    bhy_klio_param_generic_sensor_state_t state;

    (void)(bhy_klio_param_get_generic_recognition_state(&state, bhy));
    INFO("[kgetgstate] ");
    PRINT("Gesture detection enabled : %u\r\n", state.gestures_enabled);
    INFO("[kgetgstate] ");
    PRINT("Timing detection enabled  : %u\r\n", state.timing_enabled);
}

static int32_t hex_to_char(char c)
{
    c = (char)tolower(c);

    if (c >= '0' && c <= '9')
    {
        c -= '0';
    }
    else if (c >= 'a' && c <= 'f')
    {
        c -= 'a' - 10;
    }
    else
    {
        return -1;
    }

    return c;
}

/*lint -e701*/
static int32_t pattern_blob_to_bytes(const uint8_t *pattern_blob_char, uint8_t *pattern_blob)
{
    int length = (int)(strlen((char *)pattern_blob_char));

    for (int i = 0; i < length; i += 2)
    {
        int32_t u = hex_to_char((char)pattern_blob_char[i]);
        int32_t l = hex_to_char((char)pattern_blob_char[i + 1]);

        if (u < 0 || l < 0)
        {
            return -1;
        }

        pattern_blob[i / 2] = (uint8_t)((((char)u) << 4) | (char)l);
    }

    return length / 2;
}

/**
* @brief Function to load cyclic pattern
* @param[in] arg1 : Argument for pattern index
* @param[in] arg2 : Argument for pattern/adaptive pattern
* @param[in] bhy  : Device instance
*/
static void klio_load_cyclic_pattern(const char *arg1, const char *arg2, struct bhy_dev *bhy)
{
    uint8_t pattern_data[244];
    uint16_t size = (uint16_t)pattern_blob_to_bytes((uint8_t *)arg2, (uint8_t *)pattern_data);
    uint8_t loop = (uint8_t)atoi(arg1);

    if (size <= klio_vars.max_cyclic_pattern_blob_size)
    {
        if (loop < klio_vars.max_cyclic_patterns)
        {
            (void)(bhy_klio_param_write_pattern(loop, pattern_data, size, bhy));

            if (loop >= klio_vars.auto_load_pattern_write_index)
            {
                klio_vars.auto_load_pattern_write_index = loop + 1;
            }
        }
        else
        {
            INFO("[kldpatt] ");
            ERROR("Pattern index: %d >= Max patterns %d\r\n", loop, klio_vars.max_cyclic_patterns);
            (void)kldpatt_help(NULL);

            return;
        }
    }
    else
    {
        INFO("[kldpatt] ");
        ERROR("Pattern size: %d != Expected pattern size %d\r\n", size, klio_vars.max_cyclic_pattern_blob_size);
        (void)kldpatt_help(NULL);

        return;
    }
}

/**
* @brief Function to load generic pattern
* @param[in] arg1 : Argument for pattern index
* @param[in] arg2 : Argument for pattern as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_load_generic_pattern(const char *arg1, const char *arg2, struct bhy_dev *bhy)
{
    static uint8_t pattern_data[10240];
    uint16_t size = (uint16_t)pattern_blob_to_bytes((uint8_t *)arg2, (uint8_t *)pattern_data);
    uint8_t loop = (uint8_t)atoi(arg1);

    if (size <= klio_vars.max_generic_pattern_blob_size)
    {
        if (loop < klio_vars.max_generic_patterns)
        {
            (void)(bhy_klio_param_write_generic_pattern(loop, pattern_data, size, bhy));
        }
        else
        {
            INFO("[kldgpatt] ");
            ERROR("Pattern index: %d >= Max patterns %d\r\n", loop, klio_vars.max_generic_patterns);
            (void)kldgpatt_help(NULL);

            return;
        }
    }
    else
    {
        INFO("[kldgpatt] ");
        ERROR("Pattern size: %d > Maximum pattern size %d\r\n", size, klio_vars.max_generic_pattern_blob_size);
        (void)kldgpatt_help(NULL);

        return;
    }
}

/**
* @brief Function to set gesture configuration
* @param[in] arg1 : Argument for configuration index
* @param[in] arg2 : Argument for configuration as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_set_gesture_config(const char *arg1, const char *arg2, struct bhy_dev *bhy)
{
    uint8_t config_data[128] = { 0 };
    uint8_t loop = (uint8_t)atoi(arg1);
    uint16_t size = (uint16_t)pattern_blob_to_bytes((uint8_t *)arg2, config_data);

    (void)(bhy_klio_param_write_gesture_config(loop, config_data, size, bhy));

    return;
}

/**
* @brief Function to set timing configuration
* @param[in] arg : Argument for configuration as bare hex bytestring
* @param[in] bhy : Device instance
*/
static void klio_set_timing_config(const char *arg, struct bhy_dev *bhy)
{
    uint8_t config_data[128] = { 0 };
    uint16_t size = (uint16_t)pattern_blob_to_bytes((uint8_t *)arg, config_data);

    (void)(bhy_klio_param_write_timing_config(config_data, size, bhy));

    return;
}

/**
* @brief Function to get Klio parameter
* @param[in] arg : Argument for parameter ID
* @param[in] bhy : Device instance
*/
static void klio_get_parameter(const uint16_t *arg, struct bhy_dev *bhy)
{
    uint32_t param_id = (uint32_t)atoi((char *)arg);
    uint8_t buf[255];
    uint16_t size = sizeof(buf);
    float similarity = 0.0f;
    uint16_t max_cyclic_patterns = 0;

    (void)(bhy_klio_param_get_parameter((bhy_klio_param_t)param_id, buf, &size, bhy));

    switch (param_id)
    {
        case KLIO_PARAM_ALGORITHM_VERSION:
            INFO("[kgetparam] ");
            buf[sizeof(buf) - 1] = '\0';
            PRINT("Parameter %d: %s\r\n", param_id, buf);
            break;
        case KLIO_PARAM_RECOGNITION_RESPONSIVNESS:
        case KLIO_PARAM_LEARNING_SIMILARITY_THRESHOLD:
        case KLIO_PARAM_RECOGNITION_THRESHOLD_GENERIC:
            INFO("[kgetparam] ");
            memcpy(&similarity, buf, 4);
            PRINT("Parameter %d: %f\r\n", param_id, similarity);
            break;
        case KLIO_PARAM_PATTERN_BLOB_SIZE:
        case KLIO_PARAM_MAX_BLOB_SIZE_GENERIC:
        case KLIO_PARAM_RECOGNITION_MAX_PATTERNS:
        case KLIO_PARAM_MAX_NUM_PATTERNS_GENERIC:
            INFO("[kgetparam] ");
            memcpy(&max_cyclic_patterns, buf, 2);
            PRINT("Parameter %d: %u\r\n", param_id, max_cyclic_patterns);
            break;
        case KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT:
            INFO("[kgetparam] ");
            PRINT("Parameter %d: %u\r\n", param_id, buf[0]);
            break;
        default:
            for (uint16_t q = 0; q < size; q++)
            {
                if (q % 16 == 0)
                {
                    INFO("[kgetparam] ");
                    PRINT("Parameter %d: ", param_id);
                }

                PRINT("%02x ", buf[q]);
                if (q % 16 == 15 || q == size - 1)
                {
                    PRINT("\r\n");
                }
            }

            break;
    }
}

/**
* @brief Function to set Klio parameter
* @param[in] arg1 : Argument for parameter ID
* @param[in] arg2 : Argument for value to set
* @param[in] bhy  : Device instance
*/
static void klio_set_parameter(const char *arg1, char *arg2, struct bhy_dev *bhy)
{
    int param_id = atoi(arg1);
    char *param_value = arg2;
    float cycle_count;
    uint8_t ignore_insig_movement;

    switch (param_id)
    {
        case KLIO_PARAM_RECOGNITION_RESPONSIVNESS:
        case KLIO_PARAM_RECOGNITION_THRESHOLD_GENERIC:
            cycle_count = (float)atof(param_value);
            (void)(bhy_klio_param_set_parameter((bhy_klio_param_t)param_id, &cycle_count, sizeof(cycle_count), bhy));
            break;
        case KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT:
            ignore_insig_movement = (uint8_t)atoi(param_value);
            (void)(bhy_klio_param_set_parameter((bhy_klio_param_t)param_id, &ignore_insig_movement,
                                                sizeof(ignore_insig_movement), bhy));
            break;
        default:
            break;
    }
}

/**
* @brief Function to get Klio Similarity score
* @param[in] arg1 : Argument for first pattern as bare hex bytestring
* @param[in] arg2 : Argument for second pattern as bare hex bytestring
* @param[in] bhy  : Device instance
*/
static void klio_similarity_score(const uint8_t *arg1, const uint8_t *arg2, struct bhy_dev *bhy)
{
    uint8_t first_pattern_data[244] = { 0 }, second_pattern_data[244] = { 0 };
    uint16_t pattern1_size = (uint16_t)pattern_blob_to_bytes(arg1, first_pattern_data);
    uint16_t pattern2_size = (uint16_t)pattern_blob_to_bytes(arg2, second_pattern_data);

    if (pattern1_size != pattern2_size)
    {
        INFO("[ksimscore] ");
        ERROR("Patterns for similarity calculation differ in size\r\n");
    }
    else
    {
        float similarity;
        (void)(bhy_klio_param_similarity_score(first_pattern_data, second_pattern_data, pattern1_size, &similarity,
                                               bhy));
        INFO("[ksimscore] ");
        PRINT("Similarity: %f\r\n", similarity);
    }
}

/**
* @brief Function to get Multiple Klio Similarity score
* @param[in] arg1 : Argument for base index
* @param[in] arg2 : Argument for comparison indices
* @param[in] bhy  : Device instance
*/

/*lint -e772 -e830*/
static void klio_similarity_score_multiple(const char *arg1, const char *arg2, struct bhy_dev *bhy)
{
    uint8_t loop = (uint8_t)atoi(arg1);
    char *indexes_str = (char *)strdup((const char *)arg2);
    char *strtok_ptr;
    uint8_t indexes[klio_vars.max_cyclic_patterns];
    uint8_t count = 0;
    float similarity[klio_vars.max_cyclic_patterns];

    strtok_ptr = strtok(indexes_str, ",");
    while (strtok_ptr != NULL)
    {
        indexes[count] = (uint8_t)atoi(strtok_ptr);
        count++;
        strtok_ptr = strtok(NULL, ",");
    }

    (void)(bhy_klio_param_similarity_score_multiple(loop, indexes, count, similarity, bhy));

    INFO("[kmsimscore] ");
    PRINT("Using pattern id %d as reference: ", loop);
    for (uint32_t i = 0; i < count; i++)
    {
        PRINT("%d:%6f ", indexes[i], similarity[i]);
    }

    PRINT("\r\n");

    free(indexes_str);
}

/**
* @brief Function to control pattern operation
* @param[in] operation : Argument for pattern operation
* @param[in] arg1      : Argument for pattern indices
* @param[in] bhy       : Device instance
*/
static void klio_pattern_state_operation(const uint8_t operation, const char *arg1, struct bhy_dev *bhy)
{
    uint8_t count = 0;
    uint8_t pattern_states[klio_vars.max_cyclic_patterns];

    char *sep = ",";
    char *str = (char *)strdup((const char *)arg1);

    if (str == NULL)
    {
        (void)kenpatt_help(NULL);
        (void)kdispatt_help(NULL);

        return;
    }

    char *token = strtok(str, sep);

    while (token != NULL)
    {
        uint8_t loop = (uint8_t)atoi(token);

        if (loop < klio_vars.max_cyclic_patterns && count < klio_vars.max_cyclic_patterns)
        {
            pattern_states[count++] = loop;
        }
        else
        {
            free(str);
            (void)kenpatt_help(NULL);
            (void)kdispatt_help(NULL);

            return;
        }

        token = strtok(NULL, sep);
    }

    free(str);

    (void)(bhy_klio_param_set_pattern_states((bhy_klio_param_pattern_state_t) operation, pattern_states, count, bhy));
}
#endif

/**
* @brief Function to log sensor data
* @param[in] sid           : Sensor ID
* @param[in] tns           : Timestamp in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
* @param[in] logdev        : Device instance for log
*/
static void log_data(uint8_t sid,
                     uint64_t tns,
                     uint8_t event_size,
                     const uint8_t *event_payload,
                     struct logbin_dev *logdev)
{
    if (logdev && logdev->logfile)
    {
#ifndef PC
        (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
#endif
        logbin_add_data(sid, tns, event_size, event_payload, logdev);
#ifndef PC
        (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
#endif
    }
}

/**
* @brief Function to trigger FOC
* @param[in] payload : Sensor ID
* @param[in] bhy     : Device instance
*/
static void trigger_foc(const char *payload, struct bhy_dev *bhy)
{

    uint8_t str_param_id[8] = { 0 };
    uint16_t sensor_id;
    struct bhy_foc_resp foc_resp = { 0 };

    strncpy((char *)str_param_id, payload, strlen(payload));
    str_param_id[strlen(payload)] = '\0';

    sensor_id = (uint16_t)strtol((char *)str_param_id, NULL, 10);
    switch (sensor_id)
    {
        case BHY_ACCEL_FOC:
            PRINT("Keep the sensor stable for accel foc\r\n");
            (void)bhy_perform_foc((uint8_t)sensor_id, &foc_resp, bhy);
            break;

        case BHY_GYRO_FOC:
            PRINT("Keep the sensor stable for accel foc\r\n");
            (void)bhy_perform_foc(BHY_ACCEL_FOC, &foc_resp, bhy);
            PRINT("Gyro foc getting enabled\r\n");
            (void)bhy_perform_foc((uint8_t)sensor_id, &foc_resp, bhy);
            break;

        default:
            break;
    }

    switch (foc_resp.foc_status)
    {
        case BHY_FOC_SUCCESS:
            PRINT("FOC Success\r\n");
            break;
        case BHY_FOC_FAILURE:
            PRINT("FOC failed\r\n");
            break;
        case BHY_FOC_UNKNOWN_ERROR:
            PRINT("Unknown FOC failure\r\n");
            break;
        default:
            PRINT("Undefined FOC status %u\r\n", foc_resp.foc_status);
            break;
    }

}

/**
* @brief Function to stream HEX data
* @param[in] sid           : Sensor ID
* @param[in] ts            : Timestamp in seconds
* @param[in] tns           : Timestamp in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
*/
static void stream_hex_data(uint8_t sid, uint32_t ts, uint32_t tns, uint8_t event_size, const uint8_t *event_payload)
{
    /* Print sensor ID */
    HEX("%02x%08x%08x", sid, ts, tns);

    for (uint16_t i = 0; i < event_size; i++)
    {
        /* Output raw data in hex */
        PRINT_H("%02x", event_payload[i]);
    }

    PRINT_D("\r\n");
}

/**
* @brief Function to write meta information
* @param[in] log : Device instance for log
* @param[in] bhy : Device instance
*/
static void write_meta_info(struct logbin_dev *logbin, struct bhy_dev *bhy)
{
    logbin_start_meta(logbin);

    (void)bhy_update_virtual_sensor_list(bhy);

#if !defined(PC) && defined(MCU_APP30)
    coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

#if defined(MCU_APP31)
    coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

    for (uint8_t i = 1; i < BHY_SENSOR_ID_MAX; i++)
    {
        if (bhy_is_sensor_available(i, bhy))
        {
            logbin_add_meta(i,
                            get_sensor_name(i),
                            bhy->event_size[i] - 1,
                            get_sensor_parse_format(i),
                            get_sensor_axis_names(i),
                            get_sensor_default_scaling(i, bhy),
                            logbin);
        }
    }

    logbin_end_meta(logbin);

#if !defined(PC) && defined(MCU_APP30)
    coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

#if defined(MCU_APP31)
    coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

}

/**
* @brief Function to print schema information
* @param[in] bhy : Device instance
*/
static void schema_info(struct bhy_dev *bhy)
{
    /* Update virtual sensor */
    (void)bhy_update_virtual_sensor_list(bhy);

    /* Get present virtual sensor */
    (void)bhy_system_param_get_virtual_sensor_present(bhy);

    PRINT("Schema List.\r\n");
    PRINT("ID: Name: Event size: Parse format: Axis names: Scaling\r\n");

    for (uint8_t i = 1; i < BHY_SENSOR_ID_MAX; i++)
    {
        if (bhy_is_sensor_available(i, bhy))
        {
            if (i < BHY_SENSOR_ID_CUSTOM_START)
            {
                PRINT("%u: %s: ", i, get_sensor_name(i));
            }
            else
            {
                PRINT("%u: %s: ", i, custom_driver_information[i - BHY_SENSOR_ID_CUSTOM_START].sensor_name);
            }

            PRINT("%u: %s: %s: %f\r\n",
                  bhy->event_size[i] - 1,
                  get_sensor_parse_format(i),
                  get_sensor_axis_names(i),
                  get_sensor_default_scaling(i, bhy));
        }
    }
}

/**
* @brief Function to get default scaling factor for some sensors
* @param[in] sensor_id : Sensor ID
* @param[in] bhy       : Device instance
*/
static float get_sensor_default_scaling_value(uint8_t sensor_id, struct bhy_dev *bhy)
{
    float scaling = -1.0f;
    struct bhy_system_param_phys_sensor_info psi = { 0 };

    if (((sensor_id >= BHY_SENSOR_ID_ACC_PASS) && (sensor_id <= BHY_SENSOR_ID_ACC_RAW_WU)) ||
        ((sensor_id >= BHY_SENSOR_ID_GRA) && (sensor_id == BHY_SENSOR_ID_LACC_WU)) ||
        (sensor_id == BHY_SENSOR_ID_ACC_BIAS_WU))
    {
        (void)(bhy_system_param_get_physical_sensor_info(BHY_PHYS_SENSOR_ID_ACCELEROMETER, &psi, bhy));
        scaling = (float)psi.curr_range.u16_val / 32768.0f;
    }
    else if (((sensor_id >= BHY_SENSOR_ID_GYRO_PASS) && (sensor_id <= BHY_SENSOR_ID_GYRO_RAW_WU)) ||
             (sensor_id == BHY_SENSOR_ID_GYRO_BIAS_WU))
    {
        (void)(bhy_system_param_get_physical_sensor_info(BHY_PHYS_SENSOR_ID_GYROSCOPE, &psi, bhy));
        scaling = (float)psi.curr_range.u16_val / 32768.0f;
    }
    else if (((sensor_id >= BHY_SENSOR_ID_MAG_PASS) && (sensor_id <= BHY_SENSOR_ID_MAG_RAW_WU)) ||
             (sensor_id == BHY_SENSOR_ID_MAG_BIAS_WU))
    {
        (void)(bhy_system_param_get_physical_sensor_info(BHY_PHYS_SENSOR_ID_MAGNETOMETER, &psi, bhy));
        scaling = (float)psi.curr_range.u16_val / 32768.0f;
    }
    else if ((sensor_id >= BHY_SENSOR_ID_RV) && (sensor_id <= BHY_SENSOR_ID_GEORV_WU))
    {
        scaling = 1.0f / 16384.0f;
    }
    else if ((sensor_id == BHY_SENSOR_ID_ORI) || (sensor_id == BHY_SENSOR_ID_ORI_WU))
    {
        scaling = 360.0f / 32768.0f;
    }
    else if ((sensor_id == BHY_SENSOR_ID_TEMP) || (sensor_id == BHY_SENSOR_ID_TEMP_WU) ||
             (sensor_id == BHY_SENSOR_BMP_TEMPERATURE) || (sensor_id == BHY_SENSOR_BMP_TEMPERATURE_WU))
    {
        scaling = 1.0f / 100.0f;
    }

    return scaling;
}

/**
* @brief Function to get default scaling factor for all sensors
* @param[in] sensor_id : Sensor ID
* @param[in] bhy       : Device instance
*/
static float get_sensor_default_scaling(uint8_t sensor_id, struct bhy_dev *bhy)
{
    float scaling;

    scaling = get_sensor_default_scaling_value(sensor_id, bhy);

    if (scaling == -1.0f)
    {
        if ((sensor_id == BHY_SENSOR_ID_BARO) || (sensor_id == BHY_SENSOR_ID_BARO_WU))
        {
            scaling = 1.0f / 128.0f;
        }
        else if ((sensor_id == BHY_SENSOR_ID_HUM) || (sensor_id == BHY_SENSOR_ID_HUM_WU))
        {
            scaling = 1.0f;
        }
        else if ((sensor_id == BHY_SENSOR_ID_GAS) || (sensor_id == BHY_SENSOR_ID_GAS_WU))
        {
            scaling = 1.0f;
        }
        else if ((sensor_id == BHY_SENSOR_ID_LIGHT) || (sensor_id == BHY_SENSOR_ID_LIGHT_WU))
        {
            scaling = 10000.0f / 216.0f;
        }
        else if ((sensor_id == BHY_SENSOR_ID_PROX) || (sensor_id == BHY_SENSOR_ID_PROX_WU))
        {
            scaling = 1.0f;
        }
        else if ((sensor_id == BHY_SENSOR_ID_SI_ACCEL) || (sensor_id == BHY_SENSOR_ID_SI_GYROS))
        {
            /* Scaling factor already applied in firmware */
        }
        else if ((sensor_id == BHY_SENSOR_ID_HEAD_ORI_MIS_ALG) || (sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_Q) ||
                 (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_Q))
        {
            scaling = 1.0f / 16384.0f; /*2^14 -> 16384*/
        }
        else if ((sensor_id == BHY_SENSOR_ID_IMU_HEAD_ORI_E) || (sensor_id == BHY_SENSOR_ID_NDOF_HEAD_ORI_E))
        {
            scaling = 360.0f / 32768.0f; /*2^15 -> 32768*/
        }
        else
        {
            scaling = -1.0f; /* Do not apply the scaling factor */
        }
    }

    return scaling;
}

/**
* @brief Function to print BSX Parameter information
* @param[in] bsx_state : Pointer to BSX state structure
*/
static void print_bsx_algo_param_states(bhy_bsx_algo_param_state_exg *bsx_state_exg)
{
    for (uint8_t block = 0; block < BHY_BSX_STATE_MAX_BLOCKS; block++)
    {
        bhy_bsx_algo_param_state_exg *current_block = &bsx_state_exg[block];

        PRINT("Block number: %u\r\n", current_block->block_info & 0x7F);
        current_block->block_info & BHY_BSX_STATE_TRANSFER_COMPLETE ? PRINT("Completion flag: 1, Transfer complete\r\n")
        : PRINT("Completion flag: 0, Transfer not complete\r\n");
        PRINT("Block length: %u\r\n", current_block->block_len);
        PRINT("Struct length: %u\r\n", current_block->struct_len);

        PRINT("-------------------------------------------\r\n");
        PRINT("Block data \r\n");
        PRINT("Byte Hex      Dec | Data\r\n");
        PRINT("-------------------------------------------\r\n");
        for (uint8_t i = 0; i < current_block->block_len; i++)
        {
            if (i % 8 == 0)
            {
                PRINT("0x%06x %8d |", i, i);
            }

            PRINT("%02x ", current_block->state_data[i]);

            if ((i + 1) % 8 == 0 || i == current_block->block_len - 1)
            {
                PRINT("\r\n");
            }
        }

        if ((current_block->block_len == 0) || (current_block->block_info & BHY_BSX_STATE_TRANSFER_COMPLETE) != 0)
        {
            break;
        }

        PRINT("\r\n");
    }
}

/**
* @brief Function to print help for dmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dmode_help(void *ref)
{
    (void)ref;

    PRINT("  dmode <mode> \r\n");
    PRINT("    \t -Set the Injection <mode>, Real-Time(r)/Step-by-Step(s)/Normal(n)\r\n");
    PRINT("Eg:  dmode s \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for dmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dmode_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    int8_t rslt = BHY_OK;
    char injection_mode[20];
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    uint16_t code = 0x00;
    uint32_t actual_length = 0;
    uint8_t status[8] = { 0 };

    /*! Set the Mode */
    switch (argv[1][0])
    {
        case 'n':
            sprintf(injection_mode, "Normal"); /*Normal Mode */
            rslt = bhy_set_data_injection_mode(BHY_NORMAL_MODE, &cli_ref->bhy);
            break;
        case 'r':
            sprintf(injection_mode, "Real-Time"); /*Real-Time Mode */
            rslt = bhy_set_data_injection_mode(BHY_REAL_TIME_INJECTION, &cli_ref->bhy);
            break;
        case 's':
            sprintf(injection_mode, "Step-by-Step"); /*Step-by-Step Mode */
            rslt = bhy_set_data_injection_mode(BHY_STEP_BY_STEP_INJECTION, &cli_ref->bhy);
            break;
        default:
            break;
    }

    if (rslt != BHY_OK)
    {
        ERROR("Mode switching failed \r\n");

        return rslt;
    }

    /*! Check if the status register is updated and read the status*/
    rslt = bhy_read_status(&code, status, 8, &actual_length, &cli_ref->bhy); /*Fix for mode switching not occurring
                                                                                * cleanly */
    if (rslt != BHY_OK)
    {
        ERROR("Status check failed \r\n");

        return rslt;
    }

    INFO("Switching to %s mode\r\n", injection_mode);

    /*! Clear the buffer */
    memset(injection_mode, 0, sizeof(injection_mode));

    return CLI_OK;
}

/**
* @brief Function to print help for dinject command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dinject_help(void *ref)
{
    (void)ref;

    PRINT("  dinject <input_file.txt>\r\n");
    PRINT("    \t -Pass the name of the input file <input_filename.txt>\r\n");
    PRINT("   Eg:  dinject field_log.txt\r\n\r\n");
    PRINT("   For Logging Mode -\r\n");
    PRINT("  Command Flow: attlog outx logse 114:100 dmode s dinject field_log.txt dmode n logse 114:0 detlog outx\r\n");
    PRINT("  For Streaming Mode -\r\n");
    PRINT("  Command Flow: actse 114:100 dmode s dinject field_log.txt dmode n actse 114:0 \r\n\r\n");
    PRINT(
        "  Note:Ensure that the requisite application specific Data Injection firmware is loaded and the sensor is configured prior to executing Data Injection\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for dinject command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dinject_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t injection_state = DINJECT_IN_PROGRESS;
    uint8_t hex_data;

    memset(&dinject, 0, sizeof(struct data_inject));

    /*! Initialize the Data Injection Structure */
    (void)(dinject_init((char*)argv[1], &dinject, &cli_ref->bhy));

    /*    coines_delay_msec(100); / *Added for stability. Maintained for future reference * / */

    while (injection_state == DINJECT_IN_PROGRESS)
    {

        if (get_interrupt_status())
        {
            /*! Data from the FIFO is read and the relevant callback if registered are called */
            (void)(bhy_get_and_process_fifo(fifo_buffer, sizeof(fifo_buffer), &cli_ref->bhy));

            /*            coines_delay_msec(5); / *Added for stability. Maintained for future reference * / */
        }

        /*! Read '1' '8Bit Hex Data' */
        (void)dinject_parse_file(dinject.in_log_ptr, _8BIT_HEX_LEN, 1, &hex_data);

        /*! Data from the Input Log File is injected to relevant virtual sensor driver */
        injection_state = dinject_inject_data(hex_data, &dinject, &cli_ref->bhy);

        coines_delay_msec(5); /* workaround to fix the last sample of injection data lost in Linux */
    }

    /*! Close the Log file and reset the buffer parameters */
    (void)(dinject_deinit(&dinject, &cli_ref->bhy));

    /*! Clear the FIFO */
    memset(fifo_buffer, 0, sizeof(fifo_buffer)); /*Local Buffer */
    (void)(bhy_clear_fifo(0xFF, &cli_ref->bhy)); /*Read and Flush Wakeup and Non-Wakeup FIFO */
    (void)(bhy_clear_fifo(0xFE, &cli_ref->bhy)); /*Flush all the FIFOs */

    return CLI_OK;
}

/**
* @brief Function to print help for pm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t pm_help(void *ref)
{
    (void)ref;

    PRINT("  -m OR postm <pm_log_filename.bin>\r\n");
    PRINT("    \t -Pass <pm_log_filename.bin> to log the post mortem information\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for pm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t pm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_post_mortem post_mortem_data = { 0 };
    int8_t rslt;
    uint8_t error_value = 0, chip_ctrl_value = 0;

    /*! Check the Error Status */
    rslt = bhy_get_regs(BHY_REG_ERROR_VALUE, &error_value, 1, &cli_ref->bhy);

    PRINT("Error Reg Value : %x\r\n", error_value);

    if ((error_value == PM_DATA_AVAILABLE) || (error_value == WATCH_DOG_RESET) || (error_value == FATAL_FIRMWARE_ERROR))
    {
        /*! Get the Post Mortem data */
        rslt = get_post_mortem_data(&post_mortem_data, &cli_ref->bhy);
        if (rslt != BHY_OK)
        {
            PRINT("Post Mortem Data Retrieval Failed. Error : %d\r\n", rslt);

            return rslt;
        }

        /*! Log the Post Mortem data */
        rslt = log_post_mortem_data((char *)argv[1], &post_mortem_data, sizeof(struct bhy_post_mortem));
        if (rslt != PM_LOG_SUCCESS)
        {
            PRINT("Post Mortem Data Logging Failed. Error : %d\r\n", rslt);

            return rslt;
        }

#if PM_DEBUG

        /*! Print the post mortem data */
        rslt = print_post_mortem_data(&post_mortem_data);
        if (rslt != PM_PARSE_SUCCESS)
        {
            PRINT("Post Mortem Data Parsing Failed. \r\n");

            return rslt;
        }

#endif

        memset((uint8_t*)&post_mortem_data, 0, sizeof(struct bhy_post_mortem));

        /*! Read the Chip Control Register */
        rslt = bhy_get_regs(BHY_REG_CHIP_CTRL, &chip_ctrl_value, 1, &cli_ref->bhy);

        if (rslt == BHY_OK)
        {
            /*! Configure Chip Register to Clear Error and Debug Registers */
            uint8_t clr_err = chip_ctrl_value | BHY_CHIP_CTRL_CLR_ERR_REG;
            rslt = bhy_set_regs(BHY_REG_CHIP_CTRL, &clr_err, 1, &cli_ref->bhy);
        }
    }
    else
    {
        PRINT("No Fatal error observed. Post Mortem Data not available. \r\n");
    }

    if (rslt == BHY_OK)
    {
        PRINT("\r\n");
    }

    return CLI_OK;
}

/**
* @brief Function to print help for dactse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dactse_help(void *ref)
{
    (void)ref;

    PRINT("  dactse\r\n");
    PRINT("    \t -Deactivates all the active sensors\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for dactse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dactse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    /*! Disable the Sensors */
    bhy_exit(cli_ref);

    /*! Clear the FIFO */
    memset(fifo_buffer, 0, sizeof(fifo_buffer)); /*Local Buffer */
    (void)(bhy_clear_fifo(0xFF, &cli_ref->bhy)); /*Read and Flush Wakeup and Non-Wakeup FIFO */
    (void)(bhy_clear_fifo(0xFE, &cli_ref->bhy)); /*Flush all the FIFOs */

    PRINT("Deactivated all the Sensors\r\n\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for lsactse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t lsactse_help(void *ref)
{
    (void)ref;

    PRINT("  lsactse\r\n");
    PRINT("    \t -Lists all the active sensors\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for lsactse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t lsactse_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    uint8_t act_sensors = 0;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_virtual_sensor_conf_param_conf act_sensor_conf;
    struct parse_sensor_details *sensor_details;

    /*! Check the Sensor status */
    for (uint16_t i = 0; i < 256; i++)
    {
        if (sensors_active[i])
        {
            act_sensors++;
            if (act_sensors == 1)
            {
                PRINT("Active Sensors -\r\n");
            }

            (void)(bhy_virtual_sensor_conf_param_get_cfg((uint8_t)i, &act_sensor_conf, &cli_ref->bhy));

            sensor_details = parse_get_sensor_details((uint8_t)i, &cli_ref->parse_table);
            PRINT("SID : %3d \t ODR : %4.2f \t R : %4d \t Acquisition : %s\r\n",
                  i,
                  act_sensor_conf.sample_rate,
                  act_sensor_conf.range,
                  (sensor_details->parse_flag == PARSE_FLAG_STREAM) ? "Streaming" : "Logging");
        }
    }

    /*! If no active sensors */
    if (act_sensors == 0)
    {
        PRINT("No Active Sensors\r\n");
    }

    if (cli_ref->parse_table.logdev.logfile != NULL)
    {
        PRINT("Attached Log File : %s\r\n", cli_ref->parse_table.logdev.logfilename);
    }
    else
    {
        PRINT("No File attached for Logging\r\n");
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for mtapen command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapen_help(void *ref)
{
    (void)ref;

    PRINT("  mtapen <tap-setting>\r\n");
    PRINT("    \t -Enable the Multi Tap Configuration \r\n");
    PRINT("    \t -0 : No Tap \r\n");
    PRINT("    \t -1 : Single Tap \r\n");
    PRINT("    \t -2 : Double Tap \r\n");
    PRINT("    \t -3 : Double Single Tap \r\n");
    PRINT("    \t -4 : Triple Tap \r\n");
    PRINT("    \t -5 : Triple Single Tap \r\n");
    PRINT("    \t -6 : Triple Double Tap \r\n");
    PRINT("    \t -7 : Triple Double Single Tap \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for mtapen command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapen_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    bhy_event_data_multi_tap multitap_setting = (bhy_event_data_multi_tap)atoi((char *)argv[1]);

    rslt = bhy_multi_tap_param_set_config(&multitap_setting, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Multi Tap Parameter Set Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Parameter set to  %s\r\n", bhy_event_data_multi_tap_string_out[multitap_setting]);

    return CLI_OK;
}

/**
* @brief Function to print help for mtapinfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapinfo_help(void *ref)
{
    (void)ref;

    PRINT("  mtapinfo \r\n");
    PRINT("    \t -Get the Multi Tap Info\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for mtapinfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapinfo_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    /*lint -e778*/
    uint16_t len = BHY_LE24MUL(BHY_MULTI_TAP_PARAM_ENABLE_LENGTH);
    bhy_event_data_multi_tap buffer[len];

    memset(buffer, 0, len);

    rslt = bhy_multi_tap_param_get_config(buffer, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Multi Tap Parameter Get Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Info : %s\r\n", bhy_event_data_multi_tap_string_out[buffer[0]]);

    return CLI_OK;
}

/**
* @brief Function to print help for mtapsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  mtapsetcnfg <s_cnfg> <d_cnfg> <t_cnfg> \r\n");
    PRINT("    \t -Set the Multi Tap Configurations (in Hex)\r\n");
    PRINT("    \t -<s_cnfg> : Single Tap Configuration\r\n");
    PRINT("    \t\t -<s_cnfg> : <0x00>|<mode|max_pks_for_tap|<wait_for_timeout|axis_sel> [MSB->LSB]\r\n");
    PRINT("    \t\t -<mode> : <Robust/Normal/Sensitive> -> <2/1/0>\r\n");
    PRINT("    \t\t -<max_pks_for_tap> : 6\r\n");
    PRINT("    \t\t -<wait_for_timeout> : <Robust/Normal/Sensitive> -> <2/1/0>\r\n");
    PRINT("    \t\t -<axis_sel> : <Z/Y/X> -> <2/1/0>\r\n");
    PRINT("    \t -<d_cnfg> : Double Tap Configuration\r\n");
    PRINT("    \t\t -<d_cnfg> : <max_ges_dur>|<tap_peak_thrs> [MSB->LSB]\r\n");
    PRINT("    \t\t -<max_ges_dur> : <15:10> -> 0 to 2520ms at resolution of 40ms\r\n");
    PRINT("    \t\t -<tap_peak_thrs> : <9:0> -> 0 to 2000mg at resolution of 1.953mg \r\n");
    PRINT("    \t -<t_cnfg> : Triple Tap Configuration\r\n");
    PRINT(
        "    \t\t -<t_cnfg> : <quite_time_after_ges>|<min_quite_dur_bw_taps>|<tap_shock_settl_dur>|<max_dur_bw_pks> [MSB->LSB]\r\n");
    PRINT("    \t\t -<quite_time_after_ges> : <15:12> -> 0 to 75ms at resolution of 5ms\r\n");
    PRINT("    \t\t -<min_quite_dur_bw_taps> : <11:8> -> 0 to 75ms at resolution of 5ms \r\n");
    PRINT("    \t\t -<tap_shock_settl_dur> : <7:4> -> 0 to 75ms at resolution of 5ms\r\n");
    PRINT("    \t\t -<max_dur_bw_pks> : <3:0> -> 0 to 600ms at resolution of 40ms \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for mtapsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    bhy_multi_tap_param_detector multitap_cnfg;

    multitap_cnfg.stap_setting.as_uint16 = (uint16_t)string_to_int((char *)argv[1]);
    multitap_cnfg.dtap_setting.as_uint16 = (uint16_t)string_to_int((char *)argv[2]);
    multitap_cnfg.ttap_setting.as_uint16 = (uint16_t)string_to_int((char *)argv[3]);

    rslt = bhy_multi_tap_param_detector_set_config(&multitap_cnfg, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Multi Tap Detector Parameter Set Failed \r\n");

        return rslt;
    }

    PRINT("Multi Tap Detector Parameter set successfully \r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for mtapgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  mtapgetcnfg \r\n");
    PRINT("    \t -Get the Multi Tap Configurations\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for mtapgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    int8_t rslt;

    bhy_multi_tap_param_detector multitap_cnfg;

    rslt = bhy_multi_tap_param_detector_get_config(&multitap_cnfg, &cli_ref->bhy);

    if (rslt != BHY_OK)
    {
        ERROR("Multi Tap Parameter Get Failed \r\n");

        return rslt;
    }

    PRINT("Single Tap CNFG : 0x%04x\r\n", multitap_cnfg.stap_setting.as_uint16);
    PRINT("    \t\t -<axis_sel> : %d\r\n", multitap_cnfg.stap_setting.as_s.axis_sel);
    PRINT("    \t\t -<wait_for_timeout> : %d\r\n", multitap_cnfg.stap_setting.as_s.wait_for_timeout);
    PRINT("    \t\t -<max_pks_for_tap> : %d\r\n", multitap_cnfg.stap_setting.as_s.max_peaks_for_tap);
    PRINT("    \t\t -<mode> : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    PRINT("Double Tap CNFG : 0x%04x\r\n", multitap_cnfg.dtap_setting.as_uint16);
    PRINT("    \t\t -<tap_peak_thrs> : %d\r\n", multitap_cnfg.dtap_setting.as_s.tap_peak_thres);
    PRINT("    \t\t -<max_ges_dur> : %d\r\n", multitap_cnfg.dtap_setting.as_s.max_gesture_dur);
    PRINT("Triple Tap CNFG : 0x%04x\r\n", multitap_cnfg.ttap_setting.as_uint16);
    PRINT("    \t\t -<max_dur_bw_pks> : %d\r\n", multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks);
    PRINT("    \t\t -<tap_shock_settl_dur> : %d\r\n", multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur);
    PRINT("    \t\t -<min_quite_dur_bw_taps> : %d\r\n", multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps);
    PRINT("    \t\t -<quite_time_after_ges> : %d\r\n", multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture);

    return CLI_OK;
}

/**
* @brief Function to print help for accsetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetfoc_help(void *ref)
{
    (void)ref;

    PRINT("  accsetfoc <x> <y> <z> \r\n");
    PRINT("    \t -Set the Accelerometer Fast Offset Calibration (in Hex)\r\n");
    PRINT("    \t -Range of Accelerometer FOC value : -128 to 127 [8Bit Resolution]\r\n");
    PRINT("    \t -<x> : FOC calibration for X-axis \r\n");
    PRINT("    \t -<y> : FOC calibration for Y-axis \r\n");
    PRINT("    \t -<z> : FOC calibration for Z-axis \r\n");
    PRINT("    \t -e.g accsetfoc 0x0072 0x0064 0x007F\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accsetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_accel_fast_offset_calib accfoc = { 0 };

    accfoc.x_offset = (int16_t)string_to_int((char *)argv[1]);
    accfoc.y_offset = (int16_t)string_to_int((char *)argv[2]);
    accfoc.z_offset = (int16_t)string_to_int((char *)argv[3]);

    if ((accfoc.x_offset > 127) || (accfoc.x_offset < -128) || (accfoc.y_offset > 127) || (accfoc.y_offset < -128) ||
        (accfoc.z_offset > 127) || (accfoc.z_offset < -128))
    {
        ERROR("Invalid FOC value. Range : -128 to 127 [8Bit Resolution]\r\n");

        return CLI_E_INVALID_PARAM;
    }

    PRINT("Set the Accelerometer Fast Offset Calibration\r\n");
    PRINT("    \t -<x_offset> : %d\r\n", accfoc.x_offset);
    PRINT("    \t -<y_offset> : %d\r\n", accfoc.y_offset);
    PRINT("    \t -<z_offset> : %d\r\n", accfoc.z_offset);

    (void)(bhy_phy_sensor_ctrl_param_accel_set_foc_calibration(&accfoc, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for accgetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetfoc_help(void *ref)
{
    (void)ref;

    PRINT("  accgetfoc \r\n");
    PRINT("    \t -Get the Accelerometer Fast Offset Calibration\r\n");
    PRINT("    \t -Range of Accelerometer FOC value : -128 to 127 [8Bit Resolution]\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accgetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_accel_fast_offset_calib accfoc = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_accel_get_foc_calibration(&accfoc, &cli_ref->bhy));

    PRINT("Accelerometer Fast Offset Calibration : \r\n");
    PRINT("    \t -<x_offset> : %d\r\n", accfoc.x_offset);
    PRINT("    \t -<y_offset> : %d\r\n", accfoc.y_offset);
    PRINT("    \t -<z_offset> : %d\r\n", accfoc.z_offset);

    return CLI_OK;
}

/**
* @brief Function to print help for accsetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  accsetpwm <power_mode> \r\n");
    PRINT("    \t -Set the Accelerometer Power Mode\r\n");
    PRINT("    \t -<power_mode> : Power Mode\r\n");
    PRINT("    \t -'0' corresponds to Normal Mode\r\n");
    PRINT("    \t -'2' corresponds to Low Power Mode\r\n");
    PRINT("    \t -e.g accsetpwm 0\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accsetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t power_mode;
    static char* power_str[] = { "NORMAL", "UNDEFINED", "LOW POWER" };

    power_mode = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_accel_set_power_mode(power_mode, &cli_ref->bhy));

    PRINT("Set the Accelerometer Power Mode to %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for accgetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  accgetpwm \r\n");
    PRINT("    \t -Get the Accelerometer Power Mode\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accgetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t power_mode = 0;
    static char* power_str[] = { "NORMAL", "UNDEFINED", "LOW POWER" };

    (void)(bhy_phy_sensor_ctrl_param_accel_get_power_mode(&power_mode, &cli_ref->bhy));

    PRINT("Accelerometer Power Mode : %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for accsetar command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetar_help(void *ref)
{
    (void)ref;

    PRINT("  accsetar <x> <x_sign> <y> <y_sign> <z> <z_sign>\r\n");
    PRINT("    \t -Set the Accelerometer axis remapping for internal imu features\r\n");
    PRINT("    \t -Range of each axis : 0, 1\r\n");
    PRINT("    \t -Range of each axis sign : 0, 1, 2\r\n");
    PRINT("    \t -<x> : Re-map for X-axis \r\n");
    PRINT("    \t -<x_sign> : Re-map for X-axis sign \r\n");
    PRINT("    \t -<y> : Re-map for Y-axis \r\n");
    PRINT("    \t -<y_sign> : Re-map for Y-axis sign \r\n");
    PRINT("    \t -<z> : Re-map for Z-axis \r\n");
    PRINT("    \t -<z_sign> : Re-map for Z-axis sign \r\n");
    PRINT("    \t -e.g accsetar 1 1 1 1 1 1\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accsetar command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetar_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_accel_axis_remap remap = { 0 };

    remap.map_x_axis = (uint8_t)string_to_int((char *)argv[1]);
    remap.map_x_axis_sign = (uint8_t)string_to_int((char *)argv[2]);
    remap.map_y_axis = (uint8_t)string_to_int((char *)argv[3]);
    remap.map_y_axis_sign = (uint8_t)string_to_int((char *)argv[4]);
    remap.map_z_axis = (uint8_t)string_to_int((char *)argv[5]);
    remap.map_z_axis_sign = (uint8_t)string_to_int((char *)argv[6]);

    (void)(bhy_phy_sensor_ctrl_param_accel_set_axis_remapping(&remap, &cli_ref->bhy));

    PRINT("Set the Accelerometer axis remapping\r\n");
    PRINT("    \t -<x> : %d\r\n", remap.map_x_axis);
    PRINT("    \t -<x_sign> : %d\r\n", remap.map_x_axis_sign);
    PRINT("    \t -<y> : %d\r\n", remap.map_y_axis);
    PRINT("    \t -<y_sign> : %d\r\n", remap.map_y_axis_sign);
    PRINT("    \t -<z> : %d\r\n", remap.map_z_axis);
    PRINT("    \t -<z_sign> : %d\r\n", remap.map_z_axis_sign);

    return CLI_OK;
}

/**
* @brief Function to print help for accgetar command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetar_help(void *ref)
{
    (void)ref;

    PRINT("  accgetar \r\n");
    PRINT("    \t -Get the Accelerometer axis remapping\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accgetar command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetar_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_accel_axis_remap remap = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_accel_get_axis_remapping(&remap, &cli_ref->bhy));

    PRINT("Accelerometer axis remapping : \r\n");
    PRINT("    \t -<x> : %d\r\n", remap.map_x_axis);
    PRINT("    \t -<x_sign> : %d\r\n", remap.map_x_axis_sign);
    PRINT("    \t -<y> : %d\r\n", remap.map_y_axis);
    PRINT("    \t -<y_sign> : %d\r\n", remap.map_y_axis_sign);
    PRINT("    \t -<z> : %d\r\n", remap.map_z_axis);
    PRINT("    \t -<z_sign> : %d\r\n", remap.map_z_axis_sign);

    return CLI_OK;
}

/**
* @brief Function to print help for acctrignvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t acctrignvm_help(void *ref)
{
    (void)ref;

    PRINT("  acctrignvm\r\n");
    PRINT("    \t -Trigger a NVM writing for accelerometer\r\n");
    PRINT("    \t -Note: Trigger FOC first so that NVM writing can take effect \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for acctrignvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t acctrignvm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    (void)(bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing(&cli_ref->bhy));

    PRINT("Trigger a NVM writing for accelerometer\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for accgetnvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetnvm_help(void *ref)
{
    (void)ref;

    PRINT("  accgetnvm \r\n");
    PRINT("    \t -Get the NVM writing status for accelerometer\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for accgetnvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetnvm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    uint8_t status;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    (void)(bhy_phy_sensor_ctrl_param_accel_get_nvm_status(&status, &cli_ref->bhy));

    PRINT("NVM writing status for accelerometer: %s\r\n",
          (status == BHY_PHY_PARAM_ACCEL_NVM_WRITE_STATUS_DONE) ? "Done" : "In progress");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetfoc_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosetfoc <x> <y> <z> \r\n");
    PRINT("    \t -Set the Gyroscope Fast Offset Calibration (in Hex)\r\n");
    PRINT("    \t -Range of Gyroscope FOC value : -512 to 511 [10Bit Resolution]\r\n");
    PRINT("    \t -<x> : FOC calibration for X-axis \r\n");
    PRINT("    \t -<y> : FOC calibration for Y-axis \r\n");
    PRINT("    \t -<z> : FOC calibration for Z-axis \r\n");
    PRINT("    \t -e.g gyrosetfoc 0x0016 0x00f8 0x080\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_gyro_fast_offset_calib gyrofoc = { 0 };

    gyrofoc.x_offset = (int16_t)string_to_int((char *)argv[1]);
    gyrofoc.y_offset = (int16_t)string_to_int((char *)argv[2]);
    gyrofoc.z_offset = (int16_t)string_to_int((char *)argv[3]);

    (void)(bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration(&gyrofoc, &cli_ref->bhy));

    PRINT("Set the Gyroscope Fast Offset Calibration \r\n");
    PRINT("    \t -<x_offset> : %d\r\n", gyrofoc.x_offset);
    PRINT("    \t -<y_offset> : %d\r\n", gyrofoc.y_offset);
    PRINT("    \t -<z_offset> : %d\r\n", gyrofoc.z_offset);

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetfoc_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetfoc \r\n");
    PRINT("    \t -Get the Gyroscope Fast Offset Calibration\r\n");
    PRINT("    \t -Range of Gyroscope FOC value : -512 to 511 [10Bit Resolution]\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_gyro_fast_offset_calib gyrofoc = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration(&gyrofoc, &cli_ref->bhy));

    PRINT("Gyroscope Fast Offset Calibration : \r\n");
    PRINT("    \t -<x_offset> : %d\r\n", gyrofoc.x_offset);
    PRINT("    \t -<y_offset> : %d\r\n", gyrofoc.y_offset);
    PRINT("    \t -<z_offset> : %d\r\n", gyrofoc.z_offset);

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosetois command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetois_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosetois <config> \r\n");
    PRINT("    \t -Set the Gyroscope OIS configuration\r\n");
    PRINT("    \t -<config> : OIS configuration (1: Enable, 0: Disable)\r\n");
    PRINT("    \t -e.g gyrosetois 1 will enable Gyroscope OIS\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosetois command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetois_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config;

    config = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_gyro_set_ois_config(config, &cli_ref->bhy));

    PRINT("Gyroscope OIS %s\r\n", (config == BHY_PHY_PARAM_GYRO_OIS_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetois command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetois_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetois \r\n");
    PRINT("    \t -Get the Gyroscope OIS configuration \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetois command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetois_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config = 0;

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_ois_config(&config, &cli_ref->bhy));

    PRINT("Gyroscope OIS Status : %s\r\n", (config == BHY_PHY_PARAM_GYRO_OIS_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosetfs command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetfs_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosetfs <config> \r\n");
    PRINT("    \t -Set the Gyroscope Fast Startup\r\n");
    PRINT("    \t -<config> : Fast Startup configuration (1: Enable, 0: Disable)\r\n");
    PRINT("    \t -e.g gyrosetfs 1 will enable Gyroscope Fast Startup\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosetfs command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetfs_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config;

    config = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg(config, &cli_ref->bhy));

    PRINT("Gyroscope Fast Startup %s\r\n", (config == BHY_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetfs command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetfs_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetfs \r\n");
    PRINT("    \t -Get the Gyroscope Fast Startup status \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetfs command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetfs_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config = 0;

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg(&config, &cli_ref->bhy));

    PRINT("Gyroscope Fast Startup Status : %s\r\n",
          (config == BHY_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosetcrt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetcrt_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosetcrt \r\n");
    PRINT("    \t -Start Gyroscope CRT\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosetcrt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetcrt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    PRINT("Start Gyroscope Component ReTrim (CRT)\r\n");
    (void)(bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim(&cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetcrt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetcrt_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetcrt \r\n");
    PRINT("    \t -Get the Gyroscope CRT status \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetcrt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetcrt_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_gyro_crt_status status;

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_crt_status(&status, &cli_ref->bhy));

    PRINT("Gyroscope CRT Status : %s\r\n",
          (status.status == BHY_PHY_PARAM_GYRO_COMP_RETRIM_SUCCESS) ? "Successful" : "Failed");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosetpwm <power_mode> \r\n");
    PRINT("    \t -Set the Gyroscope Power Mode\r\n");
    PRINT("    \t -<power_mode> : Power Mode\r\n");
    PRINT("    \t -'0' corresponds to Normal Mode\r\n");
    PRINT("    \t -'1' corresponds to Performance Mode\r\n");
    PRINT("    \t -'2' corresponds to Low Power Mode\r\n");
    PRINT("    \t -e.g gyrosetpwm 0\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint8_t power_mode;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    static char* power_str[] = { "NORMAL", "PERFORMANCE", "LOW POWER" };

    power_mode = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_gyro_set_power_mode(power_mode, &cli_ref->bhy));

    PRINT("Set the Gyroscope Power Mode to %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetpwm \r\n");
    PRINT("    \t -Get the Gyroscope Power Mode\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    uint8_t power_mode = 0;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    static char* power_str[] = { "NORMAL", "PERFORMANCE", "LOW POWER" };

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_power_mode(&power_mode, &cli_ref->bhy));

    PRINT("Gyroscope Power Mode : %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for gyrosettat command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosettat_help(void *ref)
{
    (void)ref;

    PRINT("  gyrosettat <config> \r\n");
    PRINT("    \t -Set the Gyroscope Timer Auto Trim state\r\n");
    PRINT("    \t -<config> : Timer Auto Trim configuration (1: Start, 0: Disable)\r\n");
    PRINT("    \t -e.g gyrosettat 1 will start Gyroscope Timer Auto Trim\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrosettat command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosettat_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config;

    config = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg(config, &cli_ref->bhy));

    PRINT("Gyroscope Timer Auto Trim %s\r\n",
          (config == BHY_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_START) ? "Started" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogettat command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogettat_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogettat \r\n");
    PRINT("    \t -Get the Gyroscope Timer Auto Trim status \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogettat command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogettat_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t config = 0;

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg(&config, &cli_ref->bhy));

    PRINT("Gyroscope Timer Auto Trim Status : %s\r\n",
          (config == BHY_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_START) ? "Started" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrotrignvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrotrignvm_help(void *ref)
{
    (void)ref;

    PRINT("  gyrotrignvm\r\n");
    PRINT("    \t -Trigger a NVM writing for gyroscope\r\n");
    PRINT("    \t -Note: Trigger FOC first so that NVM writing can take effect \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrotrignvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrotrignvm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    (void)(bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing(&cli_ref->bhy));

    PRINT("Trigger a NVM writing for gyroscope\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for gyrogetnvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetnvm_help(void *ref)
{
    (void)ref;

    PRINT("  gyrogetnvm \r\n");
    PRINT("    \t -Get the NVM writing status for gyroscope\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gyrogetnvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetnvm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    uint8_t status;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_nvm_status(&status, &cli_ref->bhy));

    PRINT("NVM writing status for gyroscope: %s\r\n",
          (status == BHY_PHY_PARAM_ACCEL_NVM_WRITE_STATUS_DONE) ? "Done" : "In progress");

    return CLI_OK;
}

/**
* @brief Function to print help for magsetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t magsetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  magsetpwm <power_mode> \r\n");
    PRINT("    \t -Set the Magnetometer Power Mode\r\n");
    PRINT("    \t -<power_mode> : Power Mode\r\n");
    PRINT("    \t -'0' corresponds to Normal Mode\r\n");
    PRINT("    \t -'2' corresponds to Low Power Mode\r\n");
    PRINT("    \t -e.g magsetpwm 0\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for magsetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t magsetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint8_t power_mode;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    static char* power_str[] = { "NORMAL", "UNDEFINED", "LOW POWER" };

    power_mode = (uint8_t)atoi((char *)argv[1]);
    (void)(bhy_phy_sensor_ctrl_param_gyro_set_power_mode(power_mode, &cli_ref->bhy));

    PRINT("Set the Magnetometer Power Mode to %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for maggetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t maggetpwm_help(void *ref)
{
    (void)ref;

    PRINT("  maggetpwm \r\n");
    PRINT("    \t -Get the Magnetometer Power Mode\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for maggetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t maggetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    uint8_t power_mode = 0;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    static char* power_str[] = { "NORMAL", "UNDEFINED", "LOW POWER" };

    (void)(bhy_phy_sensor_ctrl_param_gyro_get_power_mode(&power_mode, &cli_ref->bhy));

    PRINT("Magnetometer Power Mode : %s\r\n", power_str[power_mode]);

    return CLI_OK;
}

/**
* @brief Function to print help for wwwsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wwwsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  wwwsetcnfg <maf> <manf> <alr> <all> <apd> <apu> <mdm> <mdq>\r\n");
    PRINT("    \t -Set the Wrist Wear Wakeup Configuration\r\n");
    PRINT("    \t -<maf> : min_angle_focus (u16), range 1024 to 1774\r\n");
    PRINT("    \t -<manf> : min_angle_non_focus(u16), range 1448 to 1856\r\n");
    PRINT("    \t -<alr> : angle_landscape_right (u8), range 88 to 128\r\n");
    PRINT("    \t -<all> : angle_landscape_left(u8), range 88 to 128\r\n");
    PRINT("    \t -<apd> : angle_portrait_down (u8), range 0 to 179\r\n");
    PRINT("    \t -<apu> : angle_portrait_up(u8), range 222 to 247\r\n");
    PRINT("    \t -<mdm> : min_dur_moved (u8), range 1 to 10s, in steps of 20ms\r\n");
    PRINT("    \t -<mdq> : min_dur_quite(u8), range 1 to 10s, in steps of 20ms\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wwwsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wwwsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_wrist_wear_wakeup config = { 0 };

    config.min_angle_focus = (uint16_t)atoi((char *)argv[1]);
    config.min_angle_non_focus = (uint16_t)atoi((char *)argv[2]);
    config.angle_landscape_right = (uint8_t)atoi((char *)argv[3]);
    config.angle_landscape_left = (uint8_t)atoi((char *)argv[4]);
    config.angle_portrait_down = (uint8_t)atoi((char *)argv[5]);
    config.angle_portrait_up = (uint8_t)atoi((char *)argv[6]);
    config.min_dur_moved = (uint8_t)atoi((char *)argv[7]);
    config.min_dur_quite = (uint8_t)atoi((char *)argv[8]);

    PRINT("Set the Wrist Wear Wakeup Configuration\r\n");
    PRINT("    \t -min_angle_focus : %d\r\n", config.min_angle_focus);
    PRINT("    \t -min_angle_non_focus : %d\r\n", config.min_angle_non_focus);
    PRINT("    \t -angle_landscape_right : %d\r\n", config.angle_landscape_right);
    PRINT("    \t -angle_landscape_left : %d\r\n", config.angle_landscape_left);
    PRINT("    \t -angle_portrait_down : %d\r\n", config.angle_portrait_down);
    PRINT("    \t -angle_portrait_up : %d\r\n", config.angle_portrait_up);
    PRINT("    \t -min_dur_moved : %d\r\n", config.min_dur_moved);
    PRINT("    \t -min_dur_quite : %d\r\n", config.min_dur_quite);

    (void)(bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for wwwgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wwwgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  wwwgetcnfg \r\n");
    PRINT("    \t -Get the Wrist Wear Wakeup Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wwwgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wwwgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_wrist_wear_wakeup config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg(&config, &cli_ref->bhy));

    PRINT("Wrist Wear Wakeup Configuration:\r\n");
    PRINT("    \t -min_angle_focus : %d\r\n", config.min_angle_focus);
    PRINT("    \t -min_angle_non_focus : %d\r\n", config.min_angle_non_focus);
    PRINT("    \t -angle_landscape_right : %d\r\n", config.angle_landscape_right);
    PRINT("    \t -angle_landscape_left : %d\r\n", config.angle_landscape_left);
    PRINT("    \t -angle_portrait_down : %d\r\n", config.angle_portrait_down);
    PRINT("    \t -angle_portrait_up : %d\r\n", config.angle_portrait_up);
    PRINT("    \t -min_dur_moved : %d\r\n", config.min_dur_moved);
    PRINT("    \t -min_dur_quite : %d\r\n", config.min_dur_quite);

    return CLI_OK;
}

/**
* @brief Function to print help for amsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t amsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  amsetcnfg <dur> <axis> <thrs> \r\n");
    PRINT("    \t -Set the Any Motion Configuration\r\n");
    PRINT("    \t -<dur> : duration (u16), range 0 to 163s in steps of 20ms\r\n");
    PRINT("    \t -<axis> : axis select (u8), range 0 to 7, <az>:<ay>:<ax>\r\n");
    PRINT("    \t -<thrs> : threshold (u16), range 0 to 1g in steps of 0.5mg\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for amsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t amsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_any_motion config = { 0 };

    config.duration = (uint16_t)string_to_int((char *)argv[1]);
    config.axis_sel = (uint16_t)string_to_int((char *)argv[2]);
    config.threshold = (uint16_t)string_to_int((char *)argv[3]);

    PRINT("Set the Any Motion Configuration\r\n");
    PRINT("    \t -duration : %d\r\n", config.duration);
    PRINT("    \t -axis_sel : %d\r\n", config.axis_sel);
    PRINT("    \t -threshold : %d\r\n", config.threshold);

    (void)(bhy_phy_sensor_ctrl_param_set_any_motion_config(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for amgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t amgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  amgetcnfg \r\n");
    PRINT("    \t -Get the Any Motion Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for amgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t amgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_any_motion config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_get_any_motion_config(&config, &cli_ref->bhy));

    PRINT("Any Motion Configuration:\r\n");
    PRINT("    \t -duration : %d\r\n", config.duration);
    PRINT("    \t -axis_sel : %d\r\n", config.axis_sel);
    PRINT("    \t -threshold : %d\r\n", config.threshold);

    return CLI_OK;
}

/**
* @brief Function to print help for nmsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t nmsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  nmsetcnfg <dur> <axis> <thrs> \r\n");
    PRINT("    \t -Set the No Motion Configuration\r\n");
    PRINT("    \t -<dur> : duration (u16), range 0 to 163s in steps of 20ms\r\n");
    PRINT("    \t -<axis> : axis select (u8), range 0 to 7, <az>:<ay>:<ax>\r\n");
    PRINT("    \t -<thrs> : threshold (u16), range 0 to 1g in steps of 0.5mg\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for nmsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t nmsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_no_motion config = { 0 };

    config.duration = (uint16_t)string_to_int((char *)argv[1]);
    config.axis_sel = (uint16_t)string_to_int((char *)argv[2]);
    config.threshold = (uint16_t)string_to_int((char *)argv[3]);

    PRINT("Set the No Motion Configuration\r\n");
    PRINT("    \t -duration : %d\r\n", config.duration);
    PRINT("    \t -axis_sel : %d\r\n", config.axis_sel);
    PRINT("    \t -threshold : %d\r\n", config.threshold);

    (void)(bhy_phy_sensor_ctrl_param_set_no_motion_config(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for nmgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t nmgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  nmgetcnfg \r\n");
    PRINT("    \t -Get the No Motion Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for nmgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t nmgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_no_motion config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_get_no_motion_config(&config, &cli_ref->bhy));

    PRINT("No Motion Configuration:\r\n");
    PRINT("    \t -duration : %d\r\n", config.duration);
    PRINT("    \t -axis_sel : %d\r\n", config.axis_sel);
    PRINT("    \t -threshold : %d\r\n", config.threshold);

    return CLI_OK;
}

/**
* @brief Function to print help for wgdsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wgdsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  wgdsetcnfg <mfpy_th> <mfpz_th> <gx_pos> <gx_neg> <gy_neg> <gz_neg> <fpdc> <lmfc> <mdjp> <dp> \r\n");
    PRINT("    \t -Set the Wrist Gesture Detector Configuration (in Hex)\r\n");
    PRINT("    \t -<mfpy_th> : min_flick_peak_y_thres (u16), range 0x3E8 to 0x9C4\r\n");
    PRINT("    \t -<mfpz_th> : min_flick_peak_z_thres (u16), range 0x1F4 to 0x5DC\r\n");
    PRINT("    \t -<gx_pos> : gravity_bounds_x_pos (u16), range 0x0 to 0x800\r\n");
    PRINT("    \t -<gx_neg> : gravity_bounds_x_neg (u16), range 0x0 to 0xFC00\r\n");
    PRINT("    \t -<gy_neg> : gravity_bounds_y_neg (u16), range 0x0 to 0xFC3F\r\n");
    PRINT("    \t -<gz_neg> : gravity_bounds_z_neg (u16), range 0x800 to 0xF912\r\n");
    PRINT("    \t -<fpdc> : flick_peak_decay_coeff (u16), range 0x0 to 0x8000\r\n");
    PRINT("    \t -<lmfc> : lp_mean_filter_coeff (u16), range 0x0 to 0x8000\r\n");
    PRINT("    \t -<mdjp> : max_duration_jiggle_peaks (u16), range 0xA to 0x19\r\n");
    PRINT("    \t -<dp> : device_position (u8), 0 or 1\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wgdsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wgdsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_wrist_gesture_detector config = { 0 };

    config.min_flick_peak_y_thres = (uint16_t)string_to_int((char *)argv[1]);
    config.min_flick_peak_z_thres = (uint16_t)string_to_int((char *)argv[2]);
    config.gravity_bounds_x_pos = (uint16_t)string_to_int((char *)argv[3]);
    config.gravity_bounds_x_neg = (uint16_t)string_to_int((char *)argv[4]);
    config.gravity_bounds_y_neg = (uint16_t)string_to_int((char *)argv[5]);
    config.gravity_bounds_z_neg = (uint16_t)string_to_int((char *)argv[6]);
    config.flick_peak_decay_coeff = (uint16_t)string_to_int((char *)argv[7]);
    config.lp_mean_filter_coeff = (uint16_t)string_to_int((char *)argv[8]);
    config.max_duration_jiggle_peaks = (uint16_t)string_to_int((char *)argv[9]);
    config.device_pos = (uint8_t)string_to_int((char *)argv[10]);

    PRINT("Set the Wrist Gesture Detector Configuration\r\n");
    PRINT("    \t -min_flick_peak_y_thres : 0x%04x\r\n", config.min_flick_peak_y_thres);
    PRINT("    \t -min_flick_peak_z_thres : 0x%04x\r\n", config.min_flick_peak_z_thres);
    PRINT("    \t -gravity_bounds_x_pos : 0x%04x\r\n", config.gravity_bounds_x_pos);
    PRINT("    \t -gravity_bounds_x_neg : 0x%04x\r\n", config.gravity_bounds_x_neg);
    PRINT("    \t -gravity_bounds_y_neg : 0x%04x\r\n", config.gravity_bounds_y_neg);
    PRINT("    \t -gravity_bounds_z_neg : 0x%04x\r\n", config.gravity_bounds_z_neg);
    PRINT("    \t -flick_peak_decay_coeff : 0x%04x\r\n", config.flick_peak_decay_coeff);
    PRINT("    \t -lp_mean_filter_coeff : 0x%04x\r\n", config.lp_mean_filter_coeff);
    PRINT("    \t -max_duration_jiggle_peaks : 0x%04x\r\n", config.max_duration_jiggle_peaks);
    PRINT("    \t -device_pos : 0x%02x\r\n", config.device_pos);

    (void)(bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for wgdgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wgdgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  wgdgetcnfg \r\n");
    PRINT("    \t -Get the Wrist Wear Wakeup Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wgdgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wgdgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_wrist_gesture_detector config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg(&config, &cli_ref->bhy));

    PRINT("Wrist Wear Wakeup Configuration:\r\n");
    PRINT("    \t -min_flick_peak_y_thres : 0x%04x\r\n", config.min_flick_peak_y_thres);
    PRINT("    \t -min_flick_peak_z_thres : 0x%04x\r\n", config.min_flick_peak_z_thres);
    PRINT("    \t -gravity_bounds_x_pos : 0x%04x\r\n", config.gravity_bounds_x_pos);
    PRINT("    \t -gravity_bounds_x_neg : 0x%04x\r\n", config.gravity_bounds_x_neg);
    PRINT("    \t -gravity_bounds_y_neg : 0x%04x\r\n", config.gravity_bounds_y_neg);
    PRINT("    \t -gravity_bounds_z_neg : 0x%04x\r\n", config.gravity_bounds_z_neg);
    PRINT("    \t -flick_peak_decay_coeff : 0x%04x\r\n", config.flick_peak_decay_coeff);
    PRINT("    \t -lp_mean_filter_coeff : 0x%04x\r\n", config.lp_mean_filter_coeff);
    PRINT("    \t -max_duration_jiggle_peaks : 0x%04x\r\n", config.max_duration_jiggle_peaks);
    PRINT("    \t -device_pos : 0x%02x\r\n", config.device_pos);

    return CLI_OK;
}

/**
* @brief Function to print help for baro1setcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro1setcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  baro1setcnfg <osr_p> <osr_t> <iir_filter> \r\n");
    PRINT("    \t -Set the Barometer pressure type 1 Configuration\r\n");
    PRINT("    \t -<osr_p> : Oversampling setting pressure measurement (u8), range 0 to 5\r\n");
    PRINT("    \t -<osr_t> : Oversampling setting temperature measurement (u8), range 0 to 5\r\n");
    PRINT("    \t -<iir_filter> : Filter coefficient for IIR filter (u8), range 0 to 7\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for baro1setcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro1setcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_baro_type_1 config = { 0 };

    config.osr_p = (uint8_t)string_to_int((char *)argv[1]);
    config.osr_t = (uint8_t)string_to_int((char *)argv[2]);
    config.iir_filter = (uint8_t)string_to_int((char *)argv[3]);

    PRINT("Set the Barometer pressure type 1 Configuration\r\n");
    PRINT("    \t -osr_p : %d\r\n", config.osr_p);
    PRINT("    \t -osr_t : %d\r\n", config.osr_t);
    PRINT("    \t -iir_filter : %d\r\n", config.iir_filter);

    (void)(bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for baro1getcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro1getcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  baro1getcnfg \r\n");
    PRINT("    \t -Get the Barometer pressure type 1 Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for baro1getcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro1getcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_baro_type_1 config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg(&config, &cli_ref->bhy));

    PRINT("Barometer pressure type 1 Configuration:\r\n");
    PRINT("    \t -osr_p : %d\r\n", config.osr_p);
    PRINT("    \t -osr_t : %d\r\n", config.osr_t);
    PRINT("    \t -iir_filter : %d\r\n", config.iir_filter);

    return CLI_OK;
}

/**
* @brief Function to print help for baro2setcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro2setcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  baro2setcnfg <osr_p> <osr_t> <iir_filter_p> <iir_filter_t> <dsp_config>\r\n");
    PRINT("    \t -Set the Barometer pressure type 2 Configuration\r\n");
    PRINT("    \t -<osr_p> : Oversampling setting pressure measurement (u8), range 0 to 5\r\n");
    PRINT("    \t -<osr_t> : Oversampling setting temperature measurement (u8), range 0 to 5\r\n");
    PRINT("    \t -<iir_filter_p> : Filter coefficient for pressure IIR (u8), range 0 to 7\r\n");
    PRINT("    \t -<iir_filter_t> : Filter coefficient for temperature IIR filter (u8), range 0 to 7\r\n");
    PRINT("    \t -<dsp_config> : DSP configuration (u8), range 0 to 255\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for baro2setcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro2setcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_baro_type_2 config = { 0 };

    config.osr_p = (uint8_t)string_to_int((char *)argv[1]);
    config.osr_t = (uint8_t)string_to_int((char *)argv[2]);
    config.iir_filter_p = (uint8_t)string_to_int((char *)argv[3]);
    config.iir_filter_t = (uint8_t)string_to_int((char *)argv[4]);
    config.dsp_config = (uint8_t)string_to_int((char *)argv[5]);

    PRINT("Set the Barometer pressure type 2 Configuration\r\n");
    PRINT("    \t -osr_p : %d\r\n", config.osr_p);
    PRINT("    \t -osr_t : %d\r\n", config.osr_t);
    PRINT("    \t -iir_filter_p : %d\r\n", config.iir_filter_p);
    PRINT("    \t -iir_filter_t : %d\r\n", config.iir_filter_t);
    PRINT("    \t -dsp_config : %d\r\n", config.dsp_config);

    (void)(bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for baro2getcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro2getcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  baro2getcnfg \r\n");
    PRINT("    \t -Get the Barometer pressure type 2 Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for baro2getcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro2getcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_baro_type_2 config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg(&config, &cli_ref->bhy));

    PRINT("Barometer pressure type 2 Configuration:\r\n");
    PRINT("    \t -osr_p : %d\r\n", config.osr_p);
    PRINT("    \t -osr_t : %d\r\n", config.osr_t);
    PRINT("    \t -iir_filter_p : %d\r\n", config.iir_filter_p);
    PRINT("    \t -iir_filter_t : %d\r\n", config.iir_filter_t);
    PRINT("    \t -dsp_config : %d\r\n", config.dsp_config);

    return CLI_OK;
}

/**
* @brief Function to print help for scsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t scsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  scsetcnfg <emdu> <ecu> <emdd> <ecd> <sbs> <mvd> <msd> <fcb2> <fcb1> <fcb0> <fca2> <fca1> <fce> <pdmw>\r\n");
    PRINT("  <pdmr> <sdm> <sdw> <hse> <adf> <adt> <sci> <sdpe> <sdt> <empp> <mt> <sc26> <sc27>\r\n");
    PRINT("    \t -Set the Step Counter Configuration\r\n");
    PRINT("    \t -<emdu> : env_min_dist_up (u16)\r\n");
    PRINT("    \t -<ecu> : env_coef_up (u16)\r\n");
    PRINT("    \t -<emdd> : env_min_dist_down (u16)\r\n");
    PRINT("    \t -<ecd> : env_coef_down (u16)\r\n");
    PRINT("    \t -<sbs> : step_buffer_size (u16)\r\n");
    PRINT("    \t -<mvd> : mean_val_decay (u16)\r\n");
    PRINT("    \t -<msd> : mean_step_dur (u16)\r\n");
    PRINT("    \t -<fcb2> : filter_coeff_b2 (u16)\r\n");
    PRINT("    \t -<fcb1> : filter_coeff_b1 (u16)\r\n");
    PRINT("    \t -<fcb0> : filter_coeff_b0 (u16)\r\n");
    PRINT("    \t -<fca2> : filter_coeff_a2 (u16)\r\n");
    PRINT("    \t -<fca1> : filter_coeff_a1 (u16)\r\n");
    PRINT("    \t -<fce> : filter_cascade_enabled (u16)\r\n");
    PRINT("    \t -<pdmw> : peak_duration_min_walking (u16)\r\n");
    PRINT("    \t -<pdmr> : peak_duration_min_running (u16)\r\n");
    PRINT("    \t -<sdm> : step_duration_max (u16)\r\n");
    PRINT("    \t -<sdw> : step_duration_window (u16)\r\n");
    PRINT("    \t -<hse> : half_step_enabled (u16)\r\n");
    PRINT("    \t -<adf> : activity_detection_factor (u16)\r\n");
    PRINT("    \t -<adt> : activity_detection_thres (u16)\r\n");
    PRINT("    \t -<sci> : step_counter_increment (u16)\r\n");
    PRINT("    \t -<sdpe> : step_duration_pp_enabled (u16)\r\n");
    PRINT("    \t -<sdt> : step_dur_thres (u16)\r\n");
    PRINT("    \t -<empp> : en_mcr_pp (u16)\r\n");
    PRINT("    \t -<mt> : mcr_thres (u16)\r\n");
    PRINT("    \t -<sc26> : sc_26 (u16)\r\n");
    PRINT("    \t -<sc27> : sc_27 (u16)\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for scsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t scsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_step_counter config = { 0 };

    config.env_min_dist_up = (uint16_t)string_to_int((char *)argv[1]);
    config.env_coef_up = (uint16_t)string_to_int((char *)argv[2]);
    config.env_min_dist_down = (uint16_t)string_to_int((char *)argv[3]);
    config.env_coef_down = (uint16_t)string_to_int((char *)argv[4]);
    config.step_buffer_size = (uint16_t)string_to_int((char *)argv[5]);
    config.mean_val_decay = (uint16_t)string_to_int((char *)argv[6]);
    config.mean_step_dur = (uint16_t)string_to_int((char *)argv[7]);
    config.filter_coeff_b2 = (uint16_t)string_to_int((char *)argv[8]);
    config.filter_coeff_b1 = (uint16_t)string_to_int((char *)argv[9]);
    config.filter_coeff_b0 = (uint16_t)string_to_int((char *)argv[10]);
    config.filter_coeff_a2 = (uint16_t)string_to_int((char *)argv[11]);
    config.filter_coeff_a1 = (uint16_t)string_to_int((char *)argv[12]);
    config.filter_cascade_enabled = (uint16_t)string_to_int((char *)argv[13]);
    config.peak_duration_min_walking = (uint16_t)string_to_int((char *)argv[14]);
    config.peak_duration_min_running = (uint16_t)string_to_int((char *)argv[15]);
    config.step_duration_max = (uint16_t)string_to_int((char *)argv[16]);
    config.step_duration_window = (uint16_t)string_to_int((char *)argv[17]);
    config.half_step_enabled = (uint16_t)string_to_int((char *)argv[18]);
    config.activity_detection_factor = (uint16_t)string_to_int((char *)argv[19]);
    config.activity_detection_thres = (uint16_t)string_to_int((char *)argv[20]);
    config.step_counter_increment = (uint16_t)string_to_int((char *)argv[21]);
    config.step_duration_pp_enabled = (uint16_t)string_to_int((char *)argv[22]);
    config.step_dur_thres = (uint16_t)string_to_int((char *)argv[23]);
    config.en_mcr_pp = (uint16_t)string_to_int((char *)argv[24]);
    config.mcr_thres = (uint16_t)string_to_int((char *)argv[25]);
    config.sc_26 = (uint16_t)string_to_int((char *)argv[26]);
    config.sc_27 = (uint16_t)string_to_int((char *)argv[27]);

    PRINT("Set the Step Counter Configuration\r\n");
    PRINT("    \t-env_min_dist_up: %u\r\n", config.env_min_dist_up);
    PRINT("    \t-env_coef_up: %u\r\n", config.env_coef_up);
    PRINT("    \t-env_min_dist_down: %u\r\n", config.env_min_dist_down);
    PRINT("    \t-env_coef_down: %u\r\n", config.env_coef_down);
    PRINT("    \t-step_buffer_size: %u\r\n", config.step_buffer_size);
    PRINT("    \t-mean_val_decay: %u\r\n", config.mean_val_decay);
    PRINT("    \t-mean_step_dur: %u\r\n", config.mean_step_dur);
    PRINT("    \t-filter_coeff_b2: %u\r\n", config.filter_coeff_b2);
    PRINT("    \t-filter_coeff_b1: %u\r\n", config.filter_coeff_b1);
    PRINT("    \t-filter_coeff_b0: %u\r\n", config.filter_coeff_b0);
    PRINT("    \t-filter_coeff_a2: %u\r\n", config.filter_coeff_a2);
    PRINT("    \t-filter_coeff_a1: %u\r\n", config.filter_coeff_a1);
    PRINT("    \t-filter_cascade_enabled: %u\r\n", config.filter_cascade_enabled);
    PRINT("    \t-peak_duration_min_walking: %u\r\n", config.peak_duration_min_walking);
    PRINT("    \t-peak_duration_min_running: %u\r\n", config.peak_duration_min_running);
    PRINT("    \t-step_duration_max: %u\r\n", config.step_duration_max);
    PRINT("    \t-step_duration_window: %u\r\n", config.step_duration_window);
    PRINT("    \t-half_step_enabled: %u\r\n", config.half_step_enabled);
    PRINT("    \t-activity_detection_factor: %u\r\n", config.activity_detection_factor);
    PRINT("    \t-activity_detection_thres: %u\r\n", config.activity_detection_thres);
    PRINT("    \t-step_counter_increment: %u\r\n", config.step_counter_increment);
    PRINT("    \t-step_duration_pp_enabled: %u\r\n", config.step_duration_pp_enabled);
    PRINT("    \t-step_dur_thres: %u\r\n", config.step_dur_thres);
    PRINT("    \t-en_mcr_pp: %u\r\n", config.en_mcr_pp);
    PRINT("    \t-mcr_thres: %u\r\n", config.mcr_thres);
    PRINT("    \t-sc_26: %u\r\n", config.sc_26);
    PRINT("    \t-sc_27: %u\r\n", config.sc_27);

    (void)(bhy_phy_sensor_ctrl_param_set_step_counter_config(&config, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for scgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t scgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  scgetcnfg \r\n");
    PRINT("    \t -Get the Step Counter Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for scgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t scgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_phy_sensor_ctrl_param_step_counter config = { 0 };

    (void)(bhy_phy_sensor_ctrl_param_get_step_counter_config(&config, &cli_ref->bhy));

    PRINT("Step Counter Configuration:\r\n");
    PRINT("    \t-env_min_dist_up: %u\r\n", config.env_min_dist_up);
    PRINT("    \t-env_coef_up: %u\r\n", config.env_coef_up);
    PRINT("    \t-env_min_dist_down: %u\r\n", config.env_min_dist_down);
    PRINT("    \t-env_coef_down: %u\r\n", config.env_coef_down);
    PRINT("    \t-step_buffer_size: %u\r\n", config.step_buffer_size);
    PRINT("    \t-mean_val_decay: %u\r\n", config.mean_val_decay);
    PRINT("    \t-mean_step_dur: %u\r\n", config.mean_step_dur);
    PRINT("    \t-filter_coeff_b2: %u\r\n", config.filter_coeff_b2);
    PRINT("    \t-filter_coeff_b1: %u\r\n", config.filter_coeff_b1);
    PRINT("    \t-filter_coeff_b0: %u\r\n", config.filter_coeff_b0);
    PRINT("    \t-filter_coeff_a2: %u\r\n", config.filter_coeff_a2);
    PRINT("    \t-filter_coeff_a1: %u\r\n", config.filter_coeff_a1);
    PRINT("    \t-filter_cascade_enabled: %u\r\n", config.filter_cascade_enabled);
    PRINT("    \t-peak_duration_min_walking: %u\r\n", config.peak_duration_min_walking);
    PRINT("    \t-peak_duration_min_running: %u\r\n", config.peak_duration_min_running);
    PRINT("    \t-step_duration_max: %u\r\n", config.step_duration_max);
    PRINT("    \t-step_duration_window: %u\r\n", config.step_duration_window);
    PRINT("    \t-half_step_enabled: %u\r\n", config.half_step_enabled);
    PRINT("    \t-activity_detection_factor: %u\r\n", config.activity_detection_factor);
    PRINT("    \t-activity_detection_thres: %u\r\n", config.activity_detection_thres);
    PRINT("    \t-step_counter_increment: %u\r\n", config.step_counter_increment);
    PRINT("    \t-step_duration_pp_enabled: %u\r\n", config.step_duration_pp_enabled);
    PRINT("    \t-step_dur_thres: %u\r\n", config.step_dur_thres);
    PRINT("    \t-en_mcr_pp: %u\r\n", config.en_mcr_pp);
    PRINT("    \t-mcr_thres: %u\r\n", config.mcr_thres);
    PRINT("    \t-sc_26: %u\r\n", config.sc_26);
    PRINT("    \t-sc_27: %u\r\n", config.sc_27);

    return CLI_OK;
}

/**
* @brief Function to print help for hmctrig command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmctrig_help(void *ref)
{
    (void)ref;

    PRINT("  hmctrig\r\n");
    PRINT("    \t= Trigger Head Misalignment Calibration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmctrig command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmctrig_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_head_orientation_param_trigger_hmc_calibration(&cli_ref->bhy));

    PRINT("Triggered Head Misalignment Calibration\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for hmcsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  hmcsetcnfg <sp_max_dur> <sp_min_dur> <sp_max_samples> <acc_diff_th> \r\n");
    PRINT("    \t -Set the Head Misalignment Configuration (in Hex)\r\n");
    PRINT(
        "    \t <sp_max_dur> : maximum still phase duration required for pitch and roll calibration (u8), resolution 1 sec/LSB\r\n");
    PRINT(
        "    \t <sp_min_dur> : minimum still phase duration required for pitch and roll calibration (u8), resolution 1 sec/LSB\r\n");
    PRINT(
        "    \t <sp_max_samples> : maximal number of samples for still phase for the dynamic part of head misalignment calibration (u8), resolution 1 sample/LSB\r\n");
    PRINT(
        "    \t <acc_diff_th> : threshold of acceleration difference to detect motion in acceleration signal,(i32),\r\n");
    PRINT("	   \t resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT("    \t eg. hmcsetcnfg 0x06 0x02 0x32 0x00002042 \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_misalignment_config hmc_config = { 0 };

    hmc_config.still_phase_max_dur = (uint8_t)string_to_int((char *)argv[1]);
    hmc_config.still_phase_min_dur = (uint8_t)string_to_int((char *)argv[2]);
    hmc_config.still_phase_max_samples = (uint8_t)string_to_int((char *)argv[3]);
    hmc_config.acc_diff_threshold = (int32_t)string_to_int((char *)argv[4]);

    (void)(bhy_head_orientation_param_set_hmc_configuration(&hmc_config, &cli_ref->bhy));

    PRINT("Head Misalignment Configuration set successfully \r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for hmcgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  hmcgetcnfg \r\n");
    PRINT("    \t -Get the Head Misalignment Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_misalignment_config hmc_config = { 0 };

    (void)(bhy_head_orientation_param_get_hmc_configuration(&hmc_config, &cli_ref->bhy));

    PRINT("still_phase_max_dur : 0x%02x\r\n", hmc_config.still_phase_max_dur);
    PRINT("still_phase_min_dur : 0x%02x\r\n", hmc_config.still_phase_min_dur);
    PRINT("still_phase_max_samples : 0x%02x\r\n", hmc_config.still_phase_max_samples);
    PRINT("acc_diff_threshold : 0x%08x\r\n", hmc_config.acc_diff_threshold);

    return CLI_OK;
}

/**
* @brief Function to print help for hmcsetdefcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetdefcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  hmcsetdefcnfg\r\n");
    PRINT("    \t= Set Default Head Misalignment Configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcsetdefcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetdefcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_head_orientation_param_set_default_hmc_cfg(&cli_ref->bhy));

    PRINT("Set Default Head Misalignment Calibration successfully\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for hmcver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcver_help(void *ref)
{
    (void)ref;

    PRINT("  hmcver \r\n");
    PRINT("    \t= Get the Head Misalignment Calibrator version.\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcver_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_ver hmc_version;

    INFO("Executing %\r\n", argv[0]);

    (void)(bhy_head_orientation_param_get_hmc_version(&hmc_version, &cli_ref->bhy));

    PRINT("Head Misalignment Calibrator version: %u.%u.%u\r\n\r\n",
          hmc_version.major,
          hmc_version.minor,
          hmc_version.patch);

    return CLI_OK;
}

/**
* @brief Function to print help for hmcsetcalcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetcalcorrq_help(void *ref)
{
    (void)ref;

    PRINT("  hmcsetcalcorrq <quat_x> <quat_y> <quat_z> <quat_w> \r\n");
    PRINT("    \t= Set the Head Misalignment Quaternion Calibration Correction\r\n");
    PRINT(
        "    \t <quat_x> : quaternion x, (float), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_y> : quaternion y, (float), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_z> : quaternion z, (float), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT(
        "    \t <quat_w> : quaternion w, (float), resolution 1 mg/LSB [includes 1 bit sign, 8 bits exponent and 23 bits fraction]\r\n");
    PRINT("    \t eg. hmcsetcalcorrq 1.000 1.000 1.000 1.000 \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcsetcalcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetcalcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_misalignment_quat_corr hmc_quat_corr = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    hmc_quat_corr.quaternion_x.f_val = (float)string_to_int((char *)argv[1]);
    hmc_quat_corr.quaternion_y.f_val = (float)string_to_int((char *)argv[2]);
    hmc_quat_corr.quaternion_z.f_val = (float)string_to_int((char *)argv[3]);
    hmc_quat_corr.quaternion_w.f_val = (float)string_to_int((char *)argv[4]);
    hmc_quat_corr.accuracy.f_val = 0.0f;
    (void)(bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg(&hmc_quat_corr, &cli_ref->bhy));

    PRINT("Head Misalignment Quaternion Calibration Correction set successfully\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for hmcgetcalcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetcalcorrq_help(void *ref)
{
    (void)ref;

    PRINT("  hmcgetcalcorrq \r\n");
    PRINT("    \t= Get the Head Misalignment Quaternion Calibration Correction\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcgetcalcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetcalcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_misalignment_quat_corr hmc_quat_corr = { 0 };

    (void)(bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg(&hmc_quat_corr, &cli_ref->bhy));

    PRINT("quaternion_x : %f\r\n", hmc_quat_corr.quaternion_x.f_val);
    PRINT("quaternion_y : %f\r\n", hmc_quat_corr.quaternion_y.f_val);
    PRINT("quaternion_z : %f\r\n", hmc_quat_corr.quaternion_z.f_val);
    PRINT("quaternion_w : %f\r\n", hmc_quat_corr.quaternion_w.f_val);

    return CLI_OK;
}

/**
* @brief Function to print help for hmcsetmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetmode_help(void *ref)
{
    (void)ref;

    PRINT("  hmcsetmode <mode> <x[0]> <x[1]> <x[2]> \r\n");
    PRINT("    \t= Set the Head Misalignment Mode and Vector X value\r\n");
    PRINT("    \t <mode> : HMC mode, 0=default mode, 1=semi-automatic mode\r\n");
    PRINT("    \t <x[0]> : x[0], (float), includes 1 bit sign, 8 bits exponent and 23 bits fraction \r\n");
    PRINT("    \t <x[1]> : x[1], (float), includes 1 bit sign, 8 bits exponent and 23 bits fraction \r\n");
    PRINT("    \t <x[2]> : x[2], (float), includes 1 bit sign, 8 bits exponent and 23 bits fraction \r\n");
    PRINT("    \t eg. hmcsetmode 0 0.0 0.0 1.0 \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcsetmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetmode_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_misalignment_mode_vector_x hmc_mode_vect_x = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    hmc_mode_vect_x.mode = (uint8_t)string_to_int((char *)argv[1]);
    hmc_mode_vect_x.vector_x_0.f_val = (float)string_to_int((char *)argv[2]);
    hmc_mode_vect_x.vector_x_1.f_val = (float)string_to_int((char *)argv[3]);
    hmc_mode_vect_x.vector_x_2.f_val = (float)string_to_int((char *)argv[4]);

    (void)(bhy_head_orientation_param_set_hmc_mode_vector_x(&hmc_mode_vect_x, &cli_ref->bhy));

    PRINT("Set Head Misalignment Mode and Vector X value:\r\n");
    PRINT("    \t -<mode> : %d\r\n", hmc_mode_vect_x.mode);
    PRINT("    \t -<vector_x_0> : %f\r\n", hmc_mode_vect_x.vector_x_0.f_val);
    PRINT("    \t -<vector_x_1> : %f\r\n", hmc_mode_vect_x.vector_x_1.f_val);
    PRINT("    \t -<vector_x_2> : %f\r\n", hmc_mode_vect_x.vector_x_2.f_val);

    return CLI_OK;
}

/**
* @brief Function to print help for hmcgetmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetmode_help(void *ref)
{
    (void)ref;

    PRINT("  hmcgetmode \r\n");
    PRINT("    \t= Get the Head Misalignment Mode and Vector X value\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hmcgetmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetmode_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_misalignment_mode_vector_x hmc_mode_vect_x = { 0 };

    (void)(bhy_head_orientation_param_get_hmc_mode_vector_x(&hmc_mode_vect_x, &cli_ref->bhy));

    PRINT("    \t- Head Misalignment Mode and Vector X value:\r\n");
    PRINT("    \t -<mode> : %d\r\n", hmc_mode_vect_x.mode);
    PRINT("    \t -<vector_x_0> : %f\r\n", hmc_mode_vect_x.vector_x_0.f_val);
    PRINT("    \t -<vector_x_1> : %f\r\n", hmc_mode_vect_x.vector_x_1.f_val);
    PRINT("    \t -<vector_x_2> : %f\r\n", hmc_mode_vect_x.vector_x_2.f_val);

    return CLI_OK;
}

/**
* @brief Function to print help for hosetheadcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hosetheadcorrq_help(void *ref)
{
    (void)ref;

    PRINT("  hosetheadcorrq\r\n");
    PRINT("    \t= Set Initial Heading Correction, only for IMU Head Orientation Quaternion\r\n");
    PRINT("    \t -1/0 : Enable/Disable Initial Heading Correction [Quaternion] \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hosetheadcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hosetheadcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t ho_quat_head_corr_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    ho_quat_head_corr_state[0] = (uint8_t)string_to_int((char *)argv[1]);
    (void)(bhy_head_orientation_param_set_quat_init_head_corr(ho_quat_head_corr_state, &cli_ref->bhy));

    PRINT("Quaternion Initial Heading Correction %s\r\n",
          (ho_quat_head_corr_state[0] ==
           BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for hogetheadcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hogetheadcorrq_help(void *ref)
{
    (void)ref;

    PRINT("  hogetheadcorrq \r\n");
    PRINT("    \t= Get Initial Heading Correction, only for IMU Head Orientation Quaternion\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hogetheadcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hogetheadcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t ho_quat_head_corr_state[4] = { 0 };

    (void)(bhy_head_orientation_param_get_quat_init_head_corr(ho_quat_head_corr_state, &cli_ref->bhy));

    PRINT("Quaternion Initial Heading Correction Status : %s\r\n",
          (ho_quat_head_corr_state[0] ==
           BHY_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for hover command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hover_help(void *ref)
{
    (void)ref;

    PRINT("  hover \r\n");
    PRINT("    \t= Get IMU/NDOF Head Orientation Version \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hover command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hover_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_head_orientation_param_ver ho_version;

    INFO("Executing %\r\n", argv[0]);

    (void)(bhy_head_orientation_param_get_ho_version(&ho_version, &cli_ref->bhy));

    PRINT("IMU/NDOF Head Orientation version: %u.%u.%u\r\n\r\n", ho_version.major, ho_version.minor, ho_version.patch);

    return CLI_OK;
}

/**
* @brief Function to print help for hosetheadcorre command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hosetheadcorre_help(void *ref)
{
    (void)ref;

    PRINT("  hosetheadcorre\r\n");
    PRINT("    \t= Set Initial Heading Correction, only for IMU Head Orientation Euler\r\n");
    PRINT("    \t -1/0 : Enable/Disable Initial Heading Correction [Euler] \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hosetheadcorre command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hosetheadcorre_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t ho_eul_head_corr_state[4] = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    ho_eul_head_corr_state[0] = (uint8_t)string_to_int((char *)argv[1]);
    (void)(bhy_head_orientation_param_set_eul_init_head_corr(ho_eul_head_corr_state, &cli_ref->bhy));

    PRINT("Euler Initial Heading Correction %s\r\n",
          (ho_eul_head_corr_state[0] ==
           BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for hogetheadcorre command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hogetheadcorre_help(void *ref)
{
    (void)ref;

    PRINT("  hogetheadcorre \r\n");
    PRINT("    \t= Get Initial Heading Correction, only for IMU Head Orientation Euler\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for hogetheadcorre command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hogetheadcorre_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint8_t ho_eul_head_corr_state[4] = { 0 };

    (void)(bhy_head_orientation_param_get_eul_init_head_corr(ho_eul_head_corr_state, &cli_ref->bhy));

    PRINT("Euler Initial Heading Correction Status : %s\r\n",
          (ho_eul_head_corr_state[0] ==
           BHY_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ENABLE) ? "Enabled" : "Disabled");

    return CLI_OK;
}

/**
* @brief Function to print help for foc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t foc_help(void *ref)
{
    (void)ref;

    PRINT("  foc <sensor id ('1' for accelerometer, '3' for gyroscope)>\r\n");
    PRINT("    \t Enables the fast offset compensation for the sensor \r\n");
    PRINT("    \t -e.g. foc 1 will enable fast offset compensation for accelerometer\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for foc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t foc_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    trigger_foc((char *)argv[1], &cli_ref->bhy);
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for chipid command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getchipid_help(void *ref)
{
    (void)ref;

    PRINT("  chipid \r\n");
    PRINT("    \t= Get CHIP ID of the sensor\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for chipid command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getchipid_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    uint8_t chip_id = 0;

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_get_chip_id(&chip_id, &cli_ref->bhy));

    PRINT("CHIP ID : 0x%02x\r\n", chip_id);

    return CLI_OK;
}

/**
* @brief Function to implement callback for syssetphyseninfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetphyseninfo_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_orient_matrix orient_matrix = { { 0 } };
    uint8_t loop = 0;
    char delimiter[] = ",";
    uint8_t sensor_id = (uint8_t)atoi((char *)argv[1]);
    char *axes = (char *)argv[2];
    char *token = strtok(axes, delimiter);

    while ((token != NULL))
    {
        orient_matrix.c[loop] = (int8_t)atoi(token);
        token = strtok(NULL, delimiter);
        loop++;
    }

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    (void)(bhy_system_param_set_physical_sensor_info(sensor_id, &orient_matrix, &cli_ref->bhy));

    PRINT("Set the orientation matrix for the Physical Sensors successfully\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for bsecsetalstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsetalstate_help(void *ref)
{
    (void)ref;

    PRINT("  bsecsetalstate <algo_state[0:162]>\r\n");
    PRINT("    \t Sets the BSEC algorithm state \r\n");
    PRINT("    \t <algo_state[0:162]> : BSEC algorithm state \r\n");
    PRINT("    \t -e.g. bsecsetalstate 1 2 ... 163\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecsetalstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsetalstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint8_t i;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_bsec_param_algo_state state;

    INFO("Executing %s\r\n", argv[0]);
    PRINT("BSEC algorithm state: \r\n");
    for (i = 1U; i < BHY_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH + 1U; i++)
    {
        PRINT("%s ", argv[i]);
    }

    PRINT("\r\n");

    for (i = 0U; i < BHY_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH; i++)
    {
        state.algo_state[i] = (uint8_t)string_to_int((char*)argv[i + 1]);
    }

    (void)(bhy_bsec_param_set_algo_state(&state, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for bsecgetalstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgetalstate_help(void *ref)
{
    (void)ref;

    PRINT("  bsecgetalstate\r\n");
    PRINT("    \t Gets the BSEC algorithm state \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecgetalstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgetalstate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint8_t i;
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_bsec_param_algo_state state;

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_bsec_param_get_algo_state(&state, &cli_ref->bhy));

    PRINT("BSEC algorithm state: \r\n");
    for (i = 0U; i < BHY_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH; i++)
    {
        PRINT("%u ", state.algo_state[i]);
    }

    PRINT("\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for bsecsettempoff command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsettempoff_help(void *ref)
{
    (void)ref;

    PRINT("  bsecsettempoff <offset>\r\n");
    PRINT("    \t Sets the BSEC temperature offset \r\n");
    PRINT("    \t <offset> : BSEC temperature offset \r\n");
    PRINT("    \t -e.g. bsecsettempoff 1.0\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecsettempoff command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsettempoff_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    union bhy_float_conv offset;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    offset.f_val = (float)string_to_int((char*)argv[1]);
    (void)(bhy_bsec_param_set_temp_offset(&offset, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for syssetphyseninfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetphyseninfo_help(void *ref)
{
    (void)ref;

    PRINT("  syssetphyseninfo <sensor_id> <orientation_matrix>\r\n");
    PRINT("    \t= Set the orientation of Physical sensor\r\n");
    PRINT("    \t= Eg: syssetphyseninfo 1 0,-1,0,1,0,0,0,0,1 \r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for bsecgettempoff command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgettempoff_help(void *ref)
{
    (void)ref;

    PRINT("  bsecgettempoff\r\n");
    PRINT("    \t Gets the BSEC temperature offset \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgetphysenlist command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetphysenlist_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    (void)bhy_system_param_get_physical_sensor_present(&cli_ref->bhy);

    PRINT("\r\n");
    PRINT("Physical sensor list.\r\n");
    PRINT("Sensor ID |                          Sensor Name\r\n");
    PRINT("----------+--------------------------------------|\r\n");
    for (uint8_t i = 0; i < BHY_PHYSICAL_SENSOR_ID_MAX; i++)
    {
        if (bhy_is_physical_sensor_available(i, &cli_ref->bhy))
        {
            PRINT(" %8u | %36s \r\n", i, get_physical_sensor_name(i));
        }
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgetphysenlist command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetphysenlist_help(void *ref)
{
    (void)ref;

    PRINT("  sysgetphysenlist\r\n");
    PRINT("    \t= List of physical sensor present\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgetvirsenlist command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetvirtsenlist_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s\r\n", argv[0]);

    (void)bhy_update_virtual_sensor_list(&cli_ref->bhy);

    /* Get present virtual sensor */
    (void)bhy_system_param_get_virtual_sensor_present(&cli_ref->bhy);

    PRINT("Virtual sensor list.\r\n");
    PRINT("Sensor ID |                          Sensor Name |\r\n");
    PRINT("----------+--------------------------------------|\r\n");
    for (uint8_t i = 0; i < BHY_SENSOR_ID_MAX; i++)
    {
        if (bhy_is_sensor_available(i, &cli_ref->bhy))
        {
            if (i < BHY_SENSOR_ID_CUSTOM_START)
            {
                PRINT(" %8u | %36s \r\n", i, get_sensor_name(i));
            }
            else
            {
                PRINT(" %8u | %36s \r\n", i, custom_driver_information[i - BHY_SENSOR_ID_CUSTOM_START].sensor_name);
            }
        }
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgetvirsenlist command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetvirtsenlist_help(void *ref)
{
    (void)ref;

    PRINT("  sysgetvirsenlist\r\n");
    PRINT("    \t= List of virtual sensor present\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecgettempoff command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgettempoff_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    union bhy_float_conv offset;

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_bsec_param_get_temp_offset(&offset, &cli_ref->bhy));

    PRINT("BSEC temperature offset: %f\r\n", offset.f_val);

    return CLI_OK;
}

/**
* @brief Function to print help for bsecsetsamrate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsetsamrate_help(void *ref)
{
    (void)ref;

    PRINT("  bsecsetsamrate <sample_rate>\r\n");
    PRINT("    \t Sets the BSEC sample rate \r\n");
    PRINT("    \t <sample_rate> : BSEC sample rate. Valid value: 1.0, 0.33333, 0.0033333 \r\n");
    PRINT("    \t -e.g. bsecsetsamrate 0.3333\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecsetsamrate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsetsamrate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_bsec_param_sample_rate sample_rate;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    sample_rate.sample_rate_index = (bhy_bsec_param_sample_rate_index)string_to_int((char*)argv[1]);
    (void)(bhy_bsec_param_set_sample_rate(&sample_rate, &cli_ref->bhy));

    return CLI_OK;
}

/**
* @brief Function to print help for bsecgetsamrate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgetsamrate_help(void *ref)
{
    (void)ref;

    PRINT("  bsecgetsamrate\r\n");
    PRINT("    \t Gets the BSEC sample rate \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for bsecgetsamrate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgetsamrate_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_bsec_param_sample_rate sample_rate;
    char* sample_rate_act[] =
    { "BSEC_PARAM_SAMPLE_RATE_CONT", "BSEC_PARAM_SAMPLE_RATE_LP", "BSEC_PARAM_SAMPLE_RATE_ULP" };

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_bsec_param_get_sample_rate(&sample_rate, &cli_ref->bhy));

    PRINT("BSEC sample rate: %u (%s)\r\n", sample_rate.sample_rate_index,
          sample_rate_act[sample_rate.sample_rate_index]);

    return CLI_OK;
}

/**
* @brief Function to print help for sethearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sethearactvcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  sethearactvcnfg <ss> <ppe> <mingt> <maxgt> <obs> <msmc>\r\n");
    PRINT("    \t= Set hearable activity configuration\r\n");
    PRINT("    \t-<ss> : Segment size values : 0 to 2\r\n");
    PRINT("    \t-<ppe> : post process values : 0/1\r\n");
    PRINT("    \t-<mingt> : minimum gdi threshold values : 0 to 4095\r\n");
    PRINT("    \t-<maxgt> : maximum gdi threshold values : 0 to 4095\r\n");
    PRINT("    \t-<obs> : output buffer size values : 0 to 10\r\n");
    PRINT("    \t-<msmc> : segment moderate config : 0 to 10\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sethearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sethearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_activity_param_hearable hearable = { 0 };

    hearable.seg_size = (uint16_t)atoi((char *)argv[1]);
    hearable.post_process_en = (uint16_t)atoi((char *)argv[2]);
    hearable.min_gdi_thre = (uint16_t)atoi((char *)argv[3]);
    hearable.max_gdi_thre = (uint16_t)atoi((char *)argv[4]);
    hearable.out_buff_size = (uint16_t)atoi((char *)argv[5]);
    hearable.min_seg_moder_conf = (uint16_t)atoi((char *)argv[6]);

    if ((hearable.min_gdi_thre <= 4095) && (hearable.max_gdi_thre <= 4095) && (hearable.seg_size <= 2) &&
        (hearable.out_buff_size <= 10) && (hearable.min_seg_moder_conf <= 10) && (hearable.post_process_en <= 1))
    {
        PRINT("Set hearable activity configuration\r\n");
        PRINT("    - seg_size: %u\r\n", hearable.seg_size);
        PRINT("    - post_process_en: %u\r\n", hearable.post_process_en);
        PRINT("    - min_gdi_thre: %u\r\n", hearable.min_gdi_thre);
        PRINT("    - max_gdi_thre: %u\r\n", hearable.max_gdi_thre);
        PRINT("    - out_buff_size: %u\r\n", hearable.out_buff_size);
        PRINT("    - min_seg_moder_confg: %u\r\n", hearable.min_seg_moder_conf);
        (void)(bhy_activity_param_set_hearable_config(&hearable, &cli_ref->bhy));
    }
    else
    {
        return CLI_E_INVALID_PARAM;
    }

    return CLI_OK;
}

/**
* @brief Function to print help for gethearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gethearactvcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  gethearactvcnfg\r\n");
    PRINT("    \t= Get hearable activity configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for gethearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gethearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_activity_param_hearable hearable = { 0 };

    (void)(bhy_activity_param_get_hearable_config(&hearable, &cli_ref->bhy));

    PRINT("Hearable activity configuration:\r\n");
    PRINT("    - seg_size: %u\r\n", hearable.seg_size);
    PRINT("    - post_process_en: %u\r\n", hearable.post_process_en);
    PRINT("    - min_gdi_thre: %u\r\n", hearable.min_gdi_thre);
    PRINT("    - max_gdi_thre: %u\r\n", hearable.max_gdi_thre);
    PRINT("    - out_buff_size: %u\r\n", hearable.out_buff_size);
    PRINT("    - min_seg_moder_confg: %u\r\n", hearable.min_seg_moder_conf);

    return CLI_OK;
}

/**
* @brief Function to print help for setwearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setwearactvcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  setwearactvcnfg <ppe> <mingt> <maxgt> <obs> <msmc>\r\n");
    PRINT("    \t= Set wearable activity configuration\r\n");
    PRINT("    \t-<ppe> : post process values : 0/1\r\n");
    PRINT("    \t-<mingt> : minimum gdi threshold values : 0 to 4095\r\n");
    PRINT("    \t-<maxgt> : maximum gdi threshold values : 0 to 4095\r\n");
    PRINT("    \t-<obs> : output buffer size values : 0 to 10\r\n");
    PRINT("    \t-<msmc> : segment moderate config : 0 to 10\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for setwearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setwearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_activity_param_wearable wearable = { 0 };

    wearable.post_process_en = (uint16_t)atoi((char *)argv[1]);
    wearable.min_gdi_thre = (uint16_t)atoi((char *)argv[2]);
    wearable.max_gdi_thre = (uint16_t)atoi((char *)argv[3]);
    wearable.out_buff_size = (uint16_t)atoi((char *)argv[4]);
    wearable.min_seg_moder_conf = (uint16_t)atoi((char *)argv[5]);

    if ((wearable.min_gdi_thre <= 4095) && (wearable.max_gdi_thre <= 4095) && (wearable.out_buff_size <= 10) &&
        (wearable.min_seg_moder_conf <= 10) && (wearable.post_process_en <= 1))
    {
        PRINT("Set wearable activity configuration\r\n");
        PRINT("    - post_process_en: %u\r\n", wearable.post_process_en);
        PRINT("    - min_gdi_thre: %u\r\n", wearable.min_gdi_thre);
        PRINT("    - max_gdi_thre: %u\r\n", wearable.max_gdi_thre);
        PRINT("    - out_buff_size: %u\r\n", wearable.out_buff_size);
        PRINT("    - min_seg_moder_confg: %u\r\n", wearable.min_seg_moder_conf);
        (void)(bhy_activity_param_set_wearable_config(&wearable, &cli_ref->bhy));
    }
    else
    {
        return CLI_E_INVALID_PARAM;
    }

    return CLI_OK;
}

/**
* @brief Function to print help for getwearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getwearactvcnfg_help(void *ref)
{
    (void)ref;

    PRINT("  getwearactvcnfg\r\n");
    PRINT("    \t= Get wearable activity configuration\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for getwearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getwearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)argv;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_activity_param_wearable wearable = { 0 };

    (void)(bhy_activity_param_get_wearable_config(&wearable, &cli_ref->bhy));
    PRINT("Wearable activity configuration:\r\n");
    PRINT("    - post_process_en: %u\r\n", wearable.post_process_en);
    PRINT("    - min_gdi_thre: %u\r\n", wearable.min_gdi_thre);
    PRINT("    - max_gdi_thre: %u\r\n", wearable.max_gdi_thre);
    PRINT("    - out_buff_size: %u\r\n", wearable.out_buff_size);
    PRINT("    - min_seg_moder_confg: %u\r\n", wearable.min_seg_moder_conf);

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgettimestamps command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgettimestamps_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_timestamp ts;

    INFO("Executing %s\r\n", argv[0]);
    (void)bhy_system_param_get_timestamps(&ts, &cli_ref->bhy);

    uint32_t s, ns;
    ts.host_int_ts *= 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(ts.host_int_ts / UINT64_C(1000000000));
    ns = (uint32_t)(ts.host_int_ts - (s * UINT64_C(1000000000)));

    PRINT("Host interrupt timestamp: %lu.%09lu\r\n", s, ns);

    ts.cur_ts *= 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(ts.cur_ts / UINT64_C(1000000000));
    ns = (uint32_t)(ts.cur_ts - (s * UINT64_C(1000000000)));
    PRINT("Current timestamp: %lu.%09lu\r\n", s, ns);

    ts.event_ts *= 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(ts.event_ts / UINT64_C(1000000000));
    ns = (uint32_t)(ts.event_ts - (s * UINT64_C(1000000000)));
    PRINT("Timestamp event: %lu.%09lu\r\n", s, ns);

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgettimestamps command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgettimestamps_help(void *ref)
{
    (void)ref;

    PRINT("  sysgettimestamps\r\n");
    PRINT("    \t= Get the system timestamps\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgetfwversion command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetfwversion_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_firmware_version fw_ver;

    INFO("Executing %s\r\n", argv[0]);
    (void)bhy_system_param_get_firmware_version(&fw_ver, &cli_ref->bhy);

    PRINT("\r\n");
    PRINT("Custom version number: %u\r\n", fw_ver.custom_ver_num);

#ifdef PC
    PRINT("EM Hash: %llx\r\n", fw_ver.em_hash);
    PRINT("BST Hash: %llx\r\n", fw_ver.bst_hash);
    PRINT("User Hash: %llx\r\n", fw_ver.user_hash);
#else
    char temp_em_hash[48] = { '\0' }, temp1_em_hash[32] = { '\0' }, temp2_em_hash[16] = { '\0' };
#ifdef PC
    sprintf(temp1_em_hash, "%lx", fw_ver.em_hash >> 16);
    sprintf(temp2_em_hash, "%x", fw_ver.em_hash & 0xFFFF);
#else
    sprintf(temp1_em_hash, "%llx", fw_ver.em_hash >> 16);
    sprintf(temp2_em_hash, "%llx", fw_ver.em_hash & 0xFFFF);
#endif
    strcat(temp_em_hash, temp1_em_hash);
    strcat(temp_em_hash, temp2_em_hash);
    PRINT("EM Hash: %s\r\n", temp_em_hash);

    char temp_bst_hash[48] = { '\0' }, temp1_bst_hash[32] = { '\0' }, temp2_bst_hash[16] = { '\0' };
    sprintf(temp1_bst_hash, "%lx", (long unsigned int)fw_ver.bst_hash >> 16);
    sprintf(temp2_bst_hash, "%x", (unsigned int)fw_ver.bst_hash & 0xFFFF);

    strcat(temp_bst_hash, temp1_bst_hash);
    strcat(temp_bst_hash, temp2_bst_hash);
    PRINT("BST Hash: %s\r\n", temp_bst_hash);

    char temp_user_hash[48] = { '\0' }, temp1_user_hash[32] = { '\0' }, temp2_user_hash[16] = { '\0' };
    sprintf(temp1_user_hash, "%lx", (long unsigned int)fw_ver.user_hash >> 16);
    sprintf(temp2_user_hash, "%x", (unsigned int)fw_ver.user_hash & 0xFFFF);

    strcat(temp_user_hash, temp1_user_hash);
    strcat(temp_user_hash, temp2_user_hash);
    PRINT("User Hash: %s\r\n", temp_user_hash);
#endif
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgetfwversion command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetfwversion_help(void *ref)
{
    (void)ref;

    PRINT("  sysgetfwversion\r\n");
    PRINT("    \t= Get the system firmware version\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgetfifoctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetfifoctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_system_param_fifo_control fifo_ctrl;

    INFO("Executing %s\r\n", argv[0]);

    (void)bhy_system_param_get_fifo_control(&fifo_ctrl, &cli_ref->bhy);

    PRINT("\r\n");
    PRINT("Wakeup FIFO Watermark = %lu\r\n", fifo_ctrl.wakeup_fifo_watermark);
    PRINT("Wakeup FIFO size =  %lu\r\n", fifo_ctrl.wakeup_fifo_size);
    PRINT("Non Wakeup FIFO Watermark = %lu\r\n", fifo_ctrl.non_wakeup_fifo_watermark);
    PRINT("Non Wakeup FIFO size = %lu\r\n", fifo_ctrl.non_wakeup_fifo_size);

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgetfifoctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetfifoctrl_help(void *ref)
{
    (void)ref;

    PRINT("  sysgetfifoctrl\r\n");
    PRINT("    \t= System get fifo control\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for syssetwkffctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetwkffctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint32_t water_mark;

    INFO("Executing %s\r\n", argv[0]);

    water_mark = (uint32_t)atoi((const char *)argv[1]);
    struct bhy_system_param_fifo_control fifo_ctrl;

    fifo_ctrl.wakeup_fifo_watermark = water_mark;

    assert_rslt = bhy_system_param_set_wakeup_fifo_control(&fifo_ctrl, &cli_ref->bhy);

    if (assert_rslt)
    {
        PRINT("FIFO wake-up watermark SET Failed %d", assert_rslt);
    }
    else
    {
        PRINT("FIFO wake-up watermark SET Success");
    }

    PRINT("\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for syssetwkffctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetwkffctrl_help(void *ref)
{
    (void)ref;

    PRINT("  syssetwkffctrl <watermark value>\r\n");
    PRINT("    \t= System set watermark for wake-up fifo control\r\n");
    PRINT("    \t= Eg: syssetwkffctrl 500\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for syssetnwkffctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetnwkffctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint32_t water_mark;

    INFO("Executing %s\r\n", argv[0]);

    water_mark = (uint32_t)atoi((const char *)argv[1]);
    struct bhy_system_param_fifo_control fifo_ctrl;

    fifo_ctrl.non_wakeup_fifo_watermark = water_mark;

    assert_rslt = bhy_system_param_set_wakeup_fifo_control(&fifo_ctrl, &cli_ref->bhy);

    if (assert_rslt)
    {
        PRINT("FIFO non wake-up watermark SET Failed %d", assert_rslt);
    }
    else
    {
        PRINT("FIFO non wake-up watermark SET Success");
    }

    PRINT("\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for syssetnwkffctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetnwkffctrl_help(void *ref)
{
    (void)ref;

    PRINT("  syssetnwkffctrl <watermark value>\r\n");
    PRINT("    \t= System set watermark for non wake-up fifo control\r\n");
    PRINT("    \t= Eg: syssetnwkffctrl 500\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for sysgetmectrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetmectrl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint16_t addr;

    INFO("Executing %s\r\n", argv[0]);

    addr = (uint16_t)string_to_int((char *)argv[1]);
    bhy_system_param_multi_meta_event_ctrl_t meta_event;

    (void)bhy_system_param_get_meta_event_control(addr, &meta_event, &cli_ref->bhy);

    PRINT("\r\n");
    PRINT("Meta event infomation:\r\n");
    for (uint8_t loop = 0; loop < 8; loop++)
    {
        PRINT("%d %d %d %d %d %d %d %d\r\n",
              meta_event.group[loop].as_s.meta_event4_enable_state,
              meta_event.group[loop].as_s.meta_event4_int_enable_state,
              meta_event.group[loop].as_s.meta_event3_enable_state,
              meta_event.group[loop].as_s.meta_event3_int_enable_state,
              meta_event.group[loop].as_s.meta_event2_enable_state,
              meta_event.group[loop].as_s.meta_event2_int_enable_state,
              meta_event.group[loop].as_s.meta_event1_enable_state,
              meta_event.group[loop].as_s.meta_event1_int_enable_state);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for sysgetmectrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetmectrl_help(void *ref)
{
    (void)ref;

    PRINT("  sysgetmectrl <address>\r\n");
    PRINT("    \t= System get meta event control\r\n");
    PRINT("    \t   0x101 -  non wake-up fifo control\r\n");
    PRINT("    \t   0x102 -  wake-up fifo control\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for syssetmectrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetmectrl_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint16_t addr;
    uint8_t group_idx;
    uint8_t value;

    INFO("Executing %s\r\n", argv[0]);

    addr = (uint16_t)string_to_int((char *)argv[1]);
    group_idx = (uint8_t)atoi((char *)argv[2]);
    value = (uint8_t)atoi((char *)argv[3]);

    bhy_system_param_multi_meta_event_ctrl_t meta_event;

    meta_event.group[group_idx].as_uint8 = value;

    assert_rslt = bhy_system_param_set_meta_event_control(addr, &meta_event, &cli_ref->bhy);

    if (assert_rslt)
    {
        PRINT("System meta event control SET Failed %d\r\n\r\n\r\n", assert_rslt);
    }
    else
    {
        PRINT("System meta event control SET Success\r\n\r\n\r\n");
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for syssetmectrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetmectrl_help(void *ref)
{
    (void)ref;

    PRINT("  syssetmectrl <address> <group idx> <value>\r\n");
    PRINT("    \t= System set meta event control\r\n");
    PRINT("    \t   0x101 -  non wake-up fifo control\r\n");
    PRINT("    \t   0x102 -  wake-up fifo control\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for setbsxparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setbsxparam_help(void *ref)
{
    (void)ref;

    PRINT("  setbsxparam <parameter_id> or <parameter_id> <file_name>\r\n");
    PRINT("    \t -Set the BSX calibration states for a specific physical sensor parameter id\r\n");
    PRINT(
        "    \t -<parameter_id> : ID of the parameter (e.g., '0x201' for accelerometer, '0x203' for gyroscope, '0x205' for magnetometer)\r\n");
    PRINT(
        "    \t <file_name> : Name of the file from where the calibration state is read, provide the file name with '.txt'as extension\r\n");
    PRINT("    \t Send just <parameter_id> to set the latest calibration profile \r\n");
    PRINT("    \t Send <parameter_id> <file_name> to set already saved calibration profile from a file\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for setbsxparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setbsxparam_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint32_t actual_len = 0;
    uint16_t param_id;
    char str_param_id[10] = { 0 };
    char file_name[MAX_FILENAME_LENGTH] = { 0 };
    char file_ext[6] = { 0 };
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    bhy_bsx_algo_param_state_exg bsx_state_exg[20] = { { 0 } };
    char buffer[512] = { 0 };
    int8_t ret_val = 0;
    uint32_t bytes_read = 0;

    strncpy(str_param_id, (char *)argv[1], strlen((char *)argv[1]));
    str_param_id[strlen((char *)argv[1])] = '\0';

    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    if (argc >= 2)
    {
        INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    }
    else
    {
        INFO("Executing %s %s\r\n", argv[0], argv[1]);
    }

    PRINT("\r\n\r\n");

#ifdef PC
    if (argc >= 3)
#else
    if (argc >= 2)
#endif
    {
        /* Get the calibration from the file and set the same */
        strncpy(file_name, (char *)argv[2], strlen((char *)argv[2]));
        file_name[strlen((char *)argv[2])] = '\0';

        uint32_t len = strlen(file_name);

        for (uint32_t idx = 0; idx < len; idx++)
        {
            if (file_name[idx] == '.')
            {
                strcpy(file_ext, (file_name + (idx + 1)));
                break;
            }
        }

        file_ext[strlen(file_ext)] = '\0';

        ret_val = (int8_t)strcmp(file_ext, "txt");
        if (ret_val == 0)
        {
            uint8_t file_idx = 0;

            FILE *fp = fopen(file_name, "r");

            if (fp == NULL)
            {
                PRINT("Error opening file!\r\n");

                return CLI_E_NULL_PTR;
            }

            fseek(fp, 0, SEEK_SET);

            for (;;)
            {
                bytes_read = fread(buffer, 1, (BHY_BSX_STATE_STRUCT_LEN + 2), fp);
                if (file_idx == 0 && bytes_read == 0)
                {
                    PRINT("Error in reading the file content\r\n");

                    return CLI_E_FILE_CMD;
                }
                else if (bytes_read == 0)
                {
                    break;
                }

                (void)(bhy_set_parameter(param_id, (uint8_t *)buffer, bytes_read - 2, &cli_ref->bhy));
                memset(buffer, 0, sizeof(buffer));
                file_idx++;

            }

            PRINT("Calibration profile for BSX parameter id 0x%04X is read from the file %s and calibrated\r\n",
                  param_id,
                  file_name);
            fclose(fp);
        }
        else
        {
            PRINT("Invalid file extension. Please provide a file with .txt extension\r\n");

            return CLI_E_INVALID_PARAM;
        }
    }
    else
    {
        /* Get bsx states for physical sensors*/
        (void)(bhy_bsx_algo_param_get_bsx_states(param_id, bsx_state_exg, sizeof(bsx_state_exg), &actual_len,
                                                 &cli_ref->bhy));

        /* Set bsx states for physical sensors*/
        (void)(bhy_bsx_algo_param_set_bsx_states(param_id, bsx_state_exg, &cli_ref->bhy));

        PRINT("\r\n");
        PRINT("Setting Calibration profile of BSX parameter id 0x%04X is completed \r\n", param_id);
    }

    memset(bsx_state_exg, 0, sizeof(bsx_state_exg));
    PRINT("\r\n\r\n");

    return CLI_OK;

}

/**
* @brief Function to print help for getbsxparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getbsxparam_help(void *ref)
{
    (void)ref;

    PRINT("  getbsxparam <parameter_id> or <parameter_id> <file_name>\r\n");
    PRINT("    \t -Get the BSX calibration states for a specific physical sensor parameter id\r\n");
    PRINT(
        "    \t -<parameter_id> : ID of the parameter (e.g., '0x201' for accelerometer, '0x203' for gyroscope, '0x205' for magnetometer)\r\n");
    PRINT(
        "    \t <file_name> : Name of the file to which the calibration state is written, provide the file name with '.txt'as extension\r\n");
    PRINT("    \t Send just <parameter_id> to see the calibration profile on the console\r\n");
    PRINT("    \t Send <parameter_id> <file_name> to save the calibration profile to the file\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for getbsxparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getbsxparam_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    uint32_t actual_len = 0;
    uint16_t param_id;
    char str_param_id[10] = { 0 };
    char file_name[MAX_FILENAME_LENGTH] = { 0 };
    char file_ext[6] = { 0 };
    int8_t ret_val;
    bhy_bsx_algo_param_state_exg bsx_state_exg[BHY_BSX_STATE_MAX_BLOCKS];
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    strncpy(str_param_id, (char *)argv[1], strlen((char *)argv[1]));
    str_param_id[strlen((char *)argv[1])] = '\0';

    param_id = (uint16_t)strtol(str_param_id, NULL, 0);

    if (argc >= 2)
    {
        INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);
    }
    else
    {
        INFO("Executing %s %s\r\n", argv[0], argv[1]);
    }

    PRINT("\r\n\r\n");

    PRINT("Get Calibration profile of BSX parameter id: 0x%04X\r\n", param_id);

    /* Get bsx states for physical sensors*/
    (void)(bhy_bsx_algo_param_get_bsx_states(param_id, bsx_state_exg, sizeof(bsx_state_exg), &actual_len,
                                             &cli_ref->bhy));
#ifdef PC
    if (argc >= 3)
#else
    if (argc >= 2)
#endif
    {
        /* Create a file and copy the calibration profile to the file */
        strncpy(file_name, (char *)argv[2], strlen((char *)argv[2]));
        file_name[strlen((char *)argv[2])] = '\0';

        for (uint8_t i = 0; i < strlen(file_name); i++)
        {
            if (file_name[i] == '.')
            {
                strcpy(file_ext, (file_name + (i + 1)));
                break;
            }
        }

        file_ext[strlen(file_ext)] = '\0';

        ret_val = (int8_t)strcmp(file_ext, "txt");

        if (ret_val == 0)
        {
            #ifndef PC
            if (file_exists(file_name))
            {
                (void)remove(file_name);
            }

            #endif

            FILE *fp = fopen(file_name, "w");
            uint32_t bytes_written = 0;

            if (fp == NULL)
            {
                PRINT("Error opening file!\r\n");

                return CLI_E_NULL_PTR;
            }

            for (uint8_t block = 0; block < BHY_BSX_STATE_MAX_BLOCKS; block++)
            {
                bhy_bsx_algo_param_state_exg *current_block = &bsx_state_exg[block];

                bytes_written = fwrite(current_block, 1, (BHY_BSX_STATE_STRUCT_LEN), fp);

                if (bytes_written != (uint32_t)BHY_BSX_STATE_STRUCT_LEN)
                {
                    PRINT("Error writing to the file!\r\n");

                    return CLI_E_FILE_CMD;
                }

                fprintf(fp, "\r\n");

                if ((current_block->block_len == 0) ||
                    (current_block->block_info & BHY_BSX_STATE_TRANSFER_COMPLETE) != 0)
                {
                    fclose(fp);
                    break;
                }
            }

            PRINT("\r\n");
            PRINT("Calibration profile for BSX parameter id 0x%04X is saved to the file %s\r\n", param_id, file_name);
        }
        else
        {
            PRINT("Invalid file extension. Please provide a file with .txt extension\r\n");

            return CLI_E_INVALID_PARAM;
        }
    }
    else
    {
        print_bsx_algo_param_states(bsx_state_exg);
    }

    memset(bsx_state_exg, 0, sizeof(bsx_state_exg));

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for getbsxver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getbsxver_help(void *ref)
{
    (void)ref;

    PRINT("  getbsxver \r\n");
    PRINT("    \t -Get the BSX version\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for getbsxver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getbsxver_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_bsx_algo_param_version bsx_version = { 0 };

    INFO("Executing %s\r\n", argv[0]);

    (void)(bhy_bsx_algo_param_get_bsx_version(&bsx_version, &cli_ref->bhy));

    PRINT("BSX version: %u.%u.%u.%u\r\n\r\n",
          bsx_version.major_version,
          bsx_version.minor_version,
          bsx_version.major_bug_fix_version,
          bsx_version.minor_bug_fix_version);

    return CLI_OK;

}

/**
* @brief Function to print help for virtseinfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t virtseinfo_help(void *ref)
{
    (void)ref;

    PRINT("  virtseinfo <sensor_id>\r\n");
    PRINT("    \t= Get virtual sensor information parameters\r\n");
    PRINT("    \t <sensor_id> : virtual sensor ID (1 - 191)\r\n");
    PRINT("    \t -e.g. virtseinfo 1 will get virtual sensor information parameters for accelerometer\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for virtseinfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t virtseinfo_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    struct bhy_virtual_sensor_info_param_info info;
    uint8_t sensor_id;

    sensor_id = (uint8_t)string_to_int((char *)argv[1]);

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    (void)(bhy_virtual_sensor_info_param_get_info(sensor_id, &info, &cli_ref->bhy));

    PRINT("Virtual Sensor Information:\r\n");
    PRINT("    Sensor ID: %u\r\n", info.sensor_type);
    PRINT("    Driver ID: %u\r\n", info.driver_id);
    PRINT("    Driver version: %u\r\n", info.driver_version);
    PRINT("    Power: %u\r\n", info.power);
    PRINT("    Max range: %u\r\n", info.max_range.u16_val);
    PRINT("    Resolution: %u\r\n", info.resolution.u16_val);
    PRINT("    Max rate: %f\r\n", info.max_rate.f_val);
#ifdef PC
    PRINT("    FIFO reserved: %u\r\n", info.fifo_reserved.u32_val);
    PRINT("    FIFO max: %u\r\n", info.fifo_max.u32_val);
#else
    PRINT("    FIFO reserved: %lu\r\n", info.fifo_reserved.u32_val);
    PRINT("    FIFO max: %lu\r\n", info.fifo_max.u32_val);
#endif
    PRINT("    Event size: %u\r\n", info.event_size);
    PRINT("    Min rate: %f\r\n", info.min_rate.f_val);

    return CLI_OK;
}

/**
* @brief Function to print help for phyrangeconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t phyrangeconf_help(void *ref)
{
    (void)ref;

    PRINT("  phyrangeconf <sensor_id> <range_value>\r\n");
    PRINT("    \t= Setting the range of physical sensor\r\n");
    PRINT("    \t <sensor_id> : virtual sensor ID, for Accelerometer: 1, Gyroscope: 10, Magnetometer: 19\r\n");
    PRINT("    \t <range_value> : physical range configuration\r\n");
    PRINT("    \t -e.g. phyrangeconf 1 0x10  will set the value 16G for accelerometer\r\n");
    PRINT("    \t -Using physeninfo command to check the modified value\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for phyrangeconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t phyrangeconf_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;

    INFO("Executing %s %s %s\r\n", argv[0], argv[1], argv[2]);

    uint8_t sen_id;
    uint16_t value;
    char str_value_id[8] = { 0 };

    sen_id = (uint8_t)atoi((char *)argv[1]);
    strncpy(str_value_id, (char *)argv[2], strlen((char *)argv[2]));
    str_value_id[strlen((char *)argv[2])] = '\0';
    value = (uint16_t)strtol(str_value_id, NULL, 0);

    (void)(bhy_set_virt_sensor_range(sen_id, value, &cli_ref->bhy));
    PRINT("Setting the range of sensor successfully\r\n");

    return CLI_OK;
}

/**
* @brief Function to configure sensor
* @param[in] sen_cfg    : Sensor configuration
* @param[in] sen_id     : Sensor ID
* @param[in] parse_flag : Parse flag
* @param[in] ref        : Reference to device and command line
*/
static void configure_sensor(struct bhy_virtual_sensor_conf_param_conf sen_cfg,
                             uint8_t sen_id,
                             uint8_t parse_flag,
                             struct bhy_cli_ref *ref)
{
    uint8_t phy_sen_id;
    struct bhy_dev *bhy = &(ref->bhy);
    struct parse_ref *parse_table = &(ref->parse_table);
    bhy_fifo_parse_callback_t callback;
    struct parse_sensor_details *sensor_details;
    struct bhy_system_param_phys_sensor_info psi = { 0 };

    if (first_run)
    {
        first_run = false;
        (void)bhy_update_virtual_sensor_list(bhy);

        /* Get present virtual sensor */
        (void)bhy_system_param_get_virtual_sensor_present(bhy);
    }

    /* If the payload of this sensor is not yet registered and within the custom virtual sensor id range, register the
     * default parsing function */
    if (bhy_is_sensor_available(sen_id, bhy))
    {
        if ((sen_id >= BHY_SENSOR_ID_CUSTOM_START) && (sen_id <= BHY_SENSOR_ID_CUSTOM_END) &&
            (custom_driver_information[sen_id - BHY_SENSOR_ID_CUSTOM_START].is_registered != 1))
        {
            custom_driver_information[sen_id - BHY_SENSOR_ID_CUSTOM_START].sensor_payload = bhy->event_size[sen_id];

            (void)(bhy_register_fifo_parse_callback(sen_id, parse_custom_sensor_default, parse_table, bhy));
            PRINT("No output interpretation has been provided for this sensor. ");
            PRINT("FIFO data will be printed as hex values. ");
            PRINT("For registering the payload interpretation, use the addse option\r\n");
        }
    }
    else
    {
        ERROR("The requested sensor is not present in the loaded firmware!\r\n");

        return;
    }

    PRINT("Sensor ID: %u, sample rate: %f Hz, latency: %lu ms\r\n", sen_id, sen_cfg.sample_rate, sen_cfg.latency);

    sensor_details = parse_add_sensor_details(sen_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Insufficient parsing slots\r\n");

        return;
    }

    /* If through the logging command, check for valid sample rate before enabling logging */
    if (sen_cfg.sample_rate > 0.0f)
    {
        /* Register if not already requested earlier */
        if (sensor_details->parse_flag == PARSE_FLAG_NONE)
        {
            callback = bhy_get_callback(sen_id);
            (void)(bhy_register_fifo_parse_callback(sen_id, callback, parse_table, bhy));
        }

        sensor_details->parse_flag = PARSE_SET_FLAG(sensor_details->parse_flag, parse_flag);

        phy_sen_id = get_physical_sensor_id(sen_id);
        if (phy_sen_id == BHY_PHYS_SENSOR_ID_NOT_SUPPORTED)
        {
            /* FW does not support this physical sensor. Use default scaling factor */
            sensor_details->scaling_factor = get_sensor_default_scaling(sen_id, bhy);
        }
        else
        {
            /* FW supports this physical sensor. Use dynamic scaling factor from sensor */
            (void)(bhy_system_param_get_physical_sensor_info(phy_sen_id, &psi, bhy));
            sensor_details->scaling_factor = get_sensor_dynamic_range_scaling(sen_id, (float)psi.curr_range.u16_val);
        }

        sensors_active[sen_id] = true;

        /* Flush sensor data from the FIFO */
        (void)(bhy_flush_fifo(sen_id, bhy));

        /* Enable sensor and set sample rate if there is a source requesting it */
        if (sensor_details->parse_flag != PARSE_FLAG_NONE)
        {
            (void)(bhy_virtual_sensor_conf_param_set_cfg(sen_id, &sen_cfg, bhy));
        }
    }
    else
    {
        /* Flush sensor data from the FIFO */
        (void)(bhy_flush_fifo(sen_id, bhy));

        sensor_details->parse_flag = PARSE_CLEAR_FLAG(sensor_details->parse_flag, parse_flag);

        /* Disable if there is no source requesting it */
        if (sensor_details->parse_flag == PARSE_FLAG_NONE)
        {
            (void)(bhy_virtual_sensor_conf_param_set_cfg(sen_id, &sensor_conf, bhy));
            sensors_active[sen_id] = false;
            (void)(bhy_deregister_fifo_parse_callback(sen_id, bhy));
            sensor_details->id = 0;
        }
    }

    /* Sensor data will be parsed and printed after processing all arguments */
}

/**
* @brief Function to configure for sensors in logandstream command
* @param[in] cli_ref  : Reference to device and command line
*/
static void log_configure_sensor(struct bhy_cli_ref *cli_ref)
{
    for (uint16_t idx = 1; idx <= num_sensor; idx++)
    {
        if (enable_ds[sensor_list[idx]] == false)
        {
            if (sen_cfg_list[idx].sample_rate > 50.0f)
            {
                enable_ds[sensor_list[idx]] = true;
                odr_ds[sensor_list[idx]] = (uint16_t)((round)((double)sen_cfg_list[idx].sample_rate / 50));
                configure_sensor(sen_cfg_list[idx],
                                 (uint8_t)sensor_list[idx],
                                 PARSE_FLAG_STREAM | PARSE_FLAG_LOG,
                                 cli_ref);
            }
            else
            {
                configure_sensor(sen_cfg_list[idx],
                                 (uint8_t)sensor_list[idx],
                                 PARSE_FLAG_STREAM | PARSE_FLAG_LOG,
                                 cli_ref);
            }
        }
        else
        {
            if (sen_cfg_list[idx].sample_rate <= ds_list[idx])
            {
                enable_ds[sensor_list[idx]] = false;
                configure_sensor(sen_cfg_list[idx],
                                 (uint8_t)sensor_list[idx],
                                 PARSE_FLAG_STREAM | PARSE_FLAG_LOG,
                                 cli_ref);
            }
            else
            {
                odr_ds[sensor_list[idx]] = (uint16_t)((round)((double)sen_cfg_list[idx].sample_rate / ds_list[idx]));
                DATA("Streaming sensor ID %d with sample rate is %fHz\r\n", sensor_list[idx], ds_list[idx]);
                configure_sensor(sen_cfg_list[idx],
                                 (uint8_t)sensor_list[idx],
                                 PARSE_FLAG_STREAM | PARSE_FLAG_LOG,
                                 cli_ref);
            }
        }
    }
}

/**
* @brief Function to create a log file
* @param[in] file_name_t : Reference to buffer to store file name
* @param[in] argv        : Argument for file name
* @param[in] cli_ref     : Reference to device and command line
*/
static void create_log(char *file_name_t, char *argv, struct bhy_cli_ref *cli_ref)
{
    if (cli_ref->parse_table.logdev.logfile == NULL)
    {
        PRINT("Creating %s\r\n", argv);

        cli_ref->parse_table.logdev.logfile = fopen(argv, "wb");
        memcpy(cli_ref->parse_table.logdev.logfilename, argv, strlen(argv));
        memcpy(file_name_t, argv, strlen(argv));
        if (cli_ref->parse_table.logdev.logfile)
        {
            PRINT("File %s was created\r\n", cli_ref->parse_table.logdev.logfilename);
            write_meta_info(&cli_ref->parse_table.logdev, &cli_ref->bhy);
        }
        else
        {
            ERROR("File %s could not be found/created\r\n", cli_ref->parse_table.logdev.logfilename);
        }
    }
    else
    {
        PRINT("File %s is open. Please use 'detlog' to detach the open file\r\n",
              cli_ref->parse_table.logdev.logfilename);
    }
}

/**
* @brief Function to print help for logandstream command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t logandstream_help(void *ref)
{
    (void)ref;

    PRINT("  logandstream <sensor id>:<frequency>[:<latency>][:<downsampling>] <filename> <start>\r\n");
    PRINT("  OR\r\n");
    PRINT("  logandstream <stop>\r\n");
    PRINT(
        "    \t= Streamming data for sensor <sensor id> at specified sample rate <frequency> or at <downsampling> and logging to <filename>\r\n");
    PRINT("    \t -<sensor id> is required, virtual sensor id\r\n");
    PRINT("    \t -<frequency> is required\r\n");
    PRINT("    \t -<latency> is optional\r\n");
    PRINT("    \t -<downsampling> is optional, default is 50Hz\r\n");
    PRINT("    \t -<filename>: is required, name of file for logging\r\n");
    PRINT("    \t -<start>/<stop>: is required, trigger/stop condition\r\n");
    PRINT("  Example usage:\r\n");
    PRINT("    \t logandstream 3:50::10 4:100::20 log.bin start\r\n");
    PRINT("    \t logandstream stop\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for logandstream command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t logandstream_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;

    PRINT("Executing log and stream together\r\n");
    const char start_str[] = "start";
    const char stop_str[] = "stop";
    struct bhy_cli_ref *cli_ref = (struct bhy_cli_ref *)ref;
    uint16_t idx = 0;

    if (strncmp((char *)(argv[1]), stop_str, strlen(stop_str)) != 0)
    {
        for (idx = 1; idx < MAXIMUM_VIRTUAL_SENSOR_LIST; idx++)
        {
            if (argv[idx] == NULL)
            {
                break;
            }
        }

        num_sensor = idx - 3;

        char sen_id_str[8], sample_rate_str[8], latency_str[8], downstream_str[8];
        char *start, *end;

        if (lognstream_inprogress)
        {
            PRINT(
                "There is another logandstream command has already executed now, please stop it before executing new command\r\n");

            return CLI_E_INVALID_PARAM;
        }
        else
        {
            lognstream_inprogress = true;
        }

        for (idx = 1; idx <= num_sensor; idx++)
        {
            /* Parse Sensor ID */
            start = (char *)(argv[idx]);
            end = strchr(start, ':');
            if (end == NULL)
            {
                ERROR("Sensor ID / Sample rate format error\r\n");
                lognstream_inprogress = false;

                return CLI_E_NULL_PTR;
            }

            strncpy(sen_id_str, start, (size_t)(end - start));
            sen_id_str[end - start] = '\0';
            sensor_list[idx] = (uint8_t)atoi(sen_id_str);

            /* Parse sample rate */
            start = end + 1;
            end = strchr(start, ':');

            if (end == NULL)
            {
                end = start + strlen(start);
            }

            strncpy(sample_rate_str, start, (size_t)(end - start));
            sample_rate_str[end - start] = '\0';
            sen_cfg_list[idx].sample_rate = (float)atof(sample_rate_str);

            if (sen_cfg_list[idx].sample_rate <= 0.0f)
            {
                PRINT(
                    "Incorrect value, please input positive value for sample rate to execute log and stream command\r\n");
                lognstream_inprogress = false;

                return CLI_E_INVALID_PARAM;
            }

            sen_cfg_list[idx].latency = 0;

            /*  Parse Latency */
            if (strlen(end))
            {
                start = end + 1;
                end = strchr(start, ':');

                if (end == NULL)
                {
                    end = start + strlen(start);
                }

                strncpy(latency_str, start, (size_t)(end - start));
                latency_str[end - start] = '\0';
                sen_cfg_list[idx].latency = (uint32_t)atoi(latency_str);
            }

            /* Parse downstream rate*/
            ds_list[idx] = 0.0f;
            if (strlen(end))
            {
                start = end + 1;
                end = strchr(start, ':');

                if (end == NULL)
                {
                    end = start + strlen(start);
                }

                strncpy(downstream_str, start, (size_t)(end - start));
                downstream_str[end - start] = '\0';
                ds_list[idx] = (float)atof(downstream_str);
                if (ds_list[idx] != 0.0f)
                {
                    enable_ds[sensor_list[idx]] = true;
                }
            }
        }

        if (strncmp((char *)argv[num_sensor + 2], start_str, strlen(start_str)) != 0)
        {
            PRINT("Incorrect argument, please input 'start' as a last argument in the command\r\n");
            lognstream_inprogress = false;

            return CLI_E_INVALID_PARAM;
        }

        create_log(file_name, (char *)argv[num_sensor + 1], cli_ref);

        log_configure_sensor(cli_ref);
    }
    else
    {
        if (!(lognstream_inprogress))
        {
            PRINT("There is no log and stream command in progressing now\r\n");

            return CLI_OK;
        }

        PRINT("Stopping log and stream command ...\r\n");

        struct bhy_dev *bhy = &(cli_ref->bhy);
        struct parse_ref *parse_table = &(cli_ref->parse_table);
        struct parse_sensor_details *sensor_details;

        for (idx = 1; idx <= num_sensor; idx++)
        {
            sensor_details = parse_add_sensor_details((uint8_t)sensor_list[idx], parse_table);

            /* Flush sensor data from the FIFO */
            (void)(bhy_flush_fifo((uint8_t)sensor_list[idx], bhy));

            /* Disable if there is no source requesting it */
            sensor_details->parse_flag = PARSE_FLAG_NONE;
            sensor_conf.sample_rate = 0;
            (void)(bhy_virtual_sensor_conf_param_set_cfg((uint8_t)sensor_list[idx], &sensor_conf, bhy));
            sensors_active[sensor_list[idx]] = false;
            (void)(bhy_deregister_fifo_parse_callback((uint8_t)sensor_list[idx], bhy));
            sensor_details->id = 0;
        }

        /* Detach logging*/
        if (cli_ref->parse_table.logdev.logfile != NULL)
        {
            fclose(cli_ref->parse_table.logdev.logfile);
            cli_ref->parse_table.logdev.logfile = NULL;
            PRINT("File %s was detached for logging\r\n", cli_ref->parse_table.logdev.logfilename);
            memset(cli_ref->parse_table.logdev.logfilename, 0, sizeof(cli_ref->parse_table.logdev.logfilename));
        }
        else
        {
            PRINT("No file to detach\r\n");
        }

        /* Reset all necessary variable */
        for (int loop = 0; loop < MAXIMUM_VIRTUAL_SENSOR_LIST; loop++)
        {
            enable_ds[sensor_list[loop]] = false;
            odr_ds[sensor_list[loop]] = 0;
        }

        num_sensor = 0;
        lognstream_inprogress = false;
        memset(file_name, '\0', sizeof(file_name));
        memset(sensor_list, 0, sizeof(sensor_list));
        memset(ds_list, 0, sizeof(ds_list));
        memset(sen_cfg_list, 0, sizeof(struct bhy_virtual_sensor_conf_param_conf));
    }

    return CLI_OK;
}
