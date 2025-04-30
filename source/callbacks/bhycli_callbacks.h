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
 * @file    bhycli_callbacks.h
 * @brief   Header file for the command line utility callbacks
 *
 */

#ifndef _BHYCLI_CALLBACKS_H_
#define _BHYCLI_CALLBACKS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>

#include "cli.h"

#include "bhy2.h"

#include "bhy_klio_param.h"
#include "bhy_system_param.h"
#include "bhy_swim_param.h"
#include "bhy_multi_tap_param.h"
#include "bhy_head_orientation_param.h"
#include "parse.h"
#include "bhy_event_data.h"

struct bhy_cli_ref
{
    struct bhy2_dev bhy;
    cli_dev_t cli_dev;
    struct parse_ref parse_table;
};

/**
* @brief Function to get all command lines callback
* @return Reference to callback table
*/
cli_callback_table_t * bhy_get_cli_callbacks(void);

/**
* @brief Function to get number of command line callbacks
* @return Number of command line callbacks
*/
uint8_t bhy_get_n_cli_callbacks(void);

/**
* @brief Function to initialize interface and sensor callbacks
* @param[in] cli_ref  : Reference to command line
*/
void bhy_callbacks_init(struct bhy_cli_ref  *cli_ref);

/**
* @brief Function to install virtual sensor callbacks
* @param[in] bhy         : Device instance
* @param[in] parse_table : Pointer to parse table
*/
void bhy_install_callbacks(struct bhy2_dev *bhy, struct parse_ref *parse_table);

/**
* @brief Function to check whether any sensor is active or not
* @return Sensor active status
*/
bool bhy_are_sensors_active(void);

/**
* @brief Function to deactivate all sensors and release some resources
* @param[in] cli_ref  : Reference to command line
*/
void bhy_exit(struct bhy_cli_ref  *cli_ref);

/**
* @brief Function to parse the callback
* @param[in] cli_ref  : Reference to command line
*/
void bhy_data_parse_callback(struct bhy_cli_ref *cli_ref);

/**
* @brief Function to get the parse callback
* @param[in] sensor_id : Sensor ID
* @return Parse callback
*/
bhy2_fifo_parse_callback_t bhy_get_callback(uint8_t sensor_id);

#if 0

/**
* @brief Function to print help for kstatus command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kstatus_help(void * const ref);

/**
* @brief Function to implement callback for kstatus command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kstatus_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksetstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetstate_help(void *ref);

/**
* @brief Function to implement callback for ksetstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksetgstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetgstate_help(void *ref);

/**
* @brief Function to implement callback for ksetgstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetgstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kgetgstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetgstate_help(void *ref);

/**
* @brief Function to implement callback for kgetgstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetgstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksetgestconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetgestconf_help(void *ref);

/**
* @brief Function to implement callback for ksetgestconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetgestconf_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksettimeconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksettimeconf_help(void *ref);

/**
* @brief Function to implement callback for ksettimeconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksettimeconf_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kgetstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetstate_help(void *ref);

/**
* @brief Function to implement callback for kgetstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kreset command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kreset_help(void *ref);

/**
* @brief Function to implement callback for kreset command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kreset_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kldpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kldpatt_help(void *ref);

/**
* @brief Function to implement callback for kldpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kldpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kldgpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kldgpatt_help(void *ref);

/**
* @brief Function to implement callback for kldgpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kldgpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kenpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kenpatt_help(void *ref);

/**
* @brief Function to implement callback for kenpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kenpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kdispatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kdispatt_help(void *ref);

/**
* @brief Function to implement callback for kdispatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kdispatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kdisapatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kdisapatt_help(void *ref);

/**
* @brief Function to implement callback for kdisapatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kdisapatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kswpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kswpatt_help(void *ref);

/**
* @brief Function to implement callback for kswpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kswpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kautldpatt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kautldpatt_help(void *ref);

/**
* @brief Function to implement callback for kautldpatt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kautldpatt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kgetparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kgetparam_help(void *ref);

/**
* @brief Function to implement callback for kgetparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kgetparam_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksetparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksetparam_help(void *ref);

/**
* @brief Function to implement callback for ksetparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksetparam_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for kmsimscore command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t kmsimscore_help(void *ref);

/**
* @brief Function to implement callback for kmsimscore command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t kmsimscore_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ksimscore command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ksimscore_help(void *ref);

/**
* @brief Function to implement callback for ksimscore command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ksimscore_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#endif

/**
* @brief Function to print help for version command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t version_help(void *ref);

/**
* @brief Function to implement callback for version command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t version_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for help command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t help_help(void *ref);

/**
* @brief Function to implement callback for help command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t help_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for info command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t info_help(void *ref);

/**
* @brief Function to implement callback for info command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t info_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ramb command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ramb_help(void *ref);

/**
* @brief Function to implement callback for ramb command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ramb_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#if 0

/**
* @brief Function to print help for flb command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t flb_help(void *ref);

/**
* @brief Function to implement callback for flb command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t flb_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#endif

/**
* @brief Function to print help for reset command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t reset_help(void *ref);

/**
* @brief Function to implement callback for reset command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t reset_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for addse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t addse_help(void *ref);

/**
* @brief Function to implement callback for addse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t addse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for rd command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rd_help(void *ref);

/**
* @brief Function to implement callback for rd command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rd_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wr command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wr_help(void *ref);

/**
* @brief Function to implement callback for wr command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wr_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for rdp command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rdp_help(void *ref);

/**
* @brief Function to implement callback for rdp command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rdp_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wrp command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wrp_help(void *ref);

/**
* @brief Function to implement callback for wrp command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wrp_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for physeninfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t physeninfo_help(void *ref);

/**
* @brief Function to implement callback for physeninfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t physeninfo_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for ram command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ram_help(void *ref);

/**
* @brief Function to implement callback for ram command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ram_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#if 0

/**
* @brief Function to print help for fl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t fl_help(void *ref);

/**
* @brief Function to implement callback for fl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t fl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#endif

/**
* @brief Function to print help for boot command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t boot_help(void *ref);

/**
* @brief Function to implement callback for boot command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t boot_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#if 0

/**
* @brief Function to print help for erase command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t erase_help(void *ref);

/**
* @brief Function to implement callback for erase command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t erase_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for efd command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t efd_help(void *ref);

/**
* @brief Function to implement callback for efd command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t efd_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#endif

/**
* @brief Function to print help for actse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t actse_help(void *ref);

/**
* @brief Function to implement callback for actse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t actse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for dinject command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dinject_help(void *ref);

/**
* @brief Function to implement callback for dinject command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dinject_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for schema command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t schema_help(void *ref);

/**
* @brief Function to implement callback for schema command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t schema_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hexse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hexse_help(void *ref);

/**
* @brief Function to implement callback for hexse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hexse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for logse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t logse_help(void *ref);

/**
* @brief Function to implement callback for logse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t logse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for attlog command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t attlog_help(void *ref);

/**
* @brief Function to implement callback for attlog command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t attlog_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for detlog command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t detlog_help(void *ref);

/**
* @brief Function to implement callback for detlog command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t detlog_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for slabel command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t slabel_help(void *ref);

/**
* @brief Function to implement callback for slabel command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t slabel_callback(uint8_t argc, uint8_t * const argv[], void *ref);

#if 0

/**
* @brief Function to print help for swim command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swim_help(void *ref);

/**
* @brief Function to implement callback for swim command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swim_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for swimgetaxes command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimgetaxes_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for swimsetaxes command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetaxes_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for swimsetlogging command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetlogging_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for swimsetlogging command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetlogging_help(void *ref);

/**
* @brief Function to print help for swimgetaxes command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimgetaxes_help(void *ref);

/**
* @brief Function to print help for swimsetaxes command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetaxes_help(void *ref);

#endif

/**
* @brief Function to implement callback for setvirtsenconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setvirtsenconf_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for setvirtsenconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setvirtsenconf_help(void *ref);

/**
* @brief Function to implement callback for getvirtsenconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getvirtsenconf_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for getvirtsenconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getvirtsenconf_help(void *ref);

#if 0

/**
* @brief Function to print help for swimver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimver_help(void *ref);

/**
* @brief Function to implement callback for swimver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimver_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for swimgetfreq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimgetfreq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for swimsetfreq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t swimsetfreq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for swimgetfreq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimgetfreq_help(void *ref);

/**
* @brief Function to print help for swimsetfreq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t swimsetfreq_help(void *ref);

#endif

/**
* @brief Function to print help for dmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dmode_help(void *ref);

/**
* @brief Function to implement callback for dmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dmode_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for pm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t pm_help(void *ref);

/**
* @brief Function to implement callback for pm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t pm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for dactse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t dactse_help(void *ref);

/**
* @brief Function to implement callback for dactse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t dactse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for lsactse command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t lsactse_help(void *ref);

/**
* @brief Function to implement callback for lsactse command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t lsactse_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for mtapen command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapen_help(void *ref);

/**
* @brief Function to implement callback for mtapen command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapen_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for mtapinfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapinfo_help(void *ref);

/**
* @brief Function to implement callback for mtapinfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapinfo_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for mtapsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for mtapsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for mtapgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mtapgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for mtapgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mtapgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accsetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetfoc_help(void *ref);

/**
* @brief Function to implement callback for accsetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accgetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetfoc_help(void *ref);

/**
* @brief Function to implement callback for accgetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accsetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetpwm_help(void *ref);

/**
* @brief Function to implement callback for accsetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accgetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetpwm_help(void *ref);

/**
* @brief Function to implement callback for accgetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accsetar command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accsetar_help(void *ref);

/**
* @brief Function to implement callback for accsetar command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accsetar_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accgetar command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetar_help(void *ref);

/**
* @brief Function to implement callback for accgetar command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetar_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for acctrignvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t acctrignvm_help(void *ref);

/**
* @brief Function to implement callback for acctrignvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t acctrignvm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for accgetnvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t accgetnvm_help(void *ref);

/**
* @brief Function to implement callback for accgetnvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t accgetnvm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetfoc_help(void *ref);

/**
* @brief Function to implement callback for gyrosetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetfoc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetfoc_help(void *ref);

/**
* @brief Function to implement callback for gyrogetfoc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetfoc_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosetois command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetois_help(void *ref);

/**
* @brief Function to implement callback for gyrosetois command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetois_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetois command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetois_help(void *ref);

/**
* @brief Function to implement callback for gyrogetois command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetois_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosetfs command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetfs_help(void *ref);

/**
* @brief Function to implement callback for gyrosetfs command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetfs_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetfs command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetfs_help(void *ref);

/**
* @brief Function to implement callback for gyrogetfs command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetfs_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosetcrt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetcrt_help(void *ref);

/**
* @brief Function to implement callback for gyrosetcrt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetcrt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetcrt command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetcrt_help(void *ref);

/**
* @brief Function to implement callback for gyrogetcrt command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetcrt_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosetpwm_help(void *ref);

/**
* @brief Function to implement callback for gyrosetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetpwm_help(void *ref);

/**
* @brief Function to implement callback for gyrogetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrosettat command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrosettat_help(void *ref);

/**
* @brief Function to implement callback for gyrosettat command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrosettat_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogettat command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogettat_help(void *ref);

/**
* @brief Function to implement callback for gyrogettat command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogettat_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrotrignvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrotrignvm_help(void *ref);

/**
* @brief Function to implement callback for gyrotrignvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrotrignvm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gyrogetnvm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gyrogetnvm_help(void *ref);

/**
* @brief Function to implement callback for gyrogetnvm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gyrogetnvm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for magsetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t magsetpwm_help(void *ref);

/**
* @brief Function to implement callback for magsetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t magsetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for maggetpwm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t maggetpwm_help(void *ref);

/**
* @brief Function to implement callback for maggetpwm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t maggetpwm_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wwwsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wwwsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for wwwsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wwwsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wwwgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wwwgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for wwwgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wwwgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for amsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t amsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for amsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t amsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for amgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t amgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for amgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t amgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for nmsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t nmsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for nmsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t nmsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for nmgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t nmgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for nmgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t nmgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wgdsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wgdsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for wgdsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wgdsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for wgdgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wgdgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for wgdgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wgdgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for baro1setcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro1setcnfg_help(void *ref);

/**
* @brief Function to implement callback for baro1setcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro1setcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for baro1getcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro1getcnfg_help(void *ref);

/**
* @brief Function to implement callback for baro1getcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro1getcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for baro2setcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro2setcnfg_help(void *ref);

/**
* @brief Function to implement callback for baro2setcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro2setcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for baro2getcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t baro2getcnfg_help(void *ref);

/**
* @brief Function to implement callback for baro2getcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t baro2getcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for scsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t scsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for scsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t scsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for scgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t scgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for scgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t scgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmctrig command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmctrig_help(void *ref);

/**
* @brief Function to implement callback for hmctrig command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmctrig_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcsetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetcnfg_help(void *ref);

/**
* @brief Function to implement callback for hmcsetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcgetcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetcnfg_help(void *ref);

/**
* @brief Function to implement callback for hmcgetcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcsetdefcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetdefcnfg_help(void *ref);

/**
* @brief Function to implement callback for hmcsetdefcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetdefcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcver_help(void *ref);

/**
* @brief Function to implement callback for hmcver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcver_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcsetcalcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetcalcorrq_help(void *ref);

/**
* @brief Function to implement callback for hmcsetcalcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetcalcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcgetcalcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetcalcorrq_help(void *ref);

/**
* @brief Function to implement callback for hmcgetcalcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetcalcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcsetmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcsetmode_help(void *ref);

/**
* @brief Function to implement callback for hmcsetmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcsetmode_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hmcgetmode command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hmcgetmode_help(void *ref);

/**
* @brief Function to implement callback for hmcgetmode command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hmcgetmode_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hosetheadcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hosetheadcorrq_help(void *ref);

/**
* @brief Function to implement callback for hosetheadcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hosetheadcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hogetheadcorrq command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hogetheadcorrq_help(void *ref);

/**
* @brief Function to implement callback for hogetheadcorrq command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hogetheadcorrq_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hover command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hover_help(void *ref);

/**
* @brief Function to implement callback for hover command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hover_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hosetheadcorre command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hosetheadcorre_help(void *ref);

/**
* @brief Function to implement callback for hosetheadcorre command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hosetheadcorre_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for hogetheadcorre command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t hogetheadcorre_help(void *ref);

/**
* @brief Function to implement callback for hogetheadcorre command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t hogetheadcorre_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for foc command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t foc_help(void *ref);

/**
* @brief Function to implement callback for foc command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t foc_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for chipid command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getchipid_help(void *ref);

/**
* @brief Function to implement callback for chipid command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getchipid_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for syssetphyseninfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetphyseninfo_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for syssetphyseninfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetphyseninfo_help(void *ref);

/**
* @brief Function to implement callback for sysgetvirsenlist command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetvirtsenlist_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgetvirsenlist command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetvirtsenlist_help(void *ref);

/**
* @brief Function to implement callback for sysgetphysenlist command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetphysenlist_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgetphysenlist command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetphysenlist_help(void *ref);

/**
* @brief Function to implement callback for sysgettimestamps command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgettimestamps_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgettimestamps command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgettimestamps_help(void *ref);

/**
* @brief Function to implement callback for sysgetfwversion command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetfwversion_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgetfwversion command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetfwversion_help(void *ref);

/**
* @brief Function to implement callback for sysgetfifoctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetfifoctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgetfifoctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetfifoctrl_help(void *ref);

/**
* @brief Function to implement callback for syssetwkffctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetwkffctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for syssetwkffctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetwkffctrl_help(void *ref);

/**
* @brief Function to implement callback for syssetnwkffctrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetnwkffctrl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for syssetnwkffctrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetnwkffctrl_help(void *ref);

/**
* @brief Function to implement callback for sysgetmectrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sysgetmectrl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sysgetmectrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sysgetmectrl_help(void *ref);

/**
* @brief Function to implement callback for syssetmectrl command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t syssetmectrl_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for syssetmectrl command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t syssetmectrl_help(void *ref);

/**
* @brief Function to print help for bsecsetalstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsetalstate_help(void *ref);

/**
* @brief Function to implement callback for bsecsetalstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsetalstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for bsecgetalstate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgetalstate_help(void *ref);

/**
* @brief Function to implement callback for bsecgetalstate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgetalstate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for bsecsettempoff command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsettempoff_help(void *ref);

/**
* @brief Function to implement callback for bsecsettempoff command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsettempoff_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for bsecgettempoff command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgettempoff_help(void *ref);

/**
* @brief Function to implement callback for bsecgettempoff command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgettempoff_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for bsecsetsamrate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecsetsamrate_help(void *ref);

/**
* @brief Function to implement callback for bsecsetsamrate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecsetsamrate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for bsecgetsamrate command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t bsecgetsamrate_help(void *ref);

/**
* @brief Function to implement callback for bsecgetsamrate command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t bsecgetsamrate_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to implement callback for sethearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t sethearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for sethearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t sethearactvcnfg_help(void *ref);

/**
* @brief Function to implement callback for gethearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t gethearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for gethearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t gethearactvcnfg_help(void *ref);

/**
* @brief Function to implement callback for setwearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setwearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for setwearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setwearactvcnfg_help(void *ref);

/**
* @brief Function to implement callback for getwearactvcnfg command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getwearactvcnfg_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for getwearactvcnfg command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getwearactvcnfg_help(void *ref);

/**
* @brief Function to print help for setbsxparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t setbsxparam_help(void *ref);

/**
* @brief Function to implement callback for setbsxparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t setbsxparam_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for getbsxparam command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getbsxparam_help(void *ref);

/**
* @brief Function to implement callback for getbsxparam command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getbsxparam_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for getbsxver command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t getbsxver_help(void *ref);

/**
* @brief Function to implement callback for getbsxver command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t getbsxver_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for virtseinfo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t virtseinfo_help(void *ref);

/**
* @brief Function to implement callback for virtseinfo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t virtseinfo_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for phyrangeconf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t phyrangeconf_help(void *ref);

/**
* @brief Function to implement callback for phyrangeconf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t phyrangeconf_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/**
* @brief Function to print help for logandstream command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t logandstream_help(void *ref);

/**
* @brief Function to implement callback for logandstream command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t logandstream_callback(uint8_t argc, uint8_t * const argv[], void *ref);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHYCLI_CALLBACKS_H_ */
