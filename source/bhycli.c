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
 * @file    bhycli.c
 * @brief   Command line utility for the BHI260/BHA260
 *
 */

#ifdef __STDC_ALLOC_LIB__
#define __STDC_WANT_LIB_EXT2__  1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include "bhy.h"
#include "parse.h"
#include "cli.h"
#include "common.h"
#include "coines.h"
#include "bhycli_callbacks.h"
#include "common_callbacks.h"
#include "verbose.h"

#define LIFE_LED_PERIOD_MS  UINT32_C(2500)
#define LIFE_LED_DUR_MS     UINT32_C(50)

bool echo_on = true;
bool heartbeat_on = false;
uint16_t stream_buff_len = 0;
#ifndef PC
static uint8_t out_buff[CLI_STREAM_BUF_MAX] = { 0 };
#endif
static uint16_t out_idx = 0;
#ifdef PC
static volatile bool end_streaming = false;
#endif
#ifndef PC
static uint8_t *argv[50] = { 0 };
static uint8_t argc = 0;
uint8_t inp[10240] = { 0 };
enum coines_comm_intf bhycli_intf = COINES_COMM_INTF_BLE;
#endif
static struct bhy_cli_ref cli_ref = { { { 0 } } };

bool cmd_in_process = false;

#ifdef PC

/**
* @brief Callback function for signal interrupt
* @param[in] sig_num : Signal number
*/
void sigint_handler(int sig_num)
{
    (void)sig_num;
    (void)signal(SIGINT, NULL);

    if (end_streaming)
    {
        INFO("Force exit\r\n");
        exit(0);
    }

    INFO("\r\nExiting\r\n");
    end_streaming = true;
}
#endif

#ifdef PC

/**
* @brief Function to parse COM port information from the command-line arguments
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @return COM port on success, or NULL on failure
*/
char* parse_com_port_from_args(int argc, char *argv[])
{
    const char *port_str = "port";

    for (int idx = 1; idx < argc; idx++)
    {
        if ((strncmp(argv[idx], port_str, strlen(port_str)) == 0) && (idx + 1 < argc))
        {
            return argv[idx + 1];
        }
    }

    return NULL;
}
#endif

#ifdef PC

/**
* @brief Function to perform initialization sequence
* @param[in] intf     : Type of interface
* @param[in] com_port : COM port
*/
void perform_init_seq(enum bhy_intf intf, char *com_port)
#else

/**
* @brief Function to perform initialization sequence
* @param[in] intf           : Type of interface
* @param[in] bytes_read_now : Pointer to buffer stored number of read bytes
*/
void perform_init_seq(enum bhy_intf intf, uint16_t *bytes_read_now)
#endif
{
    /* Execution starts here */
#ifdef PC
    if (com_port)
    {
        setup_interfaces(false, intf, com_port);
    }
    else
    {
        setup_interfaces(false, intf, NULL);
    }

#else
    setup_interfaces((bool)true, intf, NULL);
#endif

    /* Disable printf buffering for stdout */
    setbuf(stdout, NULL);

#ifndef PC
    if (coines_intf_connected(COINES_COMM_INTF_BLE))
    {
        bhycli_intf = COINES_COMM_INTF_BLE;
    }
    else
    {
        bhycli_intf = COINES_COMM_INTF_USB;
    }

    /* Read and discard any data from the buffer */
    if (coines_intf_available(bhycli_intf))
    {
        *bytes_read_now = coines_read_intf(bhycli_intf, inp, coines_intf_available(bhycli_intf));
    }

    PRINT("\033c\033[2J"); /* Clear screen */
    PRINT("Type help to view the list of available commands\r\n");
#endif
}

#ifndef PC

/**
* @brief Function to split input buffer
* @param[in] bytes_read : Pointer to buffer stored number of read bytes
*/
void split_input_buffer(uint16_t *bytes_read)
{
    uint16_t inp_idx;

    /* Split the inp buffer into argv[]. */
    if (!cmd_in_process)
    {
        cmd_in_process = true;
        argc = 0;
        argv[argc++] = &inp[0];

        /*lint -e{850} inp_idx is modified in the body of the 'for' loop */
        for (inp_idx = 0; inp_idx < (*bytes_read); inp_idx++)
        {
            /* Wait for the termination of an argument */
            if ((inp[inp_idx] != ' ') && (inp[inp_idx] != '\r') && (inp[inp_idx] != '\n') && (inp_idx < (*bytes_read)))
            {
                continue;
            }

            /* Skip over whitespace, replacing it with '\0'. */
            while (((inp[inp_idx] == ' ') || (inp[inp_idx] == '\r') || (inp[inp_idx] == '\n')) &&
                   (inp_idx < (*bytes_read)))
            {
                inp[inp_idx] = '\0';
                inp_idx++;
            }

            /* Are more arguments present? */
            if (inp_idx < (*bytes_read))
            {
                argv[argc++] = &inp[inp_idx];
            }
        }

        (*bytes_read) = 0;
    }
}
#endif

/**************************************************/
/*                     MAIN                       */
/**************************************************/

#ifdef PC

/**
* @brief Main function for processing command line
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @return API error codes
*/
int main(int argc, char *argv[])
{
    (void)signal(SIGINT, sigint_handler);
    bool first_run = true;
#else

/**
* @brief Main function for processing command line
* @return API error codes
*/
int main(void)
{
    uint16_t bytes_read, bytes_read_now = 0;
    uint8_t nl = '\n';

#endif
#ifndef PC
    uint32_t blink_on = 0, blink_off = 0;
    bool led_state = false;
#endif

    cli_ref.cli_dev.n_cmds = bhy_get_n_cli_callbacks();
    cli_ref.cli_dev.ref = &cli_ref;
    cli_ref.cli_dev.table = bhy_get_cli_callbacks();
    cli_ref.parse_table.bhy = &cli_ref.bhy;
    int8_t cli_ret;
    enum bhy_intf intf;
#ifdef BHY_USE_I2C
    intf = BHY_I2C_INTERFACE;
#else
    intf = BHY_SPI_INTERFACE;
#endif

#ifdef PC
    char *com_port = parse_com_port_from_args(argc, argv);
    perform_init_seq(intf, com_port);
#else
    perform_init_seq(intf, &bytes_read_now);
#endif

    bhy_callbacks_init(&cli_ref);

#ifdef PC
    while (bhy_are_sensors_active() || first_run)
    {
        first_run = false;
        if (end_streaming)
        {
            break;
        }

#else
    bytes_read = 0;
    (void)bytes_read_now;
    bytes_read_now = 0;
    cmd_in_process = true;

    for (;;)
    {
#endif

#ifndef PC
        if (coines_intf_connected(COINES_COMM_INTF_BLE))
        {
            bhycli_intf = COINES_COMM_INTF_BLE;
        }
        else
        {
            bhycli_intf = COINES_COMM_INTF_USB;
        }

        /* Read data from the service and append into the inp buffer */
        if (coines_intf_available(bhycli_intf))
        {
            bytes_read_now = coines_read_intf(bhycli_intf, &inp[bytes_read], coines_intf_available(bhycli_intf));

            /* Only for use with the terminal. Comment the following line otherwise */
            if (echo_on)
            {
                coines_write_intf(bhycli_intf, &inp[bytes_read], bytes_read_now); /* Echo back the input */
            }

            bytes_read += bytes_read_now;
            cmd_in_process = true;

            /* Check the inp buffer for a termination character. */
            for (uint16_t i = 0; i < bytes_read; i++)
            {
                if ((inp[i] == '\r') || (inp[i] == '\n'))
                {
                    cmd_in_process = false;

                    /* Only for use with the terminal. Comment the following line otherwise */
                    if (echo_on)
                    {
                        coines_write_intf(bhycli_intf, &nl, 1);
                    }
                }
            }
        }

        split_input_buffer(&bytes_read);

#endif

        /* If there are arguments, process them */
        if (argc)
        {
            cli_ret = cli_run(argc, (uint8_t **)argv, &cli_ref.cli_dev);
            if (cli_ret)
            {
                ERROR("Invalid command. Type help for detailed overview of available commands\r\n\r\n", cli_ret);
            }

            for (int i = 0; i < argc; i++)
            {
                argv[i] = NULL;
            }

            argc = 0;
        }

        if (stream_buff_len && out_idx)
        {
#ifndef PC
            (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            coines_write_intf(bhycli_intf, out_buff, out_idx);
            out_idx = 0;
            (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
#endif
        }

        bhy_data_parse_callback(&cli_ref);
#ifdef PC
        if (!bhy_are_sensors_active())
        {
            break;
        }

#endif
#ifndef PC
        if (blink_on <= coines_get_millis())
        {
            blink_on = coines_get_millis() + LIFE_LED_PERIOD_MS;
            blink_off = coines_get_millis() + LIFE_LED_DUR_MS;
            (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
            led_state = true;
            if (heartbeat_on)
            {
                PRINT("[H]%u\r\n", coines_get_millis());
                coines_flush_intf(bhycli_intf);
            }
        }

        if (led_state && (blink_off <= coines_get_millis()))
        {
            (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
            led_state = false;
            if (heartbeat_on)
            {
                PRINT("[H]%u\r\n", coines_get_millis());
                coines_flush_intf(bhycli_intf);
            }
        }

#endif

        /*lint -e527*/
#ifdef PC
        if (end_streaming)
        {
            bhy_exit(&cli_ref);
        }

#endif

    }

    close_interfaces(intf);

    return 0;
}

/**
* @brief Function to write verbose information
* @param[in] buffer : Pointer to buffer which stored information
* @param[in] length : Length of data
*/
void verbose_write(uint8_t *buffer, uint16_t length)
{
#ifndef PC
    if (stream_buff_len)
    {
        if ((out_idx + length) > sizeof(out_buff))
        {
            (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            coines_write_intf(bhycli_intf, out_buff, out_idx);
            out_idx = 0;
            (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
        }

        memcpy(&out_buff[out_idx], buffer, length);
        out_idx += length;
    }
    else
    {
        (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
        coines_write_intf(bhycli_intf, buffer, length);
        (void)coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
    }

#endif
}
