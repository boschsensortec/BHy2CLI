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
 * @file    common_callbacks.c
 * @brief   Source file for the command line utility callbacks
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <math.h>
#include <sys/stat.h>
#if defined(PC)
#include <dirent.h>
#endif

#include "common_callbacks.h"
#include "coines.h"
#include "verbose.h"

extern bool echo_on;
extern bool heartbeat_on;
extern uint16_t stream_buff_len;

extern uint8_t inp[10240];
extern enum coines_comm_intf bhy2cli_intf;
extern bool cmd_in_process;
write_file wfile;

#ifndef PC
#if (defined(MCU_APP30) || defined(MCU_APP31))

/**
* @brief Function to get size of file
* @param[in] file_name : File name
* @return Size of file
*/
static size_t get_file_size(const char *file_name);

#endif

/**
* @brief Function to implement callback for loop to write content to a file
* @param[in] echo_data : option to enable/disable echoing data
*/
void wrfile_loop_callabck(bool echo_data);

/**
* @brief Function to print help for echo command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t echo_help(void *ref)
{
    (void)ref;

    PRINT("  echo <on / off>\r\n");
    PRINT("    \t= Set the echo on or off\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for echo command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t echo_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    if (!strcmp((char *)argv[1], "on"))
    {
        echo_on = true;
    }
    else
    {
        echo_on = false;
    }

    PRINT("Setting echo to %s\r\n", echo_on ? "on" : "off");
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for heart command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t heartbeat_help(void *ref)
{
    (void)ref;

    PRINT("  heart <on / off>\r\n");
    PRINT("    \t= Set the heartbeat message on or off\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for heart command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t heartbeat_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    if (!strcmp((char *)argv[1], "on"))
    {
        heartbeat_on = true;
    }
    else
    {
        heartbeat_on = false;
    }

    PRINT("Setting Heartbeat message to %s\r\n", heartbeat_on ? "on" : "off");
    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for strbuf command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t streambuff_help(void *ref)
{
    (void)ref;

    PRINT("  strbuf <buffer size>\r\n");
    PRINT("    \t= Set the streaming buffer size. Maximum of %u bytes\r\n", CLI_STREAM_BUF_MAX);

    return CLI_OK;
}

/**
* @brief Function to implement callback for strbuf command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t streambuff_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    uint16_t buffer_size;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    buffer_size = (uint16_t)atoi((char *)argv[1]);

    if (buffer_size <= CLI_STREAM_BUF_MAX)
    {
        PRINT("Streaming buffer size set to %u\r\n", buffer_size);
        stream_buff_len = buffer_size;
    }
    else
    {
        ERROR("Invalid streaming buffer size of %u\r\n", buffer_size);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for mklog command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t mklog_help(void *ref)
{
    (void)ref;

    PRINT("  mklog <filename.ext>\r\n");
    PRINT("    \t= Create a log file (write-only)\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for mklog command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t mklog_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);

    FILE *fp = fopen((char *)argv[1], "wb");

    if (fp)
    {
        PRINT("File %s was created\r\n", argv[1]);
        fclose(fp);
    }
    else
    {
        ERROR("File %s could not be created\r\n", argv[1]);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for rm command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rm_help(void *ref)
{
    (void)ref;

    PRINT("  rm <filename.ext>\r\n");
    PRINT("    \t= Remove a file\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for rm command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rm_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)argc;
    (void)ref;

    INFO("Executing %s %s\r\n", argv[0], argv[1]);
    int ret;

    ret = remove((char *)argv[1]);

    if (ret == 0)
    {
        PRINT("File %s was removed\r\n", argv[1]);
    }
    else
    {
        ERROR("File %s could not be found/removed\r\n", argv[1]);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

#if (defined(MCU_APP30) || defined(MCU_APP31))

/**
* @brief Function to print help for ls command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t ls_help(void *ref)
{
    (void)ref;

    PRINT("  ls\t= List files in the flash\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for ls command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t ls_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)ref;
    (void)argc;

    DIR *d;
    struct dirent *dir;

    INFO("Executing %s\r\n", argv[0]);

    d = opendir(".");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            /* Max File name size allowed in FLogFS is 40 characters */
            PRINT("%40s | %u B\r\n", dir->d_name, get_file_size(dir->d_name));
        }

        closedir(d);
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

#endif

/**
* @brief Function to print help for wrfile command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t wrfile_help(void *ref)
{
    (void)ref;

    PRINT("  wrfile <filename.ext> <length_in_bytes> \r\n");
    PRINT("    \t- Pass the <filename.ext> of the file to which data needs to be written \r\n");
    PRINT("    \t- Pass the <length_in_bytes> of the data that needs to be written to the file \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for wrfile command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t wrfile_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)ref;
    (void)argc;

    INFO("Executing %s to %s for %s bytes\r\n", argv[0], argv[1], argv[2]);

    /*! Clear the Write File Structure */
    memset(&wfile, 0, sizeof(write_file));

    /*! Get the Write Limit */
    wfile.data_len = atoi((char *)argv[2]);

    /*! Open the File which needs to be written */
    wfile.wfp = fopen((char *)argv[1], "wb");

    /*! If the file is available*/
    if (wfile.wfp)
    {
        PRINT("\r\nWaiting for %ld bytes of data\r\n", wfile.data_len);

        /*! Get the Write Operation initiate timestamp */
        wfile.last_write_ts = coines_get_millis();

        /*! Enable the Write operation */
        wfile.wrAck = true;

        /*! Execute the Write Operation */
        wrfile_loop_callabck((bool)false); /*If true- Echo On */
    }
    else
    {
        /*! If File not found */
        ERROR("File not found or could not be created\r\n");
    }

    PRINT("\r\n\r\n");

    return CLI_OK;
}

/**
* @brief Function to print help for rdfile command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t rdfile_help(void *ref)
{
    (void)ref;

    PRINT("  rdfile <filename.ext> \r\n");
    PRINT("    \t- Pass the <filename.ext> of the file from which data needs to be read \r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for rdfile command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t rdfile_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)ref;
    (void)argc;

    INFO("Executing %s to %s \r\n", argv[0], argv[1]);

    char rdString;

    /*! Open the File to be read */
    FILE *fp = fopen((char *)argv[1], "r");

    /*! If the file is available */
    if (fp)
    {
        PRINT("Read Initiated\r\n");

        /*! Read till the end of File */
        while (!feof(fp))
        {
            /*! Read a character */
            rdString = (char)fgetc(fp);

            /*! If next line/row, jump to next line/row in the console */
            if ((rdString == '\r') || (rdString == '\n'))
            {
                PRINT("\r\n");
            }
            else
            {
                /*! Else parse the read data to the console */
                if (!feof(fp))
                {
                    PRINT("%c", rdString);
                }
            }
        }

        /*! Close the File */
        fclose(fp);

        PRINT("\r\nRead Completed\r\n");
    }
    else
    {
        /*! If File not found */
        ERROR("File not found or could not be created\r\n");
    }

    return CLI_OK;
}

/**
* @brief Function to implement callback for cls command
* @param[in] argc : Number of arguments in command line
* @param[in] argv : Array of pointer to arguments
* @param[in] ref  : Reference to command line
*/
int8_t cls_callback(uint8_t argc, uint8_t * const argv[], void *ref)
{
    (void)ref;
    (void)argc;
    (void)argv;

    PRINT("\033c\033[2J");
#if (defined(MCU_APP30) || defined(MCU_APP31))
    fflush(bt_w);
#endif

    return CLI_OK;
}

/**
* @brief Function to print help for cls command
* @param[in] ref  : Reference to command line
* @return API error codes
*/
int8_t cls_help(void *ref)
{
    (void)ref;

    PRINT("  cls \r\n");
    PRINT("    \t= Clear screen\r\n");

    return CLI_OK;
}

/**
* @brief Function to implement callback for loop to write content to a file
* @param[in] echo_data : option to enable/disable echoing data
*/
void wrfile_loop_callabck(bool echo_data)
{
    uint32_t curr_ts, ts;
    uint16_t bytes_read_now = 0, len_check = 0;
    double progress = 0.0, new_progress = 0.0;
    int32_t file_size = wfile.data_len;

    /*! Indicate Command operation is Active */
    cmd_in_process = true;

    /*! If Write operation is enabled*/
    while (wfile.wrAck)
    {
        /*! Check the time elapsed from last Write. If time elapsed greater than Timeout, give Write Timeout error */
        curr_ts = coines_get_millis();
        if ((curr_ts - wfile.last_write_ts) > WRITE_TIMEOUT_MS)
        {
            PRINT("\r\nWrite Timed Out \r\n");

            /*! Disable the Write operation */
            wfile.wrAck = false;
            break;
        }

        /*! Check if new data is available. If yes */
        if (coines_intf_available(bhy2cli_intf))
        {
            /*! Reset the buffer index */
            bytes_read_now = 0;

            /*! Read the buffer */
            bytes_read_now = coines_read_intf(bhy2cli_intf, inp, coines_intf_available(bhy2cli_intf));

            /*! If the Write limbhys not reached */
            if (wfile.data_len >= 0)
            {
                /*! If data in buffer greater than limit, parse the data upto limit length */
                len_check = (uint16_t)((wfile.data_len > bytes_read_now) ? bytes_read_now : wfile.data_len);

                /*! Decrement the Write limit*/
                wfile.data_len -= bytes_read_now;

                /* Only for use with the terminal. Comment the following code snippet otherwise */
                if (echo_data)
                {
                    coines_write_intf(bhy2cli_intf, inp, len_check); /* Echo back the input */
                }

                progress = (double)((file_size - wfile.data_len) * 100.0f) / file_size;

                /*lint -e777*/
                if (fabs(progress) != fabs(new_progress))
                {
                    PRINT("File Transferred : %0.2f%%\r", progress);
                    new_progress = progress;
                }

                /*! Write the data to the File*/
                fwrite(inp, len_check, 1, wfile.wfp);

                /*! Update the last Write timestamp */
                wfile.last_write_ts = coines_get_millis();

                /*! If Write Limit crossed, give Write Length Exceeds status */
                if (wfile.data_len < 0)
                {
                    PRINT("\r\nWrite Length Exceeded \r\n");

                    /*! Disable the Write operation */
                    wfile.wrAck = false;
                }
            }
        }
        else
        {
            /*! If Write Limit reached and 1 second has elapse with no new data, give Write Completed status */
            if (wfile.data_len == 0)
            {
                /*! Get the timestamp */
                ts = coines_get_millis();

                /*! Check if 1 second has elapsed */
                if ((ts - wfile.last_write_ts) > 1000)
                {
                    PRINT("\r\nWrite Completed \r\n");

                    /*! Disable the Write operation */
                    wfile.wrAck = false;
                }
            }
        }

        /* Clear the buffer */
        memset(inp, 0, sizeof(inp));
    }

    /*! Close the File*/
    fclose(wfile.wfp);

    /*! Clear the Write File structure*/
    memset(&wfile, 0, sizeof(write_file));

    /*! Clear the buffer */
    memset(inp, 0, sizeof(inp));

    /*! Indicate Command operation is Inactive */
    cmd_in_process = false;

}

#if (defined(MCU_APP30) || defined(MCU_APP31))

/**
* @brief Function to get size of file
* @param[in] file_name : File name
* @return Size of file
*/
static size_t get_file_size(const char *file_name)
{
    struct stat st;

    (void)stat(file_name, &st);

    return (size_t)st.st_size;
}
#endif
#endif
