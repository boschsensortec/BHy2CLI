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
 * @file    parse.c
 * @brief   Source file for the parse functions for the command line utility
 *
 */

#include <stdio.h>
#include "parse.h"
#include "verbose.h"
#include "coines.h"

static uint16_t count[256] = { 0 };
extern bool enable_ds[256];
extern uint16_t odr_ds[256];

#if 0

/**
* @brief Function to add accuracy to sensor data
* @param[in] data_ptr  : Pointer to input data
* @param[out] data_buf : Pointer to output buffer
* @param[in] data_size : Data size
* @param[in] accuracy  : Accuracy to add
*/
void add_accuracy_to_sensor_data(const uint8_t *data_ptr, uint8_t *data_buf, uint8_t data_size, uint8_t accuracy)
{
    if (!data_buf || !data_ptr)
    {
        ERROR("Null reference\r\n");

        return;
    }

    memset(data_buf, 0, sizeof(data_size));
    memcpy(data_buf, data_ptr, (data_size - 1));
    data_buf[data_size - 1] = accuracy;
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

/**
* @brief Function to parse sensor status meta event
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] byte1         : Byte 1 in meta event
* @param[in] byte2         : Byte 2 in meta event
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_sensor_status(char *event_text,
                                           uint32_t s,
                                           uint32_t ns,
                                           uint8_t byte1,
                                           uint8_t byte2,
                                           struct parse_ref *parse_table)
{
    struct parse_sensor_details *sensor_details;

    DATA("%s; T: %lu.%09lu; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
    sensor_details = parse_get_sensor_details(byte1, parse_table);

    /*lint -e774 */
    if (parse_table && sensor_details)
    {
        sensor_details->accuracy = byte2;
    }
    else
    {
        INFO("Parse slot not defined for %u\r\n", byte1);
    }
}

/**
* @brief Function to parse meta event
* @param[in] callback_info : Pointer to callback information
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_type(const struct bhy2_fifo_parse_data_info *callback_info,
                                  char *event_text,
                                  uint32_t s,
                                  uint32_t ns,
                                  struct parse_ref *parse_table)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            DATA("%s; T: %lu.%09lu; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            DATA("%s; T: %lu.%09lu; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            DATA("%s; T: %lu.%09lu; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            DATA("%s; T: %lu.%09lu; Algorithm event\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            parse_meta_event_sensor_status(event_text, s, ns, byte1, byte2, parse_table);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            DATA("%s; T: %lu.%09lu; BSX event (do steps main)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            DATA("%s; T: %lu.%09lu; BSX event (do steps calib)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            DATA("%s; T: %lu.%09lu; BSX event (get output signal)\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            DATA("%s; T: %lu.%09lu; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            DATA("%s; T: %lu.%09lu; FIFO overflow\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            DATA("%s; T: %lu.%09lu; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            DATA("%s; T: %lu.%09lu; FIFO watermark reached\r\n", event_text, s, ns);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            DATA("%s; T: %lu.%09lu; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
                 ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            DATA("%s; T: %lu.%09lu; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            DATA("%s; T: %lu.%09lu; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            DATA("%s; T: %lu.%09lu; Reset event. Cause : %u\r\n", event_text, s, ns, byte2);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            DATA("%s; T: %lu.%09lu; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
            break;
    }
}

/**
* @brief Function to log data
* @param[in] sid           : Sensor ID
* @param[in] tns           : Time in nanoseconds
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
#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

        logbin_add_data(sid, tns, event_size, event_payload, logdev);

#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif
    }
}

/**
* @brief Function to stream hex data
* @param[in] sid           : Sensor ID
* @param[in] ts            : Time in seconds
* @param[in] tns           : Time in nanoseconds
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
* @brief Function to print activity in string
* @param[in] activity : Activity value
*/
static void print_activity(uint16_t activity)
{
    if (activity & BHY2_STILL_ACTIVITY_ENDED)
    {
        PRINT_D(" Still activity ended,");
    }

    if (activity & BHY2_WALKING_ACTIVITY_ENDED)
    {
        PRINT_D(" Walking activity ended,");
    }

    if (activity & BHY2_RUNNING_ACTIVITY_ENDED)
    {
        PRINT_D(" Running activity ended,");
    }

    if (activity & BHY2_ON_BICYCLE_ACTIVITY_ENDED)
    {
        PRINT_D(" On bicycle activity ended,");
    }

    if (activity & BHY2_IN_VEHICLE_ACTIVITY_ENDED)
    {
        PRINT_D(" In vehicle ended,");
    }

    if (activity & BHY2_TILTING_ACTIVITY_ENDED)
    {
        PRINT_D(" Tilting activity ended,");
    }

    if (activity & BHY2_STILL_ACTIVITY_STARTED)
    {
        PRINT_D(" Still activity started,");
    }

    if (activity & BHY2_WALKING_ACTIVITY_STARTED)
    {
        PRINT_D(" Walking activity started,");
    }

    if (activity & BHY2_RUNNING_ACTIVITY_STARTED)
    {
        PRINT_D(" Running activity started,");
    }

    if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED)
    {
        PRINT_D(" On bicycle activity started,");
    }

    if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED)
    {
        PRINT_D(" In vehicle activity started,");
    }

    if (activity & BHY2_TILTING_ACTIVITY_STARTED)
    {
        PRINT_D(" Tilting activity started,");
    }
}

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct parse_sensor_details *parse_get_sensor_details(uint8_t id, struct parse_ref *ref)
{
    uint8_t i;

    for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
    {
        if (ref->sensor[i].id == id)
        {
            return &ref->sensor[i];
        }
    }

    return NULL;
}

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct parse_sensor_details *parse_add_sensor_details(uint8_t id, struct parse_ref *ref)
{
    uint8_t i = 0;

    struct parse_sensor_details *sensor_details;

    sensor_details = parse_get_sensor_details(id, ref);
    if (sensor_details)
    {

        /* Slot for the sensor ID is already used */
        return sensor_details;
    }
    else
    {
        /* Find a new slot */
        for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
        {
            if (ref->sensor[i].id == 0)
            {
                INFO("Using slot %u for SID %u\r\n", i, id);
                ref->sensor[i].id = id;

                return &ref->sensor[i];
            }
        }
    }

    return NULL;
}

/**
* @brief Function to check stream log flags
* @param[in] callback_info  : Pointer to callback information
* @param[in] parse_flag     : Stream log flags
*/
static void check_stream_log_flags(const struct bhy2_fifo_parse_data_info *callback_info, uint8_t parse_flag)
{
    if ((parse_flag & PARSE_FLAG_STREAM) && (count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
    {
        if (count[callback_info->sensor_id] == odr_ds[callback_info->sensor_id])
        {
            count[callback_info->sensor_id] = 0;
        }
    }
}

/**
* @brief Function to print log for 3-axis format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info,
                                struct bhy_event_data_xyz data,
                                float scaling_factor,
                                uint32_t s,
                                uint32_t ns,
                                const struct parse_sensor_details *sensor_details)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; acc: %u\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x * scaling_factor,
         data.y * scaling_factor,
         data.z * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for 3-axis format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_3axis_s16(bool flag,
                                     const struct bhy2_fifo_parse_data_info *callback_info,
                                     struct bhy_event_data_xyz data,
                                     uint32_t s,
                                     uint32_t ns,
                                     uint64_t tns,
                                     struct parse_ref *parse_table,
                                     uint8_t parse_flag,
                                     const struct parse_sensor_details *sensor_details,
                                     float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
        }
    }

    if (parse_flag & PARSE_FLAG_HEXSTREAM)
    {
        stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
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
* @brief Function to print log for euler format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_euler(const struct bhy2_fifo_parse_data_info *callback_info,
                            struct bhy_event_data_orientation data,
                            float scaling_factor,
                            uint32_t s,
                            uint32_t ns,
                            const struct parse_sensor_details *sensor_details)
{
    DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.heading * scaling_factor,
         data.pitch * scaling_factor,
         data.roll * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for euler format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_euler(bool flag,
                                 const struct bhy2_fifo_parse_data_info *callback_info,
                                 struct bhy_event_data_orientation data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct parse_ref *parse_table,
                                 uint8_t parse_flag,
                                 const struct parse_sensor_details *sensor_details,
                                 float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
        }
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
* @brief Function to print log for quaternion format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_quaternion(const struct bhy2_fifo_parse_data_info *callback_info,
                                 struct bhy_event_data_quaternion data,
                                 uint32_t s,
                                 uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f,
         ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

/**
* @brief Function to stream and log for quaternion format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_quaternion(bool flag,
                                      const struct bhy2_fifo_parse_data_info *callback_info,
                                      struct bhy_event_data_quaternion data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_quaternion(callback_info, data, s, ns);
            }
        }
        else
        {
            print_log_quaternion(callback_info, data, s, ns);
        }
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
* @brief Function to print log for 16-bit signed format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_s16_as_float(const struct bhy2_fifo_parse_data_info *callback_info,
                                   int16_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
}

/**
* @brief Function to stream and log for 16-bit signed format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_s16_as_float(bool flag,
                                        const struct bhy2_fifo_parse_data_info *callback_info,
                                        int16_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
        }
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
* @brief Function to print log for 32-bit scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u32(const struct bhy2_fifo_parse_data_info *callback_info,
                                 uint32_t data,
                                 uint32_t s,
                                 uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %lu\r\n", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 32-bit scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_u32(bool flag,
                                      const struct bhy2_fifo_parse_data_info *callback_info,
                                      uint32_t data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u32(callback_info, data, s, ns);
            }
        }
        else
        {
            print_log_scalar_u32(callback_info, data, s, ns);
        }
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
* @brief Function to print log for scalar event format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_event(const struct bhy2_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu;\r\n", callback_info->sensor_id, s, ns);
}

/**
* @brief Function to stream and log for scalar event format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_event(bool flag,
                                        const struct bhy2_fifo_parse_data_info *callback_info,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct parse_ref *parse_table,
                                        uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_event(callback_info, s, ns);
            }
            else
            {
                print_log_scalar_event(callback_info, s, ns);
            }
        }
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
* @brief Function to print log for activity format
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_activity(const struct bhy2_fifo_parse_data_info *callback_info,
                               uint16_t activity,
                               uint32_t s,
                               uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; ", callback_info->sensor_id, s, ns);

    print_activity(activity);

    PRINT_D("\r\n");
}

/**
* @brief Function to stream and log for activity format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_activity(bool flag,
                                    const struct bhy2_fifo_parse_data_info *callback_info,
                                    uint16_t activity,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_activity(callback_info, activity, s, ns);
            }
        }
        else
        {
            print_log_activity(callback_info, activity, s, ns);
        }
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
* @brief Function to print log for 16-bit unsigned format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_u16_as_float(const struct bhy2_fifo_parse_data_info *callback_info,
                                   uint16_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
}

/**
* @brief Function to stream and log for 16-bit unsigned format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_u16_as_float(bool flag,
                                        const struct bhy2_fifo_parse_data_info *callback_info,
                                        uint16_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_u16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            print_log_u16_as_float(callback_info, data, scaling_factor, s, ns);
        }
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
* @brief Function to print log for 24-bit unsigned format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_u24_as_float(const struct bhy2_fifo_parse_data_info *callback_info,
                                   uint32_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
}

/**
* @brief Function to stream and log for 24-bit unsigned format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_u24_as_float(bool flag,
                                        const struct bhy2_fifo_parse_data_info *callback_info,
                                        uint32_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
        }
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
* @brief Function to print log for proximity format
* @param[in] callback_info  : Pointer to callback information
* @param[in] text           : Text to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_proxmity(const struct bhy2_fifo_parse_data_info *callback_info,
                               char *text,
                               uint32_t s,
                               uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %s\r\n", callback_info->sensor_id, s, ns, text);
}

/**
* @brief Function to stream and log for 24-bit unsigned format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] text           : Text to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_proxmity(bool flag,
                                    const struct bhy2_fifo_parse_data_info *callback_info,
                                    char *text,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_proxmity(callback_info, text, s, ns);
            }
        }
        else
        {
            print_log_proxmity(callback_info, text, s, ns);
        }
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
* @brief Function to print log for 8-bit unsigned scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u8(const struct bhy2_fifo_parse_data_info *callback_info,
                                uint8_t data,
                                uint32_t s,
                                uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %u\r\n", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 8-bit unsigned scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_scalar_u8(bool flag,
                                 const struct bhy2_fifo_parse_data_info *callback_info,
                                 uint8_t data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct parse_ref *parse_table,
                                 uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u8(callback_info, data, s, ns);
            }
        }
        else
        {
            print_log_scalar_u8(callback_info, data, s, ns);
        }
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
* @brief Function to print log for generic format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_generic(const struct bhy2_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; D: ", callback_info->sensor_id, s, ns);

    for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
    {
        PRINT_D("%02X", callback_info->data_ptr[i]);
    }

    PRINT_D("\r\n");
}

/**
* @brief Function to stream and log for generic format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_generic(bool flag,
                                   const struct bhy2_fifo_parse_data_info *callback_info,
                                   uint32_t s,
                                   uint32_t ns,
                                   uint64_t tns,
                                   struct parse_ref *parse_table,
                                   uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_generic(callback_info, s, ns);
            }
        }
        else
        {
            print_log_generic(callback_info, s, ns);
        }
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
* @brief Function to print log for device orientation format
* @param[in] callback_info  : Pointer to callback information
* @param[in] ori            : Pointer to device orientation
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_device_ori(const struct bhy2_fifo_parse_data_info *callback_info,
                                 char *ori,
                                 uint32_t s,
                                 uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %s\r\n", callback_info->sensor_id, s, ns, ori);
}

/**
* @brief Function to stream and log for device orientation format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] ori            : Pointer to device orientation
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_device_ori(bool flag,
                                      const struct bhy2_fifo_parse_data_info *callback_info,
                                      char *ori,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_device_ori(callback_info, ori, s, ns);
            }
        }
        else
        {
            print_log_device_ori(callback_info, ori, s, ns);
        }
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
* @brief Function to print log for GPS format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_gps(const struct bhy2_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    DATA("[GPS]; T: %lu.%09lu; data: %s\r\n", s, ns, callback_info->data_ptr);
}

/**
* @brief Function to stream and log for GPS format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_gps(bool flag,
                               const struct bhy2_fifo_parse_data_info *callback_info,
                               uint32_t s,
                               uint32_t ns,
                               uint64_t tns,
                               struct parse_ref *parse_table,
                               uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_gps(callback_info, s, ns);
            }
        }
        else
        {
            print_log_gps(callback_info, s, ns);
        }
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
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint32_t s, ns;
    uint64_t tns;
    char *event_text;

    if (!callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    parse_meta_event_type(callback_info, event_text, s, ns, parse_table);
}

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhy_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse euler format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy_event_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhy_event_data_parse_orientation(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhy_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    bhy_event_data_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 16-bit signed format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_s16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not define for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_scalar_u32(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    data = BHY2_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_scalar_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    activity = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

#if 0

/**
* @brief Function to parse 16-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_u16_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    float scaling_factor;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id])
    {
        flag = true;
        stream_and_log_u16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_u16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}
#endif

/**
* @brief Function to parse 24-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_u24_as_float(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHY2_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

#if 0

/**
* @brief Function to parse proximity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_proximity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *text;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    text = callback_info->data_ptr[0] ? "near" : "far";

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_proxmity(flag, callback_info, text, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_proxmity(flag, callback_info, text, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

#endif

/**
* @brief Function to parse 8-bit unsigned scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_scalar_u8(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    data = callback_info->data_ptr[0];
    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_generic(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse device orientation format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_device_ori(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *ori;
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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

    switch (callback_info->data_ptr[0])
    {
        case 0:
            ori = "Portrait upright";
            break;
        case 1:
            ori = "Landscape left";
            break;
        case 2:
            ori = "Portrait upside down";
            break;
        case 3:
            ori = "Landscape right";
            break;
        default:
            ori = "Unknown orientation";
            break;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_device_ori(flag, callback_info, ori, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_device_ori(flag, callback_info, ori, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

#if 0

/**
* @brief Function to parse GPS format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_gps(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct parse_ref *parse_table = (struct parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct parse_sensor_details *sensor_details;
    bool flag;

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
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    callback_info->data_ptr[callback_info->data_size - 2] = '\0';

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_gps(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_gps(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}
#endif

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;

    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */

    if (!callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */

    DATA("[DEBUG MSG]; T: %lu.%09lu; %s\r\n", s, ns, debug_msg);
}
