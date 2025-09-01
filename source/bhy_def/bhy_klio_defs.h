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
* @file       bhy_klio_defs.h
* @date       2025-08-20
* @version    v1.0.0
*
*/

#ifndef __BHY_KLIO_PARAM_DEFS_H__
#define __BHY_KLIO_PARAM_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>

#define BHY_SENSOR_ID_KLIO      UINT8_C(112)
#define BHY_SENSOR_ID_KLIO_LOG  UINT8_C(127)

#ifndef BHY_KLIO_PAGE
#define BHY_KLIO_PAGE           UINT16_C(9)
#endif
#define BHY_KLIO_PARAM(id)      (((BHY_KLIO_PAGE) << 8) | (id))

#define MAX_DATA_SIZE           244

/*!
 * @enum bhy_klio_param
 *
 * KLIO algorithm parameters
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_ALGORITHM_VERSION
 * Algorithm version. (read)
 * @code{.c} char version[244]; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_RECOGNITION_RESPONSIVNESS
 * The approximate number of cycles / repetitions for recognition to recognize
 * an activity. (read / write)
 * @code{.c} float cycle_fraction; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_PATTERN_BLOB_SIZE
 * Pattern blob size in bytes. (read)
 * @code{.c} uint16_t pattern_blob_size; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_RECOGNITION_MAX_PATTERNS
 * Read Maximum number of patterns. (read)
 * @code{.c} uint16_t recognition_max_patterns; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT
 * Flag if insignificant movements should be ignored in learning. Value 0 or 1. (read / write)
 * @code{.c} uint8_t ignore_insig_movement; @endcode
 */
typedef enum
{
    BHY_KLIO_PARAM_ALGORITHM_VERSION = 0,
    BHY_KLIO_PARAM_RECOGNITION_SENSITIVITY = 1,
    BHY_KLIO_PARAM_RECOGNITION_RESPONSIVNESS = 2,
    BHY_KLIO_PARAM_LEARNING_SPEED = 3,
    BHY_KLIO_PARAM_LEARNING_SIMILARITY_THRESHOLD = 4,
    BHY_KLIO_PARAM_LEARNING_FREQUENCIES = 5,
    BHY_KLIO_PARAM_LEARNING_PROGRESS = 6,
    BHY_KLIO_PARAM_PATTERN_BLOB_SIZE = 7,
    BHY_KLIO_PARAM_RECOGNITION_MAX_PATTERNS = 8,
    BHY_KLIO_PARAM_INPUT_SENSORS = 9,
    BHY_KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT = 10
} bhy_klio_param_t;

/*!
 * @enum bhy_klio_param_pattern_parameter_t
 *
 * KLIO parameters that can be set for each pattern
 *
 * @var bhy_klio_param_pattern_parameter_t::KLIO_PATTERN_PARAM_EXPONENT_SCALING_FACTOR
 * Exponent scaling factor. (read/write)
 * A value smaller than 1.0 will make the pattern less likely to be recognized.
 * A value larger than 1.0 will make the pattern more likely to be recognized.
 * @code{.c} float exponent_scaling_factor @endcode
 */
typedef enum
{
    BHY_KLIO_PATTERN_PARAM_EXPONENT_SCALING_FACTOR = 0,
} bhy_klio_param_pattern_parameter_t;

typedef struct
{
    uint8_t max_patterns;
    uint8_t max_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
} bhy_klio_info;

/*!
 *
 * @brief Convenience wrapper used in bhy_klio_param.c algo parameter wrapper.
 *
 */
typedef struct
{
    uint8_t id;
    uint8_t flags;
    uint8_t size;
    union
    {
        uint8_t data[MAX_DATA_SIZE];
        char version[MAX_DATA_SIZE];
        float cycle_fraction;
        uint16_t pattern_blob_size;
        uint16_t recognition_max_patterns;
        uint8_t ignore_insig_movement;
    } BHY_PACKED payload;
} BHY_PACKED bhy_klio_param_wrapper_t;

/*!
* @brief Header for pattern parameter requests.
*/
typedef struct
{
    uint8_t pattern_id;
    uint8_t parameter_id;
    uint8_t size;
    uint8_t padding[1]; /* Padding to make the struct a multiple of 4 bytes for convenience. */
} BHY_PACKED bhy_klio_param_pattern_parameter_wrapper_header_t;

/*!
 *
 * @brief Struct used for writing, requesting read, and reading a pattern parameter.
 *
 * When setting a parameter. Size should be set to the size of the payload.
 * And the new value should be in the payload field.
 *
 * To request a parameter read,  the pattern and parameter that should be read can be specified,
 * together with a size of zero, indicating that no parameter should be set.
 * When reading the parameter, the size field will be set to the actual size of the payload.
 *
 * The struct should be padded to a multiple of 4 bytes.
 */
typedef struct
{
    bhy_klio_param_pattern_parameter_wrapper_header_t header;
    union
    {
        uint8_t data[sizeof(float)];
        float exponent_scaling_factor;
    } BHY_PACKED payload;
} BHY_PACKED bhy_klio_param_pattern_parameter_wrapper_t;

/*!
 *
 * @brief bhy klio state structure
 *
 */
typedef struct
{
    uint8_t learning_enabled; /*!< 0 - disable learning, 1 - enable learning. */
    uint8_t learning_reset; /*!< 0 - nop, 1 - reset learning. Always read as 0. */
    uint8_t recognition_enabled; /*!< 0 - disable recognition, 1 - enable recognition. */
    uint8_t recognition_reset; /*!< 0 - nop, 1 - reset learning. Always read as 0. */
} BHY_PACKED bhy_klio_param_sensor_state_t;

/*!
 *
 * @brief bhy klio sensor log frame
 *
 * When the algorithm consumes a new set of samples, it is sent in this structure.
 *
 */
typedef struct
{
    float timestamp;
    float accel[3];
    float gyro[3];
} BHY_PACKED bhy_klio_param_log_frame_t;

/*!
 *
 * @brief bhy klio pattern transfer format
 *
 */
typedef struct
{
    uint8_t block_id; /*!< Write as 0. */
    uint8_t block_size; /*!< Write as 0. */

    /*! Size of pattern_data in bytes. The size of patterns may be obtained
    using the @ref bhy_klio_param::BHY_KLIO_PARAM_PATTERN_BLOB_SIZE
    parameter. */
    uint16_t full_size;

    /*! Id of pattern to write. */
    uint8_t pattern_id;

    /*! Actual pattern data describing the exercise. */
    uint8_t pattern_data[MAX_DATA_SIZE];
} BHY_PACKED bhy_klio_param_pattern_transfer_t;

/*!
 *
 * @brief Convenience wrapper used by @ref bhy_klio_param_similarity_score_multiple().
 *
 */
typedef struct
{
    uint32_t zero;
    uint8_t index;
    uint8_t count;
    uint8_t indexes;
} BHY_PACKED bhy_klio_param_similarity_calculation_t;

/*!
 *
 * @brief Convenience wrapper used by @ref bhy_klio_param_set_pattern_states().
 *
 */
typedef struct
{
    uint8_t operation;
    uint8_t count;
    uint8_t indexes;
} BHY_PACKED bhy_klio_param_pattern_state_op_t;

/*!
 * KLIO pattern state
 */
typedef enum
{
    /*! disable pattern */
    BHY_KLIO_PATTERN_STATE_DISABLE = 0,
    /*! enable pattern */
    BHY_KLIO_PATTERN_STATE_ENABLE = 1,
    /*! switch hand */
    BHY_KLIO_PATTERN_STATE_SWITCH_HAND = 2,
    /*! disable adaptive pattern */
    BHY_KLIO_PATTERN_STATE_AP_DISABLE = 3
} bhy_klio_param_pattern_state_t;

/*!
 * @brief KLIO host interface parameter identifiers.
 *
 * The parameter id used when calling the BHY functions directly. E.g. to read the driver status this may be used.
 *
 * ~~~{.c}
 * uint32_t klio_driver_status = 0;
 * uint32_t ret_len = 0;
 * int8_t rslt = bhy_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_DRIVER_STATUS),
 *                                  (uint8_t*)&klio_driver_status,
 *                                  sizeof(klio_driver_status),
 *                                  &ret_len,
 *                                  dev);
 * if (rslt != BHY_OK)
 * {
 *   return rslt;
 * }
 * ~~~
 */
typedef enum
{
    /*! Read and write algorithm state. */
    BHY_KLIO_HIF_PARAM_ALGORITHM_STATE = 0,
    /*! Read and write patterns. */
    BHY_KLIO_HIF_PARAM_PATTERN = 1,
    /*! Read and write algorithm parameters. */
    BHY_KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER = 2,
    BHY_KLIO_HIF_PARAM_STATISTIC = 3,
    BHY_KLIO_HIF_PARAM_PATTERN_STATE = 4,
    /*! Perform pattern similarity. */
    BHY_KLIO_HIF_PARAM_PATTERN_SIMILARITY = 5,
    BHY_KLIO_HIF_PARAM_PATTERN_COMBINING = 6,
    /*! Read driver status. */
    BHY_KLIO_HIF_PARAM_DRIVER_STATUS = 7,
    /*! Reset driver to its startup state. */
    BHY_KLIO_HIF_PARAM_RESET = 8,
    /*! Read and write pattern parameters. */
    BHY_KLIO_HIF_PARAM_PATTERN_PARAM = 9
} bhy_klio_param_hif_parameter_t;

/*!
 * KLIO driver error codes
 */
typedef enum
{
    /*! No error. */
    BHY_KLIO_DRIVER_ERROR_NONE = 0,
    /*! Invalid parameter. */
    BHY_KLIO_DRIVER_ERROR_INVALID_PARAMETER = 1,
    /*! Parameter out of range. */
    BHY_KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE = 2,
    /*!
     * Invalid pattern operation. E.g. trying to read a learnt pattern when no
     * pattern has been learnt, or trying to write an invalid pattern.
     */
    BHY_KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION = 3,
    /*! Not implemented. */
    BHY_KLIO_DRIVER_ERROR_NOT_IMPLEMENTED = 4,
    /*! Insufficient buffer size */
    BHY_KLIO_DRIVER_ERROR_BUFSIZE = 5,
    /*! Internal error. */
    BHY_KLIO_DRIVER_ERROR_INTERNAL = 6,
    /*! Undefined error */
    BHY_KLIO_DRIVER_ERROR_UNDEFINED = 7,
    /*!
     * Previous operation is still progressing. Read driver status again until
     * operation is finished.
     */
    BHY_KLIO_DRIVER_ERROR_OPERATION_PENDING = 8
} bhy_klio_param_driver_error_state_t;

typedef int8_t (*bhy_klio_param_read_reset_driver_status_func)(uint32_t *driver_status, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_read_pattern_func)(const uint8_t id, uint8_t *buffer, uint16_t *length,
                                                   struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_set_state_func)(const bhy_klio_param_sensor_state_t *state, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_get_state_func)(bhy_klio_param_sensor_state_t *state, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_write_pattern_func)(const uint8_t idx, const uint8_t *pattern_data, const uint16_t size,
                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_set_pattern_states_func)(const bhy_klio_param_pattern_state_t operation,
                                                         const uint8_t *pattern_ids, const uint16_t arr_size,
                                                         struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_similarity_score_func)(const uint8_t *first_pattern, const uint8_t *second_pattern,
                                                       const uint16_t size, float *similarity, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_similarity_score_multiple_func)(const uint8_t idx, const uint8_t *indexes,
                                                                const uint8_t count, float *similarity,
                                                                struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_set_parameter_func)(const bhy_klio_param_t id, const void *parameter_data,
                                                    const uint16_t size, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_get_parameter_func)(const bhy_klio_param_t id, uint8_t *parameter_data, uint16_t *size,
                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_set_pattern_parameter_func)(const uint8_t pattern_id,
                                                            const bhy_klio_param_pattern_parameter_t parameter_id,
                                                            const void *parameter_data, const uint16_t size,
                                                            struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_get_pattern_parameter_func)(const uint8_t pattern_id,
                                                            const bhy_klio_param_pattern_parameter_t parameter_id,
                                                            uint8_t *result_buffer, const uint16_t result_buffer_size,
                                                            uint16_t *bytes_written, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_reset_func)(struct bhy_dev *dev);

typedef bhy_klio_info* (*bhy_get_klio_info_func)(void);

typedef void (*bhy_set_klio_info_func)(const bhy_klio_info* info);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_KLIO_PARAM_DEFS_H__ */
