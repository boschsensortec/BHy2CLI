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
* @date       2025-03-28
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

#define BHY_SENSOR_ID_KLIO          UINT8_C(112)
#define BHY_SENSOR_ID_KLIO_GENERIC  UINT8_C(117)
#define BHY_SENSOR_ID_KLIO_LOG      UINT8_C(127)

#ifndef BHY_KLIO_PAGE
#define BHY_KLIO_PAGE               UINT16_C(9)
#endif
#define BHY_KLIO_PARAM(id)          (((BHY_KLIO_PAGE) << 8) | (id))

#define MAX_DATA_SIZE               244

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
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_MAX_NUM_PATTERNS_GENERIC
 * Maximum number of patterns for generic gesture recognition. (read)
 * @code{.c} uint16_t max_num_patterns_generic; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_MAX_BLOB_SIZE_GENERIC
 * Maximum size of one generic gesture pattern. (read)
 * @code{.c} uint16_t max_blob_size_generic; @endcode
 *
 * @var bhy_klio_param::BHY_KLIO_PARAM_RECOGNITION_THRESHOLD_GENERIC
 * Recognition threshold for the generic gesture algorithm, between 0.0 and 1.0.
 * A lower number means the algorithm will recognize more easily, but can also lead to false positives.
 * If the generic gesture algorithm is disabled, this will be read as -999.0. (read / write)
 * @code{.c} float recognition_threshold_generic; @endcode
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
    BHY_KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT = 10,
    /* Start of parameters for the generic gesture algorithm */
    BHY_KLIO_PARAM_MAX_NUM_PATTERNS_GENERIC = 128,
    BHY_KLIO_PARAM_MAX_BLOB_SIZE_GENERIC = 129,
    BHY_KLIO_PARAM_RECOGNITION_THRESHOLD_GENERIC = 130
} bhy_klio_param_t;

typedef struct
{
    uint8_t max_cyclic_patterns;
    uint8_t max_cyclic_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
    uint8_t max_generic_patterns;
    uint16_t max_generic_pattern_blob_size;
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
        uint16_t max_num_patterns_generic;
        uint16_t max_blob_size_generic;
        float recognition_threshold_generic;
    } BHY_PACKED payload;
} BHY_PACKED bhy_klio_param_wrapper_t;

/*!
 *
 * @brief bhy cyclic klio state structure
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
 * @brief bhy generic klio state structure
 *
 */
typedef struct
{
    uint8_t gestures_enabled; /*!< 0 - gesture detection disabled, 1 - gesture detection enabled */
    uint8_t timing_enabled; /*!< 0 - timing detection disabled, 1 - timing detection enabled (read-only) */
    uint8_t dummy[2]; /*!< Padding bytes to make the parameter size a multiple of 4 bytes, since the BHI requires this.
                       * */
} BHY_PACKED bhy_klio_param_generic_sensor_state_t;

/*!
 *
 * @brief bhy klio generic data structure
 *
 * When the algorithm generates a new data frame, it is sent in this structure.
 */
typedef struct
{
    /*! The ID of the recognized subgesture. */
    uint8_t subgest_id;

    /*! Number of repetitions of this subgesture. */
    uint8_t subgest_count;

    /*! The score of the recognized subgesture. */
    float subgest_score;

    /*! The repetition count of the full gesture. Only valid if timing recognition is enabled. */
    uint8_t full_gest_count;

    /*! The score of the full gesture. Only valid if timing recognition is enabled. */
    float full_gest_score;
} BHY_PACKED bhy_klio_param_generic_sensor_frame_t;

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
 * @brief bhy klio cyclic pattern transfer format
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
 * @brief bhy klio generic pattern transfer format
 *
 */
typedef struct
{
    /*! Start offset to write this chunk of pattern data to. */
    uint16_t offset;

    /*! Size of full pattern data in bytes. The maximum size of patterns may be obtained
    using the @ref bhy_klio_param::BHY_KLIO_PARAM_MAX_PATTERN_SIZE_GENERIC
    parameter. */
    uint16_t full_size;

    /*! Size of this block of pattern data in bytes. */
    uint8_t block_size;

    /*! Id of pattern to write. */
    uint8_t pattern_id;

    /*! Actual pattern data describing the exercise. */
    uint8_t pattern_data[MAX_DATA_SIZE];
} BHY_PACKED bhy_klio_param_generic_pattern_transfer_t;

/*!
 *
 * @brief bhy klio config transfer format
 *
 */
typedef struct
{
    /*! Size of config data in bytes. */
    uint16_t size;

    /*! Index to load config to. Only used for gesture config. */
    uint8_t id;

    /*! Actual config data. */
    uint8_t config_data[MAX_DATA_SIZE];
} BHY_PACKED bhy_klio_param_config_transfer_t;

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
    /*! Read and write cyclic algorithm state. */
    BHY_KLIO_HIF_PARAM_ALGORITHM_STATE = 0,
    /*! Read and write cyclic patterns. */
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
    /*! Reset driver, including both Klio and non-rep algorithm states. */
    BHY_KLIO_HIF_PARAM_RESET = 8,
    /* Start of parameters for the generic algorithm. */
    /*! Read and write generic algorithm state. */
    BHY_KLIO_HIF_PARAM_ALGORITHM_STATE_GENERIC = 128,
    /*! Write generic patterns. */
    BHY_KLIO_HIF_PARAM_PATTERN_GENERIC = 129,
    /*! Write generic algorithm config. */
    BHY_KLIO_HIF_PARAM_CONFIG_GENERIC = 130
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

typedef int8_t (*bhy_klio_param_set_generic_recognition_state_func)(const bhy_klio_param_generic_sensor_state_t *state,
                                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_get_state_func)(bhy_klio_param_sensor_state_t *state, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_get_generic_recognition_state_func)(bhy_klio_param_generic_sensor_state_t *state,
                                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_write_pattern_func)(const uint8_t idx, const uint8_t *pattern_data, const uint16_t size,
                                                    struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_write_generic_pattern_func)(const uint8_t idx, const uint8_t *pattern_data,
                                                            const uint16_t size, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_write_gesture_config_func)(const uint8_t idx, const uint8_t *config_data,
                                                           const uint16_t size, struct bhy_dev *dev);

typedef int8_t (*bhy_klio_param_write_timing_config_func)(const uint8_t *config_data, const uint16_t size,
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

typedef int8_t (*bhy_klio_param_reset_func)(struct bhy_dev *dev);

typedef bhy_klio_info* (*bhy_get_klio_info_func)(void);

typedef void (*bhy_set_klio_info_func)(const bhy_klio_info* info);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY_KLIO_PARAM_DEFS_H__ */
