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
 * @file    dinject_defs.h
 * @brief   Definitions Header file for the data injection functions for the command line utility
 *
 */

#ifndef DINJECT_DEFS_H_
#define DINJECT_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

/*! Hex String Length */
/*! Corresponds to number of characters in xBit Hex data*/
#define _8BIT_HEX_LEN                     2
#define _16BIT_HEX_LEN                    4
#define _32BIT_HEX_LEN                    8

#define ACCEL_INJECT_ID                   INT8_C(55)
#define GYRO_INJECT_ID                    INT8_C(58)
#define MAG_INJECT_ID                     INT8_C(59)
#define PRESSURE_INJECT_ID                INT8_C(60)
#define GAS_INJECT_ID                     INT8_C(66)
#define IAQ_INJECT_ID                     INT8_C(68)

#define ACC_COR_INJECT_DRIVER_ID          INT8_C(76)
#define ACC_RAW_INJECT_DRIVER_ID          INT8_C(75)
#define ACC_PASSTH_INJECT_DRIVER_ID       INT8_C(74)
#define GYRO_COR_INJECT_DRIVER_ID         INT8_C(79)
#define GYRO_RAW_INJECT_DRIVER_ID         INT8_C(78)
#define GYRO_PASSTH_INJECT_DRIVER_ID      INT8_C(77)
#define GAME_ROTATION_INJECT_DRIVER_ID    INT8_C(80)
#define ORIENTATION_INJECT_DRIVER_ID      INT8_C(81)
#define LINEACC_INJECT_DRIVER_ID          INT8_C(82)

#define TIMESTAMP_SMALL_DELTA_NWU_ID      INT8_C(245)
#define TIMESTAMP_SMALL_DELTA_WU_ID       INT8_C(251)

#define TIMESTAMP_LARGE_DELTA_NWU_ID      INT8_C(246)
#define TIMESTAMP_LARGE_DELTA_WU_ID       INT8_C(252)

#define FULL_TIMESTAMP_WU_ID              INT8_C(247)
#define FULL_TIMESTAMP_NWU_ID             INT8_C(253)

#define META_EVENT_WU_ID                  INT8_C(248)
#define META_EVENT_NWU_ID                 INT8_C(254)

#define DEBUT_EVENT_ID                    INT8_C(250)

#define ACCEL_INJECT_EVENT_SIZE           INT8_C(7)
#define GYRO_INJECT_EVENT_SIZE            INT8_C(7)
#define MAG_INJECT_EVENT_SIZE             INT8_C(7)
#define PRESSURE_INJECT_EVENT_SIZE        INT8_C(5)
#define GAS_INJECT_EVENT_SIZE             INT8_C(5)
#define IAQ_INJECT_EVENT_SIZE             INT8_C(14)
#define TIMESTAMP_SMALL_DELTA_EVENT_SIZE  INT8_C(2)
#define TIMESTAMP_LARGE_DELTA_EVENT_SIZE  INT8_C(3)
#define FULL_TIMESTAMP_EVENT_SIZE         INT8_C(6)
#define META_EVENT_SIZE                   INT8_C(4)
#define DEBUG_EVENT_SIZE                  INT8_C(18)

#define QUAT_INJECT_EVENT_SIZE            INT8_C(9)
#define EULER_INJECT_EVENT_SIZE           INT8_C(7)

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* DINJECT_DEFS_H_ */
