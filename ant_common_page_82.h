/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef ANT_COMMON_PAGE_82_H__
#define ANT_COMMON_PAGE_82_H__

/** @file
 *
 * @defgroup ant_common_page_82 ANT+ common page 82
 * @{
 * @ingroup ant_sdk_common_pages
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANT_COMMON_PAGE_82 (82) ///< @brief ID value of common page 82.

/**@brief Data structure for ANT+ common data page 82.
 *
 * @note This structure implements only page 82 specific data.
 */

typedef enum {
    BATTERY_NEW = 1,
    BATTERY_GOOD = 2,
    BATTERY_OK = 3,
    BATTERY_LOW = 4,
    BATTERY_CRITICAL = 5,
} battery_status_t;

typedef struct
{
    battery_status_t battery_status;
} ant_common_page82_data_t;

/**@brief Initialize page 82.
 */
#define DEFAULT_ANT_COMMON_page82()                                     \
    (ant_common_page82_data_t)                                          \
    {                                                                   \
        .battery_status = BATTERY_NEW \
    }

/**@brief Function for encoding page 82.
 *
 * @param[in]  p_page_data      Pointer to the page data.
 * @param[out] p_page_buffer    Pointer to the data buffer.
 */
void ant_common_page_82_encode(uint8_t * p_page_buffer,
                               volatile ant_common_page82_data_t const * p_page_data);

/**@brief Function for decoding page 82.
 *
 * @param[in]  p_page_buffer    Pointer to the data buffer.
 * @param[out] p_page_data      Pointer to the page data.
 */
void ant_common_page_82_decode(uint8_t const * p_page_buffer,
                               volatile ant_common_page82_data_t * p_page_data);


#ifdef __cplusplus
}
#endif

#endif // ANT_COMMON_PAGE_82_H__
/** @} */
