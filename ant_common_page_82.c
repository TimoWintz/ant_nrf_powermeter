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
#include "sdk_common.h"

#include "ant_common_page_82.h"

#define NRF_LOG_MODULE_NAME ant_common_page_82
#if ANT_COMMON_PAGE_82_LOG_ENABLED
#define NRF_LOG_LEVEL       ANT_COMMON_PAGE_82_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  ANT_COMMON_PAGE_82_INFO_COLOR
#else // ANT_COMMON_PAGE_82_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // ANT_COMMON_PAGE_82_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief ant+ common page 82 data layout structure. */
typedef struct
{
    uint8_t reserved; ///< unused, fill by 0xFF
    uint8_t battery_identifier;
    uint8_t cumulative_operating_time[3];
    uint8_t fractional_battery_voltage;
    uint8_t descriptive_bit_field;
}ant_common_page82_data_layout_t;



void ant_common_page_82_encode(uint8_t                                 * p_page_buffer,
                               volatile ant_common_page82_data_t const * p_page_data)
{
    ant_common_page82_data_layout_t * p_outcoming_data =
        (ant_common_page82_data_layout_t *)p_page_buffer;

    p_outcoming_data->reserved = UINT8_MAX;
    p_outcoming_data->battery_identifier = UINT8_MAX;
    for (int i = 0; i < 3 ; i++)
        p_outcoming_data->cumulative_operating_time[i] = 0;
    p_outcoming_data->fractional_battery_voltage = UINT8_MAX;

    uint8_t descriptive_bit_field = 0xF;
    descriptive_bit_field = descriptive_bit_field | (p_page_data->battery_status << 2);
    p_outcoming_data->descriptive_bit_field = descriptive_bit_field;
}


void ant_common_page_82_decode(uint8_t const                     * p_page_buffer,
                               volatile ant_common_page82_data_t * p_page_data)
{
    ant_common_page82_data_layout_t * p_outcoming_data =
        (ant_common_page82_data_layout_t *)p_page_buffer;
    p_page_data->battery_status = (p_page_data->battery_status << 2) & 0b111111;
}
