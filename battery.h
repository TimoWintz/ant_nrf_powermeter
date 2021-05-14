#ifndef BATTERY_H
#define BATTERY_H

// https://devzone.nordicsemi.com/f/nordic-q-a/50141/how-to-measure-a-state-of-a-battery-by-using-saadc-example-with-nrf52832

#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_saadc.h"

#define INPUTS 1
#define TIMER  1000 // ms
#define AIN_BATTERY 0

APP_TIMER_DEF(saadc_timer);

static nrf_saadc_value_t buffer[2][INPUTS];
static uint8_t battery_level = 0xFF;

static inline uint16_t adc_to_mv(nrf_saadc_value_t adc_result) {
    // http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52840.ps/saadc.html?cp=2_0_0_5_22_2#saadc_digital_output
    // Vresult[mV] = (adc_result * Vref[mV]) / (2^resolution * gain) = (adc_result * 600) / (2^10 * 1/6)
    return (adc_result * 3600) >> 10;
}

uint8_t get_battery_level() {
    return battery_level;
}

static void saadc_timeout_handler(void *p_context) { // ADC for battery status
    UNUSED_PARAMETER(p_context);
    APP_ERROR_CHECK(nrfx_saadc_sample());
}

void saadc_event_handler(nrfx_saadc_evt_t const *p_event) {
    if (p_event->type != NRFX_SAADC_EVT_DONE)
        return;
    nrf_saadc_value_t *adc_result = p_event->data.done.p_buffer;
    uint16_t mv = (adc_to_mv(adc_result[AIN_BATTERY]) * 105) / 100; // max voltage is only 2.86V instead of 3V
    battery_level = battery_level_in_percent(mv);

    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(adc_result, INPUTS));
}

void saadc_init(void) {
    //  SAADC config & calibrate
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
    APP_ERROR_CHECK(nrfx_saadc_init(&saadc_config, saadc_event_handler));
    APP_ERROR_CHECK(nrfx_saadc_calibrate_offset());
    while (nrfx_saadc_is_busy())
        sd_app_evt_wait();


    // Channels config
    nrf_saadc_channel_config_t battery_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    APP_ERROR_CHECK(nrfx_saadc_channel_init(AIN_BATTERY, &battery_config));

    // double buffering
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(buffer[0], INPUTS));
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(buffer[1], INPUTS));

    // start SAADC timer
    APP_ERROR_CHECK(app_timer_create(&saadc_timer, APP_TIMER_MODE_REPEATED, saadc_timeout_handler));
    APP_ERROR_CHECK(app_timer_start(saadc_timer, APP_TIMER_TICKS(TIMER), NULL));
}

#endif