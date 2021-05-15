/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Garmin Canada Inc. 2012
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 *    1) Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *
 *    2) Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *    3) Neither the name of Garmin nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior
 *       written permission.
 *
 * The following actions are prohibited:
 *
 *    1) Redistribution of source code containing the ANT+ Network
 *       Key. The ANT+ Network Key is available to ANT+ Adopters.
 *       Please refer to http://thisisant.com to become an ANT+
 *       Adopter and access the key. 
 *
 *    2) Reverse engineering, decompilation, and/or disassembly of
 *       software provided in binary form under this license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW 
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 *
 */


#include <stdio.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "ant_key_manager.h"
#include "ant_bpwr.h"
#include "ant_state_indicator.h"
#include "ant_interface.h"


#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "LSM6DS3.h"
#include "ADS1232.h"
#include "power.h"
#include "storage.h"
#include "battery.h"

#include "config.h"

#define ACCEL_MEAS_INTERVAL                APP_TIMER_TICKS(1000/ACCEL_HZ)

#define SCALE_HZ 10
#define SCALE_MEAS_INTERVAL                APP_TIMER_TICKS(1000/SCALE_HZ)

#define DEFAULT_SCALE_OFFSET 70000
#define DEFAULT_SCALE_SCALE 2000

#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)
    #error Unsupported value of MODIFICATION_TYPE.
#endif

#define CRANK_LENGTH 175



/** DATA STORAGE */

static stored_data_t m_stored_data;
static void init_stored_data() {
    m_stored_data.params.scale_offset = DEFAULT_SCALE_OFFSET;
    m_stored_data.params.scale_scale = DEFAULT_SCALE_SCALE;
    storage_init();
    storage_read(&m_stored_data);
}


/** POWER COMPUTATION */

static power_compute_t m_power_compute;

static void init_power_compute() {
    m_power_compute.accel = 0.0;

    m_power_compute.force_cnt = 0;
    m_power_compute.force_sum = 0.0;

    m_power_compute.rev_cnt = 0;
    m_power_compute.rev_timer_cnt = 0;

    m_power_compute.crank_length = 0.001 * CRANK_LENGTH;
    m_power_compute.rev_timer_cnt = app_timer_cnt_get();
}


/** IMU */

static imu_t m_imu;

static void init_imu() {
    m_imu.accel_enabled = true;
    m_imu.gyro_enabled = false;
    m_imu.temp_enabled = true;

    imu_begin(&m_imu);
}

/** SCALE */

static scale_t m_scale;
static bool wait_for_calibration = false;

static void init_scale() {
    scale_begin(GAIN64, SLOW);
    m_scale.calibrating = true;
    m_scale.offset = m_stored_data.params.scale_offset;
    m_scale.scale = m_stored_data.params.scale_scale;
    
    int error = scale_read(&m_scale);
    APP_ERROR_CHECK(error);
    m_scale.calibrating = false;
}

/** SLEEP MODE */


static void sleep_mode_enter(bool wake_up)
{
    NRF_LOG_INFO("Entering sleep mode");
    NRF_LOG_FLUSH();

    scale_power_down();


    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    if (wake_up) {
        // Prepare wakeup buttons.
        imu_wake_up_interrupt();
        nrf_gpio_cfg_input (IMU_INT1_PIN, NRF_GPIO_PIN_PULLDOWN);
        while (true) {
            uint32_t val = nrf_gpio_pin_read(IMU_INT1_PIN);
            if (val > 0) {
                NRF_LOG_INFO("HIGH");
            }
            else {
                break;
            }
            NRF_LOG_FLUSH();
            nrf_delay_ms(100);
        }
        nrf_gpio_cfg_sense_input(IMU_INT1_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    }
    else {
        imu_power_down();
    }


    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/** BATTERY STATUS */

static int m_battery_status;

static void init_battery() {
    saadc_init();
}


/** TIMERS */

static bool wait_for_sleep = false;

APP_TIMER_DEF(m_accel_timer_id);

static void accel_timeout_handler(void * p_context) // Read acceleration
{
    imu_update(&m_imu);
    power_update_accel(&m_power_compute, &m_imu);
    if (app_timer_cnt_get() - m_power_compute.rev_timer_cnt > APP_TIMER_TICKS(SLEEP_TIMEOUT_S * 1000)) {
        wait_for_sleep = true;
    }
}

APP_TIMER_DEF(m_scale_timer_id);

static void scale_timeout_handler(void * p_context) // Read scale
{
    if (scale_available()) {
        scale_read(&m_scale);
    }
    else {
        NRF_LOG_WARNING("Scale unavailable !")
    }
    power_update_scale(&m_power_compute, &m_scale);
}



static void timers_init(void)
{
    ret_code_t err_code;

     // Create timers.
    err_code = app_timer_create(&m_accel_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                accel_timeout_handler);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_scale_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                scale_timeout_handler);

    APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_accel_timer_id, ACCEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_scale_timer_id, SCALE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/** ANT */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUM,
                             CHAN_ID_TRANS_TYPE,
                             CHAN_ID_DEV_NUM,
                             ANTPLUS_NETWORK_NUM);
BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                            (ant_bpwr_torque_t)(SENSOR_TYPE),
                            ant_bpwr_calib_handler,
                            ant_bpwr_evt_handler);

static ant_bpwr_profile_t m_ant_bpwr;


NRF_SDH_ANT_OBSERVER(m_ant_observer, ANT_BPWR_ANT_OBSERVER_PRIO,
                     ant_bpwr_sens_evt_handler, &m_ant_bpwr);


void bsp_evt_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            break;

        case BSP_EVENT_KEY_1:
            break;

        case BSP_EVENT_KEY_2:
            break;

        default:
            break;
    }
}


void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
    nrf_pwr_mgmt_feed();
    uint8_t bat;

    switch (event)
    {
        case ANT_BPWR_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_16_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_17_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_18_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_80_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_81_UPDATED:
            bat = get_battery_level();
            if (bat >= 100) {
                m_ant_bpwr.BPWR_PROFILE_battery_status = BATTERY_NEW;
            }
            else if (bat >= 80) {
                m_ant_bpwr.BPWR_PROFILE_battery_status = BATTERY_GOOD;
            }
            else if (bat >= 60) {
                m_ant_bpwr.BPWR_PROFILE_battery_status = BATTERY_OK;
            }
            else if (bat >= 40) {
                m_ant_bpwr.BPWR_PROFILE_battery_status = BATTERY_LOW;
            }
            else {
                m_ant_bpwr.BPWR_PROFILE_battery_status = BATTERY_CRITICAL;
            }
            break;

        default:
            break;
    }
}

typedef enum {
    SET_SCALE = 1
} custom_data_type_t;

static void ant_process_custom_calib_data(uint8_t* data) {
    int32_t* p_value = (int32_t*) &(data[1]);
    if (data[0] == SET_SCALE) {
        m_scale.scale = (float) *p_value;
        wait_for_calibration = true;
        
    }
}

/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR calibration] */
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            wait_for_calibration = true;
            m_scale.calibrating_offset = true;
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            ant_process_custom_calib_data(p_page1->data.custom_calib);
            break;

        default:
            break;
    }
}



/**
 * @brief Function for setup all thinks not directly associated with ANT stack/protocol.
 * @desc Initialization of: @n
 *         - app_timer, pre-setup for bsp.
 *         - bsp for signaling LEDs and user buttons.
 */
static void utils_setup(void)
{
    // Initialize and start a single continuous mode timer, which is used to update the event time
    // on the main data page.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS,
                        bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
    APP_ERROR_CHECK(err_code);

    
}

/**
 *@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**
 * @brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
static void profile_setup(void)
{
/** @snippet [ANT BPWR TX Profile Setup] */
    ret_code_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                  BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                  BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ant_channel_radio_tx_power_set(m_ant_bpwr.channel_number, RADIO_TX_POWER_LVL_4, 0);
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
                                           BPWR_MANUFACTURER_ID,
                                           BPWR_MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
                                           BPWR_SW_REVISION_MINOR,
                                           BPWR_SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    m_power_compute.page_16 = &m_ant_bpwr.page_16;
    m_power_compute.common = &m_ant_bpwr.common;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
/** @snippet [ANT BPWR TX Profile Setup] */
}

/**@brief Function for application main entry, does not return.
 */
int main(void)
{
    log_init();
    utils_setup();
    timers_init();
    softdevice_setup();
    profile_setup();

    init_stored_data();
    init_scale();
    init_imu();
    init_power_compute();
    init_battery();

    application_timers_start();

    for (;;)
    {
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
        if (wait_for_calibration && !m_scale.calibrating_offset)
        {
            NRF_LOG_INFO("Calibration done!");
            m_stored_data.params.scale_offset = m_scale.offset;
            m_stored_data.params.scale_scale = m_scale.scale;
            storage_write(&m_stored_data);
            wait_for_calibration = false;
        }
        if (wait_for_sleep)
        {
            sleep_mode_enter(true);
        }
    }
}   
