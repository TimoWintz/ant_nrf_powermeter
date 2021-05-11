#include "power.h"

#include "app_timer.h"
#include "nrf_log.h"
# define M_PI           3.14159265358979323846  /* pi */


static float v[2] = {0., 0.};

static float filter_step(float x) {
  v[0] = v[1];
  v[1] = (2.127896704574157583e-1 * x)
  + (0.57442065908516848349 * v[0]);
  return (v[0] + v[1]);
}

void power_update_accel(power_compute_t * p_power_compute, imu_t * p_imu)
{
    float new_accel;
    float revolution_time;
    new_accel = filter_step(p_imu->az_g);
    if (p_power_compute->accel > 0 && new_accel < 0) {
        p_power_compute->rev_cnt++;
        uint32_t rev_timer_cnt = app_timer_cnt_get();
        revolution_time = (float)(rev_timer_cnt - p_power_compute->rev_timer_cnt) / APP_TIMER_CLOCK_FREQ;
        float power_float = 2 * M_PI * p_power_compute->crank_length * 2 * p_power_compute->force_sum / p_power_compute->force_cnt / revolution_time;
        if (power_float < 0) {
            power_float = -power_float;
        }
        p_power_compute->last_power = (uint16_t) (power_float);
        
        p_power_compute->rev_timer_cnt = rev_timer_cnt;
        p_power_compute->last_cadence = (uint16_t) (60.0 / revolution_time);
         NRF_LOG_INFO("Revolution: %i rpm, %i W (%i measurements)",
            (int) p_power_compute->last_cadence,
            (int) p_power_compute->last_power,
            (int) p_power_compute->force_cnt);
        p_power_compute->rev_time = revolution_time;
        p_power_compute->force_sum = 0.0;
        p_power_compute->force_cnt = 0;
       
    }
    p_power_compute->accel = new_accel;
}

void power_update_scale(power_compute_t * p_power_compute, scale_t * p_scale)
{
    p_power_compute->force_sum += p_scale->units;
    p_power_compute->force_cnt++;
}

void power_update_pages(power_compute_t * p_power_compute, ant_bpwr_profile_t* p_profile) {
    p_profile->BPWR_PROFILE_instantaneous_power = p_power_compute->last_power;
    p_profile->BPWR_PROFILE_accumulated_power += p_power_compute->last_power;
    p_profile->BPWR_PROFILE_instantaneous_cadence = p_power_compute->last_cadence;
    p_profile->BPWR_PROFILE_power_update_event_count++;
}