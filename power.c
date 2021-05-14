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

static bool is_down = false;

void power_update_accel(power_compute_t * p_power_compute, imu_t * p_imu)
{
    float revolution_time;
    p_power_compute->accel = filter_step(p_imu->az_g);
    if (p_power_compute->accel > 0.5) {
        is_down = true;
    }
    if (is_down && p_power_compute->accel < -0.5) {
        p_power_compute->rev_cnt++;
        uint32_t rev_timer_cnt = app_timer_cnt_get();
        revolution_time = (rev_timer_cnt - p_power_compute->rev_timer_cnt) / (float) APP_TIMER_TICKS(1000);
        float power_float = (2 * M_PI * p_power_compute->crank_length * 2 * 
            p_power_compute->force_sum / p_power_compute->force_cnt / revolution_time);
        if (power_float < 0) {
            power_float = -power_float;
        }
        p_power_compute->page_16->instantaneous_power = (uint16_t) (power_float);
        p_power_compute->page_16->accumulated_power += (uint16_t) (power_float);
        p_power_compute->page_16->update_event_count++;
        
        p_power_compute->rev_timer_cnt = rev_timer_cnt;
        p_power_compute->common->instantaneous_cadence = (uint16_t) (60.0 / revolution_time);
         NRF_LOG_INFO("Revolution: %i rpm, %i W (%i measurements)",
            (int) p_power_compute->common->instantaneous_cadence,
            (int) power_float,
            (int) p_power_compute->force_cnt);
        p_power_compute->rev_time = revolution_time;
        p_power_compute->force_sum = 0.0;
        p_power_compute->force_cnt = 0;
        is_down = false;
    }
}

void power_update_scale(power_compute_t * p_power_compute, scale_t * p_scale)
{
    p_power_compute->force_sum += p_scale->units;
    p_power_compute->force_cnt++;
}