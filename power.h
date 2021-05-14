#ifndef POWER_H
#define POWER_H

#include "LSM6DS3.h"
#include "ADS1232.h"
#include "ant_bpwr.h"

typedef struct {
    float v[2];
} filter_t;

typedef struct {
    ant_bpwr_common_data_t* common;  
    ant_bpwr_page16_data_t* page_16;

    float accel;

    uint16_t force_cnt;
    float force_sum;
    float crank_length;

    uint16_t rev_cnt;
    float rev_time;
    uint32_t rev_timer_cnt;
} power_compute_t;

void power_update_accel(power_compute_t * p_power_compute, imu_t * p_imu);
void power_update_scale(power_compute_t * p_power_compute, scale_t * p_scale);
void power_update_pages(power_compute_t * p_power_compute, ant_bpwr_profile_t* p_profile);

#endif