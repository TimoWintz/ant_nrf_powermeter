#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "nrf_drv_twi.h"
#define ACCEL_HZ 104

typedef struct 
{
    bool accel_enabled;
    float accel_g[3];

    bool gyro_enabled;
    float gyro_dps[3];

    bool temp_enabled;
    float temp_c;
} imu_t;


bool imu_begin(imu_t* p_imu);
void imu_end(imu_t* p_imu);
void imu_update(imu_t* p_imu);
void imu_wake_up_interrupt();
void imu_power_down();
void imu_calibrate_gyro();
int32_t imu_get_offset();

#endif