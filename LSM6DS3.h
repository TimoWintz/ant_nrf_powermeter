#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "nrf_drv_twi.h"
#define ACCEL_HZ 104

typedef struct 
{
    bool accel_enabled;
    float ax_g;
    float ay_g;
    float az_g;

    bool gyro_enabled;
    float gx_dps;
    float gy_dps;
    float gz_dps;

    bool temp_enabled;
    float temp_c;
} imu_t;


bool imu_begin(imu_t* p_imu);
void imu_end(imu_t* p_imu);
void imu_update(imu_t* p_imu);

#endif