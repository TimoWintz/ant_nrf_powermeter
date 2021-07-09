#ifndef CONFIG_H
#define CONFIG_H

#define SCALE_HZ 10
#define FLASH_WRITE_MAX_TRIES 10

#define SLEEP_TIMEOUT_S 180 // sleep after 3 min
#define IMU_INT1_PIN NRF_GPIO_PIN_MAP(0, 25)

#define AUTO_ZERO_TOLERANCE 0.05 // N constant torque tolerance
#define AUTO_ZERO_INTERVAL 3000 // constant torque value

#define USE_GYRO 1 // Use gyro or accelerometer for cadence

#define GYRO_CALIBRATION_MEAS 10

#define ACCEL_AXIS 2
#define GYRO_AXIS 0

#endif