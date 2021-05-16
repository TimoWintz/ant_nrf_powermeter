#ifndef CONFIG_H
#define CONFIG_H

#define SCALE_HZ 10

#define SLEEP_TIMEOUT_S 180 // sleep after 3 min
#define IMU_INT1_PIN NRF_GPIO_PIN_MAP(0, 25)

#define AUTO_ZERO_TOLERANCE 1.0 // N constant torque tolerance
#define AUTO_ZERO_INTERVAL 2000 // constant torque value

#endif