#include "LSM6DS3.h"

#include <math.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_drv_twi.h"

#define LSM6DS3_ADDRESS            0x6A

#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10
#define LSM6DS3_CTRL2_G            0X11

#define LSM6DS3_STATUS_REG         0X1E

#define LSM6DS3_CTRL3_C            0X12
#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17

#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23
#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25
#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27

#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29
#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B
#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D

#define LSM6DS3_OUT_L_TEMP         0X20
#define LSM6DS3_OUT_H_TEMP         0X21


#define SDA_PIN NRF_GPIO_PIN_MAP(0, 27)
#define SCL_PIN NRF_GPIO_PIN_MAP(0, 28)
#define CS_PIN NRF_GPIO_PIN_MAP(0, 29)
#define SAD_PIN NRF_GPIO_PIN_MAP(0, 26)

#define ACCEL_RANGE 4
#define GYRO_RANGE 125


#define ERROR

static const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);
static volatile bool xfer_completed = false;
static int16_t buffer[3];

void twi_event_handler(nrf_drv_twi_evt_t * p_event, void * p_context)
{
    if ((p_event->type == NRF_DRV_TWI_EVT_DONE))
    { xfer_completed = true; } 
}

static float v[2];
float imu_filter_step(float x) {
  v[0] = v[1];
  v[1] = (2.127896704574157583e-1 * x)
  + (0.57442065908516848349 * v[0]);
  return (v[0] + v[1]);
}



static float gyro_raw_to_dps(int16_t val) {
  uint8_t divisor = GYRO_RANGE / 125;
  if (GYRO_RANGE == 245 ) {
      divisor = 2;
  }
  return (float)val * 4.375 * (divisor) / 1000;
}

static float accel_raw_to_g(int16_t val) {
   return (float)val *0.061 * (ACCEL_RANGE >> 1) / 1000;
}

static float temp_raw_to_c(int16_t val) {
  return (float)val / 16 + 25;
}



static bool read_registers(uint8_t address, uint8_t* data, size_t length) {
  ret_code_t ret = nrf_drv_twi_tx(&twi, LSM6DS3_ADDRESS, &address, 1, true);
  if (ret != NRF_SUCCESS) {
    APP_ERROR_CHECK(ret);
    return false;
  }
  ret = nrf_drv_twi_rx(&twi, LSM6DS3_ADDRESS, data, length);
  if (ret != NRF_SUCCESS) {
    APP_ERROR_CHECK(ret);
    return false;
  }
  return true;
}

static int read_register(uint8_t address) {
  uint8_t value;
  if (!read_registers(address, &value, 1)) {
    NRF_LOG_WARNING("READ %i", value);
    return -1;
  }
  return (int) value;
}

static bool write_register(uint8_t address, uint8_t value) {
  static uint8_t data[2];
  data[0] = address;
  data[1] = value;
  ret_code_t ret = nrf_drv_twi_tx(&twi, LSM6DS3_ADDRESS, data, 2, false);
  if (ret != NRF_SUCCESS) {
    APP_ERROR_CHECK(ret);
    return false;
  }
  return true;
}

static bool imu_gyro_available() {
  int value = read_register(LSM6DS3_STATUS_REG);
  return (value >= 0 && (value & 0x02));
}

bool imu_begin(imu_t* p_imu) {
  ret_code_t err_code;
  const nrf_drv_twi_config_t twi_config = {
     .scl                = SCL_PIN,
     .sda                = SDA_PIN,
     .frequency          = NRF_DRV_TWI_FREQ_400K,
     .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
     .clear_bus_init     = false
  };
  err_code = nrf_drv_twi_init(&twi, &twi_config, NULL, NULL);

  if (err_code !=NRF_SUCCESS) {
    NRF_LOG_ERROR("Failed to initialize TWI interface (%i)", err_code);
    APP_ERROR_CHECK(err_code);
    return false;
  }
  nrf_drv_twi_enable(&twi);

  // Enable I2C interface on slave 0
  nrf_gpio_cfg_output(CS_PIN);
  nrf_gpio_pin_set(CS_PIN);

  nrf_gpio_cfg_output(SAD_PIN);
  nrf_gpio_pin_clear(SAD_PIN);


  int id = read_register(LSM6DS3_WHO_AM_I_REG);
  NRF_LOG_INFO("IMU ID: %i", id);


  if (id != 0x69) {
    NRF_LOG_ERROR("Failed to identify IMU", err_code);
    return false;
  }

  if (p_imu->accel_enabled) {
    // Set the Accelerometer control register to work at 104 Hz, 4G,and in bypass mode and enable ODR/4
    // low pass filter(check figure9 of LSM6DS3's datasheet)
    if (!write_register(LSM6DS3_CTRL1_XL, 0x4A)) {
      return false;
    }
    // Set the ODR config register to ODR/4
    if (! write_register(LSM6DS3_CTRL8_XL, 0x09)) {
      return false;
    }
  }
  else {
    if (!write_register(LSM6DS3_CTRL1_XL, 0x00)) {
      return false;
    }
  }
  if (p_imu->gyro_enabled) {
    // set gyroscope power mode to high performance and bandwidth to 16 MHz
      //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
    if (!write_register(LSM6DS3_CTRL2_G, 0x4C)) {
      return false;
    }
    if (!write_register(LSM6DS3_CTRL7_G, 0x00)) {
      return false;
    }
  }
  else {
    if (!write_register(LSM6DS3_CTRL2_G, 0x00)) {
      return false;
    }
  }
  p_imu->ax_g = 0;
  p_imu->ay_g = 0;
  p_imu->az_g = 0;

  p_imu->gx_dps = 0;
  p_imu->gy_dps = 0;
  p_imu->gz_dps = 0;

  p_imu->temp_c = 20;
  return true;
}

static bool imu_read_acceleration(int16_t* data) {
  if (!read_registers(LSM6DS3_OUTX_L_XL, (uint8_t*)data, 3*sizeof(data[0]))) {
    NRF_LOG_WARNING("Failed to read acceleration");
    return false;
  }
  return true;
}

static bool imu_read_temperature(int16_t* data) {
  if (!read_registers(LSM6DS3_OUT_L_TEMP, (uint8_t*)data, sizeof(data[0]))) {
    NRF_LOG_WARNING("Failed to read temperature");
    return false;
  }
  return true;
}

static bool imu_acceleration_available() {
  int value = read_register(LSM6DS3_STATUS_REG);
  return (value >= 0 && (value & 0x01));
}

static bool imu_temperature_available() {
  int value = read_register(LSM6DS3_STATUS_REG);
  return (value >= 0 && (value & 0x03));
}

static bool imu_read_gyro(int16_t* data) {
  if (!read_registers(LSM6DS3_OUTX_L_G, (uint8_t*)data, 3*sizeof(data[0]))) {
    NRF_LOG_WARNING("Failed to read gyro");
    return false;
  }
  return true;
}

void imu_update(imu_t* p_imu) {
      // NRF_LOG_INFO("Reading IMU");
  if (p_imu->accel_enabled && imu_acceleration_available()) {
    imu_read_acceleration(buffer);
    p_imu->ax_g = accel_raw_to_g(buffer[0]);
    p_imu->ay_g = accel_raw_to_g(buffer[1]);
    p_imu->az_g = accel_raw_to_g(buffer[2]);
    // NRF_LOG_INFO("Accel (%i, %i, %i)", buffer[0], buffer[1], buffer[2]);
  }
  if (p_imu->gyro_enabled && imu_gyro_available()) {
      imu_read_gyro(buffer);
      p_imu->gx_dps = gyro_raw_to_dps(buffer[0]);
      p_imu->gy_dps = gyro_raw_to_dps(buffer[1]);
      p_imu->gz_dps = gyro_raw_to_dps(buffer[2]);
  }
  if (p_imu->temp_enabled && imu_temperature_available()) {
      imu_read_temperature(buffer);
      p_imu->temp_c = temp_raw_to_c(buffer[0]);
  }
}