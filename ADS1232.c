#include "ADS1232.h"
// based on https://github.com/HamidSaffari/ADS123X

#include "nrf_gpio.h"
#include "nrf_timer.h"
#include "nrf_delay.h"
#include "nrf_log.h"

#define DOUT_PIN NRF_GPIO_PIN_MAP(0, 11)
#define SCLK_PIN NRF_GPIO_PIN_MAP(0, 12)
#define PDWN_PIN NRF_GPIO_PIN_MAP(0, 13)
#define GAIN0_PIN NRF_GPIO_PIN_MAP(0, 14)
#define GAIN1_PIN NRF_GPIO_PIN_MAP(0, 15)
#define SPEED_PIN NRF_GPIO_PIN_MAP(0, 16)
#define TEMP_PIN NRF_GPIO_PIN_MAP(0, 02)

static bool scale_unavailable;

void scale_begin(int gain, int speed) {
  nrf_gpio_cfg_input (DOUT_PIN, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_output (SCLK_PIN);
  nrf_gpio_cfg_output (PDWN_PIN);
  nrf_gpio_cfg_output (GAIN0_PIN);
  nrf_gpio_cfg_output (GAIN1_PIN);
  nrf_gpio_cfg_output (SPEED_PIN);
  nrf_gpio_cfg_output (TEMP_PIN);


  scale_set_gain(gain);
  scale_set_speed(speed);
  scale_set_channel();
  scale_power_up();
}

int _speed;

void scale_set_channel()
{
  nrf_gpio_pin_clear(TEMP_PIN);
}
void scale_set_gain(int gain)
{
  switch(gain)
  {
    case GAIN1:
    {
      nrf_gpio_pin_clear(GAIN1_PIN);
      nrf_gpio_pin_clear(GAIN0_PIN);
      break;
    }
    case GAIN2:
    {
      nrf_gpio_pin_clear(GAIN1_PIN);
      nrf_gpio_pin_set(GAIN0_PIN);
      break;
    }
    case GAIN64:
    {
      nrf_gpio_pin_set(GAIN1_PIN);
      nrf_gpio_pin_clear(GAIN0_PIN);
      break;
    }
    case GAIN128:
    {
      nrf_gpio_pin_set(GAIN1_PIN);
      nrf_gpio_pin_set(GAIN0_PIN);
      break;
    }
  }
}

void scale_power_up() {
  nrf_gpio_pin_set(PDWN_PIN);
  nrf_gpio_pin_clear(SCLK_PIN);
}

void scale_power_down() {
  nrf_gpio_pin_clear(PDWN_PIN);
  nrf_gpio_pin_set(SCLK_PIN);
}

void scale_set_speed(int speed)
{
  _speed = speed;
  switch(speed)
  {
    case SLOW:
    {
      nrf_gpio_pin_clear(SPEED_PIN);
      break;
    }
    case FAST:
    {
      nrf_gpio_pin_set(SPEED_PIN);
      break;
    }
  }
}

bool scale_available()
{
  return (!scale_unavailable && nrf_gpio_pin_read(DOUT_PIN) == 0);
}

int scale_read(scale_t* scale)
{
  scale_unavailable = true;
  bool calibrating = scale->calibrating;
  int* value = &(scale->raw);


    int i=0;
    unsigned long start;
    *value = 0;

    // Read 24 bits
    for(i=23 ; i >= 0; i--) {
      nrf_gpio_pin_set(SCLK_PIN);
      nrf_delay_us(1);
      *value = (*value << 1) + nrf_gpio_pin_read(DOUT_PIN);
      nrf_gpio_pin_clear(SCLK_PIN);
    }

    if(calibrating){
    // 2 extra bits for calibrating
        for(i=1 ; i >= 0; i--) {
          nrf_gpio_pin_set(SCLK_PIN);
          nrf_delay_us(1);
          nrf_gpio_pin_clear(SCLK_PIN);
        }
    }

    /* Bit 23 is acutally the sign bit. Shift by 8 to get it to the
     * right position (31), divide by 256 to restore the correct value.*/
     
    *value = (*value << 8) / 256;
    

    if(!calibrating){
      /* The data pin now is high or low depending on the last bit that
       * was read.
       * To get it to the default state (high) we toggle the clock one
       * more time (see datasheet).*/
      nrf_gpio_pin_set(SCLK_PIN);
      nrf_delay_us(1);
      nrf_gpio_pin_clear(SCLK_PIN);
             
    }
    while(scale_available());
    scale->units = (float)(scale->raw - scale->offset) / scale->scale;
    NRF_LOG_INFO("Scale reading: %i (%i mN)", *value, (int) (1000 * scale->units));
    scale_unavailable = false;
    return NoERROR; // Success
}