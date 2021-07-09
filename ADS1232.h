#ifndef ADS1232_H
#define ADS1232_H

#include <stdbool.h>



enum ERROR_t {
	NoERROR,
	TIMEOUT_HIGH,     // Timeout waiting for HIGH
	TIMEOUT_LOW,      // Timeout waiting for LOW
	WOULD_BLOCK,      // weight not measured, measuring takes too long
	STABLE_TIMEOUT,   // weight not stable within timeout
	DIVIDED_by_ZERO    
};

enum Gain{
	GAIN1 = 1,
	GAIN2,
	GAIN64,
	GAIN128
};

enum Speed{
	SLOW = 0,
	FAST
};

enum Channel{
	AIN1 = 1,
	AIN2 = 2,
	TEMP = 3
};

typedef struct {
	int raw;
	float units;
	int offset;
	float scale;
	bool calibrating;
	bool calibrating_offset;

	float temp_c;
} scale_t;


void scale_begin(int gain, int speed);
void scale_end();
bool scale_available();
int scale_read(scale_t* scale);
void scale_power_up();
void scale_power_down();
void scale_set_gain(int gain);
void scale_set_speed(int speed);
void scale_set_channel();
void scale_set_scale(long scale);
void scale_set_offset(long offset);
int scale_read_units(float* value, long* value_raw);
unsigned long millis(void);

#endif