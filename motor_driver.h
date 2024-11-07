#ifndef kriz_motor
#define kriz_motor

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

struct motor {
	float torque;
	uint8_t direction;
	float torque_meas;
	uint8_t state;
	};

int motor_read(struct motor* motor);
int motor_write(struct motor* motor);
void motor_init(struct motor* motor);

#endif