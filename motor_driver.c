#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "motor_driver.h"

#define I2C_PORT i2c0
#define MOTOR_ADDR 0x0A

int motor_read(struct motor* motor)
{
    uint8_t buffer[5];
    if (i2c_read_blocking(I2C_PORT, MOTOR_ADDR, buffer, 5, false) != 5)
    {
        return -1;
    }

    motor->state = buffer[0];
	float* floatPtr = (float*)(buffer + 1);
	motor->torque_meas = *floatPtr;
    return 0;
}

int motor_write(struct motor* motor)
{
    uint8_t buffer[5];
    buffer[0] = motor->direction;
	*(float*)(buffer + 1) = motor->torque;

    if (i2c_write_blocking(I2C_PORT, MOTOR_ADDR, buffer, 5, false) != 5);
    {
        return -1;
    }
    return 0;
}

void motor_init(struct motor* motor)
{
    motor->torque = 0;
	motor->direction = 0;
	motor->torque_meas = 0;
	motor->state = 0;
}

