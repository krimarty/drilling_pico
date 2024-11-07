#ifndef kriz_linear
#define kriz_linear

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

struct linear{
    uint8_t command;
    uint8_t speed;
    uint8_t states;
    uint16_t height;
};

int linear_read(struct linear* linear);
int linear_write(struct linear* linear);
void linear_init(struct linear* linear);




#endif