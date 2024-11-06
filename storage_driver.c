#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define STORAGE_ADDR 0x08

struct storage{
    uint8_t command;
    uint8_t active_slot;
    uint8_t pos_to_store;
    uint8_t errors;
    uint16_t weight;
    bool meas_ready;
    bool manual;
    uint16_t samples[3];
    uint8_t raw;
};

int storage_read(struct storage* storage)
{
    uint8_t buffer[3];
    if (i2c_read_blocking(I2C_PORT, STORAGE_ADDR, buffer, 3, false) != 3)
    {
        return -1;
    }
    uint16_t *ptr = (uint16_t*) buffer;
    storage->weight = *ptr;
    
    storage->raw = buffer[2];
    uint8_t tmp = 192 & buffer[2]; //192 = B'1100 0000'
    storage->active_slot = tmp >> 6;


    tmp = 32 & buffer[2]; //32 = B'0010 0000'
    storage->meas_ready = tmp >> 5;

    tmp = 16 & buffer[2]; //16 = B'0001 0000'
    storage->manual = tmp >> 4;

    storage->errors = 15 & buffer[2]; //15 = B'0000 1111'
    return 0;
}

int storage_write(struct storage* storage)
{
    uint8_t buffer = storage->command;
    if (i2c_write_blocking(I2C_PORT, STORAGE_ADDR, &buffer, 1, false) != 1);
    {
        return -1;
    }
    return 0;
}

void storage_init(struct storage *storage)
{
    storage->command = 0;
    storage->active_slot = 0;
    storage->pos_to_store = 0;
    storage->errors = 0;
    storage->weight = 0;
    storage->meas_ready = false;
    storage->manual = false;
    storage->samples[0] = 0;
    storage->samples[1] = 0;
    storage->samples[2] = 0;
}

