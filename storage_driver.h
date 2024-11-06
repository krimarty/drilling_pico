#ifndef kriz_246877
#define kriz_246877

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"


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


int storage_read(struct storage* storage);
int storage_write(struct storage* storage);
void storage_init(struct storage* storage);



#endif
