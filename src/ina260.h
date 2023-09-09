#ifndef INA260_H
#define INA260_H

#include <Arduino.h>

void set_ina260_mode(uint8_t slave_address);
uint16_t *read_ina260_data(uint8_t slave_address);

#endif