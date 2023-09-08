#ifndef INA_FUNCTIONS_H
#define INA_FUNCTIONS_H

#include <Arduino.h>

uint16_t request_reg_data(uint8_t slave_address, uint8_t register_address);

#endif