#ifndef INA219_H
#define INA219_H

#include <Arduino.h>

uint16_t *read_ina219_data(uint8_t slave_address);
void set_ina219_mode(uint8_t slave_address);
void set_ina219_calibration_register(float max_current, uint8_t slave_address);
void setup_ina219(uint8_t slave_address, uint16_t max_current);

#endif