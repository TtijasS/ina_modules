#ifndef INA_FUNCTIONS_H
#define INA_FUNCTIONS_H

#include <Arduino.h>

uint16_t request_reg_data(uint8_t slave_address, uint8_t register_address);

typedef uint16_t* (*ReadingFunction)(uint8_t);

void send_data_n_times(uint8_t slave_address, uint16_t number_of_readings, ReadingFunction read_ina_data);

#endif