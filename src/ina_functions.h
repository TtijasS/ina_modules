#ifndef INA_FUNCTIONS_H
#define INA_FUNCTIONS_H

#include <Arduino.h>

uint16_t request_ina_reg_data(uint8_t slave_address, uint8_t register_address);

typedef uint16_t* (*ReadingFunction)(uint8_t);

void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n);

void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n, uint8_t from_pwm, uint8_t to_pwm, uint8_t analog_port);

void startbit();

void endbit();

void read_motor_n(uint8_t slave_address, uint8_t analog_port);

#endif