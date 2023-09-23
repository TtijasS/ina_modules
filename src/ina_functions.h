#ifndef INA_FUNCTIONS_H
#define INA_FUNCTIONS_H

#include <Arduino.h>

#include "fsm_cls.h"

uint16_t request_ina_reg_data(uint8_t slave_address, uint8_t register_address);

typedef uint16_t *(*ReadingFunction)(uint8_t);

void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n);

void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n, uint8_t from_pwm, uint8_t to_pwm, uint8_t analog_port);

void rw_data_fsm(FsmCls &fsm, uint8_t slave_address, uint8_t analog_port);

void read_device_ntimes(uint8_t slave_address, uint8_t analog_port, uint16_t readings_n);

void utf8_mode_startbit();

void utf8_mode_stopbit();

void measuring_mode_startbit();

void measuring_mode_stopbit();

void multimeasuring_mode_startbit();

void exit_serial_signal();

void multi_device_measuring(uint16_t n_readings, uint16_t per_pwm);

void store_delay();

void send_deltatime(unsigned long deltatime);

void request_delays_startbit();

#endif