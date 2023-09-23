#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// Default = 0x40
// A0 soldered = 0x41
// A1 soldered = 0x44
// A0 and A1 soldered = 0x45
const uint8_t INA219_PWM3 = 0x40;   // INA260 I2C address
const uint8_t INA219_PWM9 = 0x45;   // INA260 I2C address
const uint8_t INA219_PWM10 = 0x41;  // INA260 I2C address
const uint8_t INA219_PWM11 = 0x44;  // INA260 I2C address

const uint8_t PWM3 = 3;    // PWM pin 3
const uint8_t PWM9 = 9;    // PWM pin 9
const uint8_t PWM10 = 10;  // PWM pin 10
const uint8_t PWM11 = 11;  // PWM pin 6

const double MAX_CURRENT{700.0};  // mA
const uint16_t N_READINGS{14000};
const uint8_t READ_PER_PWM{19};

#endif