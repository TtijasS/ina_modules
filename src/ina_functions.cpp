#include "ina_functions.h"

#include <Arduino.h>
#include <Wire.h>

uint16_t request_reg_data(uint8_t slave_address, uint8_t register_address) {
    Wire.beginTransmission((uint8_t)slave_address);
    Wire.write(register_address);  // Register address you want to read
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (16bit register)
    if (Wire.available() >= 2) {
        uint16_t reg_data{};
        reg_data = (Wire.read() << 8) | (Wire.read());  // Voltage
        return reg_data;
    } else {
        Serial.print("Error: Unable to read data from slave address: ");
        Serial.println(slave_address, HEX);
    }
    return 0;
}