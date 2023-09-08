#include "ina260.h"

#include <Arduino.h>
#include <Wire.h>

uint16_t *read_ina260_data(uint8_t slave_address) {
    static uint16_t dataArray[2];  // Array to store the raw data

    Wire.beginTransmission((uint8_t)slave_address);
    Wire.write(0x01);  // Register address for reading current
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 register)
    if (Wire.available() >= 2) {
        dataArray[0] = (Wire.read() << 8) | Wire.read();  // Current
    } else {
        Serial.println("Error: Unable to read data from INA260");
    }

    Wire.beginTransmission((uint8_t)slave_address);
    Wire.write(0x02);  // Register address for reading voltage
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    if (Wire.available() >= 2) {
        dataArray[1] = (Wire.read() << 8) | Wire.read();  // Voltage
    } else {
        Serial.println("Error: Unable to read data from INA260");
    }

    // Wire.beginTransmission((uint8_t)slave_address);
    // Wire.write(0x03);  // Register address for reading voltage
    // Wire.endTransmission();

    // Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    // if (Wire.available() >= 2) {
    // 	dataArray[2] = (Wire.read() << 8) | Wire.read();  // Voltage
    // } else {
    // 	Serial.println("Error: Unable to read data from INA260");
    // }

    return dataArray;
}

void set_ina260_mode(uint8_t slave_address) {
    // page 22 https://www.ti.com/lit/ds/symlink/ina260.pdf?ts=1692770623410&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA260%253FHQS%253Dti-null-null-verifimanuf_manuf-manu-pf-octopart-wwe
    Serial.println("Setting INA260 mode");

    // Actually 0b01100001 00100111
    Serial.println("Def config: 110000100100111");

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    if (Wire.available() >= 2) {
        uint16_t config_data;
        config_data = (Wire.read() << 8) | (Wire.read());  // Voltage
        Serial.print("Old config: ");
        Serial.println(config_data, BIN);
    } else {
        Serial.println("Error: Unable to read data from INA260");
    }
    Wire.endTransmission();

    // Prepare config register settings
    uint8_t high_byte = 0b01100000;
    uint8_t low_byte = 0b00000111;

    // Begin transmission with selected slave module
    Wire.beginTransmission(slave_address);

    // select config register
    Wire.write(0x00);

    // write new config
    Wire.write(high_byte);
    Wire.write(low_byte);

    // end transmission with selected slave module
    Wire.endTransmission();

    // get new confing setting to check if it's working
    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    if (Wire.available() >= 2) {
        uint16_t config_data;
        config_data = (Wire.read() << 8) | (Wire.read());  // Voltage
        Serial.print("New config: ");
        Serial.println(config_data, BIN);
    } else {
        Serial.println("Error: Unable to read data from INA260");
    }
    Wire.endTransmission();
}