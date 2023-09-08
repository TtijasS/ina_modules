#include "ina219.h"

#include <Arduino.h>
#include <Wire.h>

// ina219 specific constants
const double SHUNT_VAL = 0.002;            // 2mOhm
const double BUS_VOLTAGE_LSB = 0.004;      // 4mV
const unsigned int REGISTER_RANGE{32768};  // 15 bit

// pg 22 https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1692784130490&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-eu%2526utm_content%253DINA219%2526ds_k%253DINA219%2BDatasheet%2526DCM%253Dyes%2526gclid%253DCj0KCQjw3JanBhCPARIsAJpXTx7DPjhosIxfe6pl48GAchbIMAvx_KeAZJ5H3Dv8KqVLjmxZV7nUCesaAm1BEALw_wcB%2526gclsrc%253Daw.ds
uint16_t *read_ina219_i_v(uint8_t slave_address) {
    static uint16_t raw_readings[2];  // Array to store the raw data

    Wire.beginTransmission((uint8_t)slave_address);
    Wire.write(0x04);  // current register
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 register)
    if (Wire.available() >= 2) {
        raw_readings[0] = (Wire.read() << 8) | Wire.read();  // Current
    } else {
        Serial.println("Error: Unable to read data from INA219");
    }

    Wire.beginTransmission((uint8_t)slave_address);
    Wire.write(0x02);  // bus voltage register
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    if (Wire.available() >= 2) {
        raw_readings[1] = (Wire.read() << 8) | Wire.read();  // Voltage
        raw_readings[1] >>= 3;
    } else {
        Serial.println("Error: Unable to read data from INA219");
    }

    return raw_readings;
}

// page 19 https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1692784130490&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-eu%2526utm_content%253DINA219%2526ds_k%253DINA219%2BDatasheet%2526DCM%253Dyes%2526gclid%253DCj0KCQjw3JanBhCPARIsAJpXTx7DPjhosIxfe6pl48GAchbIMAvx_KeAZJ5H3Dv8KqVLjmxZV7nUCesaAm1BEALw_wcB%2526gclsrc%253Daw.ds
void set_ina219_mode(uint8_t slave_address) {
    Serial.println("Setting INA219 mode");
    // default 111001 10011111
    //     new 111000 10001111
    // Prepare config register settings
    uint8_t high_byte = 0b00111000;
    uint8_t low_byte = 0b10001111;

    // Begin transmission with selected slave module
    Wire.beginTransmission(slave_address);

    // select config register
    Wire.write(0x00);

    // write new config
    Wire.write(high_byte);
    Wire.write(low_byte);

    // end transmission with selected slave module
    Wire.endTransmission();
}

// pg 12 https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1692784130490&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-eu%2526utm_content%253DINA219%2526ds_k%253DINA219%2BDatasheet%2526DCM%253Dyes%2526gclid%253DCj0KCQjw3JanBhCPARIsAJpXTx7DPjhosIxfe6pl48GAchbIMAvx_KeAZJ5H3Dv8KqVLjmxZV7nUCesaAm1BEALw_wcB%2526gclsrc%253Daw.ds
void set_ina219_calibration_register(float max_current, uint8_t slave_address) {
    // 1A max -> current_lsb = 30.517uA
    // cal_reg = 671.1 (671 truncated)
    // new cal_reg value binary is then 1010011110

    // pow(2,15) equals (1 << 15) shift
    // unsigned int register_range{1 << 15};

    double current_lsb = max_current / REGISTER_RANGE;  // gives 0.000030517578125
    double calibration_value = 0.04096 / (current_lsb * SHUNT_VAL);
    Serial.print("Calibration value: ");
    Serial.println(calibration_value);

    // convert to uint16_t and truncate the decimal part
    uint16_t calibration_int = (static_cast<uint16_t>(calibration_value) + 1);

    Serial.print("Truncated pre shift:  ");
    Serial.println(calibration_int, BIN);

    calibration_int = calibration_int << 1;  // shift left by 1 bit because FS0 (LSB) is not used
    // keep the MSB and truncate the LSB
    uint8_t calibration_MSB = (calibration_int >> 8) & 0xFF;
    uint8_t calibration_LSB = calibration_int & 0xFF;

    Serial.print("Truncated post shift: ");
    Serial.println(calibration_int, BIN);

    Serial.print("MSB: ");
    Serial.print(calibration_MSB, BIN);
    Serial.print(" LSB: ");
    Serial.println(calibration_LSB, BIN);
    // Begin transmission with selected slave module
    Wire.beginTransmission(slave_address);
    // select config register
    Wire.write(0x05);

    // // write new config
    Wire.write((calibration_int >> 8) & 0xFF);
    Wire.write(calibration_int & 0xFF);
    // Wire.write(0b00000010);
    // Wire.write(0b10011111);

    // end transmission with selected slave module
    Wire.endTransmission();
}