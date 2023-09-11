#include "ina_functions.h"

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "ina219.h"
#include "ina260.h"

/**
 * This function is used to request data from the ina219 or ina260 modules
 *
 * @param slave_address Address of the ina219 or ina260 modules
 * @param register_address Address of the register you want to read
 * @return Data from the register
 */
uint16_t request_ina_reg_data(uint8_t slave_address, uint8_t register_address) {
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

/**
 * This function is used to read data from the ina219 or ina260 modules readings_n times
 *
 * @param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
 * @param slave_address Address of the ina219 or ina260 modules
 * @param readings_n Number of readings to take
 */
void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n) {
    uint16_t *raw_readings = nullptr;

    for (uint16_t i = 0; i < readings_n; i++) {
        delayMicroseconds(60);
        raw_readings = reading_function(slave_address);
        if (raw_readings != nullptr) {
            Serial.write((byte)(raw_readings[1] & 0xFF));
            Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
            Serial.write((byte)(raw_readings[0] & 0xFF));
            Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
        } else {
            endbit();
            delay(5);
            Serial.println("Error: Unable to get readings.");
        }
    }
}

void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_n, uint8_t from_pwm, uint8_t to_pwm, uint8_t analog_port) {
    /**
     *	This function is used to read data from the ina219 or ina260 modules readings_n times for each pwm value from_pwm to to_pwm
     *
     *	@param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
     *	@param slave_address Address of the ina219 or ina260 modules
     *	@param readings_n Number of readings to take at each pwm value
     *	@param from_pwm PWM value to start from
     *	@param to_pwm PWM value to end at
     *	@param analog_port Port the pwm is connected to
     */

    // Ensure valid PWM range
    if (from_pwm > 255) from_pwm = 255;
    if (to_pwm > 255) to_pwm = 255;

    uint16_t *raw_readings;
    for (uint16_t pwm = from_pwm; pwm <= to_pwm; pwm++) {
        analogWrite(analog_port, pwm);

        // Short delay of 0.5 milliseconds (500 microseconds) to allow the system to stabilize
        delayMicroseconds(200);

        for (uint16_t i = 0; i < readings_n; i++) {
            delayMicroseconds(60);
            raw_readings = reading_function(slave_address);

            if (raw_readings != nullptr) {
                Serial.write((byte)(raw_readings[1] & 0xFF));
                Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
                Serial.write((byte)(raw_readings[0] & 0xFF));
                Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
            } else {
                endbit();
                delay(5);
                Serial.println("Error: Unable to get readings.");
            }
        }
    }
}

void startbit() {
    /**
     * This function is used to send the startbit to the serial monitor
     */
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xF0);
    Serial.write(0xF0);
}

void endbit() {
    /**
     * This function is used to send the endbit to the serial monitor
     */
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0x0F);
    Serial.write(0x0F);
}

void read_motor_11() {
    /**
     * This function is used to read data from the ina219 module connected to motor 11
     */
    delay(100);
    startbit();
    rw_data_n_times(read_ina219_data, INA219_ADDRESS, 20, 0, 255, 11);
    analogWrite(11, 255);
    rw_data_n_times(read_ina219_data, INA219_ADDRESS, 5000);
    endbit();
    analogWrite(11, 0);
}