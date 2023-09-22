#include "ina_functions.h"

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "fsm_cls.h"
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
 * This function is used to read data from the ina219 or ina260 modules n_readings times
 *
 * @param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
 * @param slave_address Address of the ina219 or ina260 modules
 * @param n_readings Number of readings to take
 */
void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t n_readings) {
    uint16_t *raw_readings = nullptr;

    for (uint16_t i = 0; i < n_readings; i++) {
        delayMicroseconds(60);
        raw_readings = reading_function(slave_address);
        if (raw_readings != nullptr) {
            Serial.write((byte)(raw_readings[1] & 0xFF));
            Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
            Serial.write((byte)(raw_readings[0] & 0xFF));
            Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
        } else {
            measuring_mode_stopbit();
            delay(5);
            Serial.println("Error: Unable to get readings.");
        }
    }
}

/**
 *	This function is used to read data from the ina219 or ina260 modules n_readings times for each pwm value from_pwm to to_pwm
 *
 *	@param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
 *	@param slave_address Address of the ina219 or ina260 modules
 *	@param n_readings Number of readings to take at each pwm value
 *	@param from_pwm PWM value to start from
 *	@param to_pwm PWM value to end at
 *	@param analog_port Port the pwm is connected to
 */
void rw_data_n_times(ReadingFunction reading_function, uint8_t slave_address, uint16_t n_readings, uint8_t from_pwm, uint8_t to_pwm, uint8_t analog_port) {
    // Ensure valid PWM range
    if (from_pwm > 255) from_pwm = 255;
    if (to_pwm > 255) to_pwm = 255;

    uint16_t *raw_readings;
    for (uint16_t pwm = from_pwm; pwm <= to_pwm; pwm++) {
        analogWrite(analog_port, pwm);

        for (uint16_t i = 0; i < n_readings; i++) {
            raw_readings = reading_function(slave_address);

            if (raw_readings != nullptr) {
                Serial.write((byte)(raw_readings[1] & 0xFF));
                Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
                Serial.write((byte)(raw_readings[0] & 0xFF));
                Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
            } else {
                measuring_mode_stopbit();
                delay(5);
                Serial.println("Error: Unable to get readings.");
            }
        }
    }
}

/**
 * This is the FSM version of function that is used to read data from the ina219 or ina260 modules n_readings times
 *
 * @param fsm State machine
 * @param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
 * @param slave_address Address of the ina219 or ina260 modules
 * @param readings_per_pwm Number of readings to take at each pwm value until max is reached.
 * @param analog_port Port the pwm is connected to
 */
void rw_data_fsm(FsmCls &fsm, ReadingFunction reading_function, uint8_t slave_address, uint16_t readings_per_pwm, uint8_t analog_port) {
    uint16_t *raw_readings = nullptr;

    // Serial.print("PWM  ");
    // Serial.print(analog_port);
    // Serial.print(" > pwm_state: ");
    // Serial.print(fsm.get_pwm_state());
    // Serial.print(" readings_state: ");
    // Serial.println(fsm.get_readings_state());

    // read data
    raw_readings = reading_function(slave_address);

    if (raw_readings != nullptr) {
        // write divice id (array position on the pyhton side)
        switch (analog_port) {
            case 3:
                Serial.write((byte)0);
                break;
            case 9:
                Serial.write((byte)1);
                break;
            case 10:
                Serial.write((byte)2);
                break;
            case 11:
                Serial.write((byte)3);
                break;
            default:
                break;
        }
        // write current and voltage data
        Serial.write((byte)(raw_readings[1] & 0xFF));
        Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
        Serial.write((byte)(raw_readings[0] & 0xFF));
        Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
    } else {
        measuring_mode_stopbit();
        delay(5);
        utf8_mode_startbit();
        Serial.println("Error: Unable to get readings.");
        utf8_mode_stopbit();
    }

    if (fsm.check_pwm_bound(255)) {
        if (fsm.check_readings_bound(readings_per_pwm)) {
            fsm.increment_readings();
        } else {
            fsm.increment_pwm();
            fsm.reset_readings();
            analogWrite(analog_port, fsm.get_pwm_state());
        }
    }
}

/**
 * This function is used to send the utf8 communication flag to the serial monitor
 */
void utf8_mode_startbit() {
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFA);
    Serial.write(0xFA);
}

/**
 * This function is used to send the utf8 communication flag to the serial monitor
 */
void utf8_mode_stopbit() {
    // first write the measuring_mode_startbit that is sent when you do serial.print()
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFB);
    Serial.write(0xFB);
    Serial.write(0x0A);
}

/**
 * This function is used to send the measuring_mode_startbit to the serial monitor
 */
void measuring_mode_startbit() {
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xF1);
    Serial.write(0xF1);
}

/**
 * This function is used to send the measuring_mode_stopbit to the serial monitor
 */
void measuring_mode_stopbit() {
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xF0);
    Serial.write(0xF0);
}

/**
 * This function is used to send the exit_serial_signal to the serial monitor
 */
void exit_serial_signal() {
    utf8_mode_startbit();
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0x0A);
}

/**
 * This function is used to send the deltatime in microseconds in utf8 mode
 */
void send_deltatime(unsigned long deltatime) {
    utf8_mode_startbit();
    Serial.print("Deltatime: ");
    Serial.println(deltatime);
    utf8_mode_stopbit();
}

/**
 * This function is used to read data from the desired ina219 module
 *
 * @param slave_address Address of the ina219 module
 * @param analog_port Port the pwm is connected to
 * @param n_readings Number of readings to take from start to finish
 */
void read_motor_x(uint8_t slave_address, uint8_t analog_port, uint16_t n_readings) {
    uint8_t n_per_pwm = 20;
    unsigned long deltatime{};
    delay(10);
    measuring_mode_startbit();
    deltatime = micros();
    rw_data_n_times(read_ina219_data, slave_address, n_per_pwm, 0, 255, analog_port);
    analogWrite(analog_port, 255);
    rw_data_n_times(read_ina219_data, slave_address, n_readings - n_per_pwm * 256);
    deltatime = micros() - deltatime;
    measuring_mode_stopbit();

    send_deltatime(deltatime);

    // Send the deltatime in microseconds in utf8 mode
}