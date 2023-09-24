#include "ina_functions.h"

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "fsm_cls.h"
#include "ina219.h"
#include "ina260.h"
#include "shared_vars.h"
#include "timer.h"

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
 * This function is used to read data from the desired ina219 module
 *
 * @param slave_address Address of the ina219 module
 * @param analog_port Port the pwm is connected to
 * @param n_readings Number of readings to take from start to finish
 */
void read_device_ntimes(uint8_t slave_address, uint8_t analog_port, uint16_t n_readings) {
    uint8_t n_per_pwm = 20;
    unsigned long deltatime{};
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

/**
 * This is the FSM version of function that is used to read data from the ina219 or ina260 modules n_readings times
 *
 * @param fsm State machine
 * @param delay_ms Delay device startup in ms
 * @param reading_function Function used to read the data (either read_ina219_data or read_ina260_data)
 * @param slave_address Address of the ina219 or ina260 modules
 * @param readings_per_pwm Number of readings to take at each pwm value until max is reached.
 * @param analog_port Port the pwm is connected to
 */
void rw_data_fsm(FsmCls &fsm, uint8_t slave_address, uint8_t analog_port) {
    uint16_t *raw_readings = nullptr;

    // read data
    raw_readings = read_ina219_data(slave_address);

    // Serial.print("Port: ");
    // Serial.print(analog_port, HEX);
    // Serial.print("  PWM: ");
    // Serial.print(fsm.get_pwm_state());
    // Serial.print("  Readings: ");
    // Serial.println(fsm.get_readings_state());

    if (raw_readings != nullptr) {
        // write divice id (array position on the pyhton side)
        // switch (analog_port) {
        //     case 3:
        //         Serial.write((byte)0);
        //         break;
        //     case 9:
        //         Serial.write((byte)1);
        //         break;
        //     case 10:
        //         Serial.write((byte)2);
        //         break;
        //     case 11:
        //         Serial.write((byte)3);
        //         break;
        //     default:
        //         break;
        // }
        // write current and voltage data
        Serial.write((byte)(raw_readings[1] & 0xFF));
        Serial.write((byte)((raw_readings[1] >> 8) & 0xFF));
        Serial.write((byte)(raw_readings[0] & 0xFF));
        Serial.write((byte)((raw_readings[0] >> 8) & 0xFF));
    } else {
        measuring_mode_stopbit();
        delay(5);
        utf8_mode_startbit();
        Serial.print("Error: Unable to get readings from ");
        Serial.println(slave_address, HEX);
        utf8_mode_stopbit();
    }

    fsm.step();
}

void multi_device_measuring(uint16_t n_readings, uint16_t per_pwm) {
    uint16_t i = 0;
    multimeasuring_mode_startbit();

    FsmCls fsm_pwm3(pwm_delays[0], 255, per_pwm, PWM3),
        fsm_pwm9(pwm_delays[1], 255, per_pwm, PWM9),
        fsm_pwm10(pwm_delays[2], 255, per_pwm, PWM10),
        fsm_pwm11(pwm_delays[3], 255, per_pwm, PWM11);

    // Just in case update timers
    fsm_pwm3.update_timer();
    fsm_pwm9.update_timer();
    fsm_pwm10.update_timer();
    fsm_pwm11.update_timer();

    // Prepare deltatime timer
    unsigned long deltatime = micros();

    while (i < n_readings) {
        if (Serial.available() > 0) {
            in_byte = Serial.read();
            // Keyboard character &
            if (in_byte == 0x26) {
                break;

                // Keyboard character !
            } else if (in_byte == 0x21) {
                fsm_pwm3.set_pwm_bound(0);
                fsm_pwm9.set_pwm_bound(0);
                fsm_pwm10.set_pwm_bound(0);
                fsm_pwm11.set_pwm_bound(0);
                fsm_pwm3.set_readings_bound(0);
                fsm_pwm9.set_readings_bound(0);
                fsm_pwm10.set_readings_bound(0);
                fsm_pwm11.set_readings_bound(0);
                // Keyboard character "
            } else if (in_byte == 0x22) {
                fsm_pwm3.set_pwm_bound(255);
                fsm_pwm9.set_pwm_bound(255);
                fsm_pwm10.set_pwm_bound(255);
                fsm_pwm11.set_pwm_bound(255);
                fsm_pwm3.set_readings_bound(20);
                fsm_pwm9.set_readings_bound(20);
                fsm_pwm10.set_readings_bound(20);
                fsm_pwm11.set_readings_bound(20);
                fsm_pwm3.update_timer();
                fsm_pwm9.update_timer();
                fsm_pwm10.update_timer();
                fsm_pwm11.update_timer();
            }
        }

        rw_data_fsm(fsm_pwm3, INA219_PWM3, PWM3);
        rw_data_fsm(fsm_pwm9, INA219_PWM9, PWM9);
        rw_data_fsm(fsm_pwm10, INA219_PWM10, PWM10);
        rw_data_fsm(fsm_pwm11, INA219_PWM11, PWM11);

        // 0 is infinite readings
        i++;
    }
    deltatime = micros() - deltatime;
    analogWrite(PWM3, 0);
    analogWrite(PWM9, 0);
    analogWrite(PWM10, 0);
    analogWrite(PWM11, 0);
    // unsigned long deltatime = deltatime_timer.get_deltatime();
    measuring_mode_stopbit();
    utf8_mode_startbit();
    Serial.println("Finished measuring.");
    Serial.print("Deltatime: ");
    Serial.println(deltatime);
    utf8_mode_stopbit();
}

void update_delays() {
    delays_mode_startbit();
    // Setup timer
    TimerCls timeout;
    // Check for timeout (returns true when timeout is exceeded)
    while (!timeout.stopwatch(40)) {
        // Check if there is enough data available
        if (Serial.available() >= 4) {
            // Read and check the flag byte.
            byte flag = (byte)Serial.read();
            // if flag == * character read the next 3 bytes
            if (flag == 0x2A) {
                // Read the pwm_delays array index
                byte index = (byte)Serial.read();
                // Read the MSB byte of the delay
                byte msb = (byte)Serial.read();
                // Read the LSB byte of the delay
                byte lsb = (byte)Serial.read();
                // Combine bytes to one uint16_t number
                uint16_t combined = (msb << 8) | lsb;

                // Ensure the index is within bounds
                if (index < 4) {
                    // Store the delay in array
                    pwm_delays[index] = combined;
                    // Send the delay back to the serial monitor to confirm
                    Serial.write(msb);
                    Serial.write(lsb);
                    timeout.upd_timer();
                } else {
                    utf8_mode_startbit();
                    Serial.print("Error: Index out of bounds: ");
                    Serial.println(index);
                    utf8_mode_stopbit();
                }
            } else if (flag == 0xFD) {
                return;
            }
        }
    }
    utf8_mode_startbit();
    Serial.println("Error: Timeout.");
    utf8_mode_stopbit();
}

void delays_mode_startbit() {
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFD);
    Serial.write(0xFD);
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
 * This function is used to send the measuring_mode_startbit to the serial monitor
 */
void multimeasuring_mode_startbit() {
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xF4);
    Serial.write(0xF4);
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
