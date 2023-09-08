// INA219 sensor
#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "ina219.h"
#include "ina_functions.h"
#include "timer.h"

// Variables
char in_byte{};
bool state{false};
uint16_t *raw_readings;  // Array to store the raw current and voltage readings

void setup() {
    TCCR1A = 0b00000001;  // 8bit
    TCCR1B = 0b00001001;  // x1 fast pwm
    TCCR2B = 0b00000001;  // x1
    TCCR2A = 0b00000011;  // fast pwm

    Wire.begin();
    // wire max frequency
    TWBR = 2;  // 800kHz

    Serial.begin(115200);

    while (!Serial) {
        delay(10);
    }

    set_ina219_mode(INA219_ADDRESS);

    Serial.print("Config register: ");
    Serial.println(request_reg_data(INA219_ADDRESS, 0x00), BIN);
    Serial.print("Calibration register: ");
    Serial.println(request_reg_data(INA219_ADDRESS, 0x05), BIN);

    Serial.println("Setting calibration register");
    set_ina219_calibration_register(MAX_CURRENT, INA219_ADDRESS);
    Serial.println("Calibration register set to 1A");

    Serial.print("Re-check calibration register: ");
    Serial.println(request_reg_data(INA219_ADDRESS, 0x05), BIN);
}

Timercls timer1;
Timercls timer2;
Timercls timer3;

void loop() {
    // int i{0};
    // wait for the serial port to connect

    // while (Serial.available() > 0) {
    //     in_byte = Serial.read();
    //     Serial.print(">");
    //     Serial.println(in_byte);
    // }
    // if (in_byte == 'a') {
    //     analogWrite(11, 255);
    // } else if (in_byte == 'b') {
    //     analogWrite(11, 0);
    // }
    if (timer1.stopwatch(1000))
        Serial.println("1");
    if (timer2.stopwatch(2000))
        Serial.println("	2");
    if (timer3.stopwatch(3000))
        Serial.println("		3");
    // while (i < 3101) {
    //     if (i == 100) {
    //         analogWrite(11, 255);
    //     } else if (i == 2100) {
    //         analogWrite(11, 0);
    //     } else if (micros() - timer2 > 150) {
    //         uint16_t *raw_readings = read_ina219_i_v(INA219_ADDRESS);
    //         timer2 = micros();
    //         if (i == 0)
    //             data_array[i] = raw_readings[1];  // voltage
    // 			Serial.write(raw_readings[1]);
    //         else
    //             data_array[i] = raw_readings[0];  // current
    // 			Serial.write(raw_readings[0]);
    //         ++i;
    //     }
    // }

    // for (int i : data_array) {
    //     Serial.print(i);
    //     Serial.print(", ");
    // }

    // Serial.println();
    // Serial.println("Done");
    // // stop the program here
    // exit(0);

    // iterate through elements of data_array

    // if (timer_function(10, timer2)) {
    //     // 1011 11101100
    //     // Serial.print(raw_readings[1], BIN);
    //     // Serial.print(" ");
    //     // Serial.print((raw_readings[1] >> 8) & 0xFF, BIN);
    //     // Serial.print(" ");
    //     // Serial.println((raw_readings[1]) & 0xFF, BIN);
    //     // Serial.println(raw_readings[1] * 0.004);
    // }
    // Serial.print(static_cast<int16_t>(raw_readings[0]) * CURRENT_LSB * 10);
}