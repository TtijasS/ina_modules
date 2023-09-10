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

    Serial.println("Current multiplicator: ");
    Serial.println(MAX_CURRENT / 32768, 10);
    delay(5000);
}

void loop() {
    while (Serial.available() <= 0) {
        delay(10);
    }

    while (Serial.available() > 0) {
        in_byte = Serial.read();
    }
    Serial.print('>');
    Serial.println(in_byte);

    if (in_byte == 'a') {
        read_motor_11();
    }
}