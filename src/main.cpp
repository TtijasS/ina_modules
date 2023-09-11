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

    Serial.print("Setting config reg");
    set_ina219_mode(INA219_ADDRESS);
    Serial.println("Setting calibration reg");
    set_ina219_calibration_register(MAX_CURRENT, INA219_ADDRESS);
    Serial.print("Calibration register set to ");
    Serial.print(MAX_CURRENT);
    Serial.println("mA");

    Serial.println("Config register: ");
    Serial.println(request_ina_reg_data(INA219_ADDRESS, 0x00), BIN);
    Serial.print("Calibration register: ");
    Serial.println(request_ina_reg_data(INA219_ADDRESS, 0x05), BIN);
    Serial.println("Current multiplicator: ");
    Serial.println(MAX_CURRENT / 32768, 10);
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