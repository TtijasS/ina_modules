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
uint16_t pwm_delays[4]{};
uint8_t pwm_delays_index{0xFF};

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

    Serial.println("Setup INA219_PWM3");
    setup_ina219(INA219_PWM3, MAX_CURRENT);
    Serial.println();
    Serial.println("Setup INA219_PWM9");
    setup_ina219(INA219_PWM9, MAX_CURRENT);
    Serial.println();
    Serial.println("Setup INA219_PWM10");
    setup_ina219(INA219_PWM10, MAX_CURRENT);
    Serial.println();
    Serial.println("Setup INA219_PWM11");
    setup_ina219(INA219_PWM11, MAX_CURRENT);
    Serial.println();

    Serial.print("Current multiplicator: ");
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

    // Keyboard character "<"
    if (in_byte == 0x3C) {
        read_motor_n(INA219_PWM3, PWM3);
        read_motor_n(INA219_PWM9, PWM9);
        read_motor_n(INA219_PWM10, PWM10);
        read_motor_n(INA219_PWM11, PWM11);

    }  // Keyboard character ">"
    else if (in_byte == 0x3E) {
        pwm_delays[pwm_delays_index] = read_two_bytes();

        if (pwm_delays_index >= 4)
            pwm_delays_index = 0xFF;
        else
            pwm_delays_index++;
    }
}

int read_two_bytes() {
    while (Serial.available() < 2) {  // Wait until two bytes are available
        delay(10);
    }

    byte msb = (byte)Serial.read();  // Read the first byte (MSB)
    byte lsb = (byte)Serial.read();  // Read the second byte (LSB)

    uint16_t combined = (msb << 8) | lsb;  // Shift the MSB left by 8 bits and OR with LSB
    return combined;
}