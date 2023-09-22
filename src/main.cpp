// INA219 sensor
#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "fsm_cls.h"
#include "ina219.h"
#include "ina_functions.h"
#include "timer.h"

// Variables
char in_byte{};
bool state{false};
uint16_t *raw_readings;  // Array to store the raw current and voltage readings

bool pwm3_state{false};
bool pwm9_state{false};
bool pwm10_state{false};
bool pwm11_state{false};

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
    utf8_mode_startbit();
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
    utf8_mode_stopbit();
}

void loop() {
    while (Serial.available() <= 0) {
        delay(10);
    }

    while (Serial.available() > 0) {
        in_byte = Serial.read();
    }

    // Keyboard character ~
    if (in_byte == 0x7E)
        exit_serial_signal();

    // Keyboard character !
    if (in_byte == 0x21) {
        read_motor_x(INA219_PWM3, PWM3, N_READINGS);
        analogWrite(PWM3, 0);
        // pwm3_state = !pwm3_state;
        // if (pwm3_state)
        //     analogWrite(PWM3, 255);
        // else
        //     analogWrite(PWM3, 0);

        // Keyboard character "
    } else if (in_byte == 0x22) {
        read_motor_x(INA219_PWM9, PWM9, N_READINGS);
        analogWrite(PWM9, 0);
        // pwm9_state = !pwm9_state;
        // if (pwm9_state)
        //     analogWrite(PWM9, 255);
        // else
        //     analogWrite(PWM9, 0);

        // Keyboard character #
    } else if (in_byte == 0x23) {
        read_motor_x(INA219_PWM10, PWM10, N_READINGS);
        analogWrite(PWM10, 0);
        // pwm10_state = !pwm10_state;
        // if (pwm10_state)
        //     analogWrite(PWM10, 255);
        // else
        //     analogWrite(PWM10, 0);

        // Keyboard character $
    } else if (in_byte == 0x24) {
        read_motor_x(INA219_PWM11, PWM11, N_READINGS);
        analogWrite(PWM11, 0);
        // pwm11_state = !pwm11_state;
        // if (pwm11_state)
        //     analogWrite(PWM11, 255);
        // else
        //     analogWrite(PWM11, 0);
    }

    // Keyboard character %
    if (in_byte == 0x25) {
        multi_device_measuring();
    }

    // Keyboard character "<"
    if (in_byte == 0x3C) {
        read_motor_x(INA219_PWM3, PWM3, N_READINGS);
        analogWrite(PWM3, 0);
        read_motor_x(INA219_PWM9, PWM9, N_READINGS);
        analogWrite(PWM9, 0);
        read_motor_x(INA219_PWM10, PWM10, N_READINGS);
        analogWrite(PWM10, 0);
        read_motor_x(INA219_PWM11, PWM11, N_READINGS);
        analogWrite(PWM11, 0);
        exit_serial_signal();

        // Keyboard character ">"
    } else if (in_byte == 0x3E) {
        pwm_delays[pwm_delays_index] = read_two_bytes();

        if (pwm_delays_index <= 3)
            pwm_delays_index++;
        else
            pwm_delays_index = 0xFF;
    }
}