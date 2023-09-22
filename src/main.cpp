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
uint16_t pwm_delays[4]{};
uint8_t pwm_delays_index{0xFF};
uint16_t readings_n{14000};
uint8_t read_per_pwm{2};

FsmCls fsm_pwm3;
FsmCls fsm_pwm9;
FsmCls fsm_pwm10;
FsmCls fsm_pwm11;

bool pwm3_state{false};
bool pwm9_state{false};
bool pwm10_state{false};
bool pwm11_state{false};

// Function declarations
int read_two_bytes();

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

    rw_data_fsm(fsm_pwm3, read_ina219_data, INA219_PWM3, read_per_pwm, PWM3);
    rw_data_fsm(fsm_pwm9, read_ina219_data, INA219_PWM9, read_per_pwm, PWM9);
    rw_data_fsm(fsm_pwm10, read_ina219_data, INA219_PWM10, read_per_pwm, PWM10);
    rw_data_fsm(fsm_pwm11, read_ina219_data, INA219_PWM11, read_per_pwm, PWM11);
    delay(10);
    // character ~ (0x7E)
    // if (in_byte == 0x7E)
    //     exit_serial_signal();
    // if (in_byte == 0x21) {
    //     read_motor_x(INA219_PWM3, PWM3, readings_n);
    //     analogWrite(PWM3, 0);
    //     // pwm3_state = !pwm3_state;
    //     // if (pwm3_state)
    //     //     analogWrite(PWM3, 255);
    //     // else
    //     //     analogWrite(PWM3, 0);
    // } else if (in_byte == 0x22) {
    //     read_motor_x(INA219_PWM9, PWM9, readings_n);
    //     analogWrite(PWM9, 0);
    //     // pwm9_state = !pwm9_state;
    //     // if (pwm9_state)
    //     //     analogWrite(PWM9, 255);
    //     // else
    //     //     analogWrite(PWM9, 0);
    // } else if (in_byte == 0x23) {
    //     read_motor_x(INA219_PWM10, PWM10, readings_n);
    //     analogWrite(PWM10, 0);
    //     // pwm10_state = !pwm10_state;
    //     // if (pwm10_state)
    //     //     analogWrite(PWM10, 255);
    //     // else
    //     //     analogWrite(PWM10, 0);
    // } else if (in_byte == 0x24) {
    //     read_motor_x(INA219_PWM11, PWM11, readings_n);
    //     analogWrite(PWM11, 0);
    //     // pwm11_state = !pwm11_state;
    //     // if (pwm11_state)
    //     //     analogWrite(PWM11, 255);
    //     // else
    //     //     analogWrite(PWM11, 0);
    // }

    // // Keyboard character "<"
    // if (in_byte == 0x3C) {
    //     read_motor_x(INA219_PWM3, PWM3, readings_n);
    //     analogWrite(PWM3, 0);
    //     read_motor_x(INA219_PWM9, PWM9, readings_n);
    //     analogWrite(PWM9, 0);
    //     read_motor_x(INA219_PWM10, PWM10, readings_n);
    //     analogWrite(PWM10, 0);
    //     read_motor_x(INA219_PWM11, PWM11, readings_n);
    //     analogWrite(PWM11, 0);
    //     exit_serial_signal();

    // }  // Keyboard character ">"
    // else if (in_byte == 0x3E) {
    //     pwm_delays[pwm_delays_index] = read_two_bytes();

    //     if (pwm_delays_index <= 3)
    //         pwm_delays_index++;
    //     else
    //         pwm_delays_index = 0xFF;
    // }
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