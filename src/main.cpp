// INA219 sensor
#include <Arduino.h>
#include <Wire.h>

#include "timer.h"

const uint8_t INA219_ADDRESS = 0x40;       // INA260 I2C address
const double SHUNT_VAL = 0.002;            // 2mOhm
const double BUS_VOLTAGE_LSB = 0.004;      // 4mV
const unsigned int REGISTER_RANGE{32768};  // 15 bit
const double MAX_CURRENT{500.0};           // 1000mA
double CURRENT_LSB = MAX_CURRENT / REGISTER_RANGE;
char in_byte{};

// Variables
bool state{false};
uint16_t *raw_readings;  // Array to store the raw current and voltage readings

// Functions
bool timer_function(unsigned long &, unsigned long);
uint16_t *read_ina219_i_v(uint8_t);
uint16_t request_reg_data(uint8_t, uint8_t);
void set_ina219_calibration_register(float);
void set_ina219_mode();

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

    set_ina219_mode();

    Serial.print("Config register: ");
    Serial.println(request_reg_data(INA219_ADDRESS, 0x00), BIN);
    Serial.print("Calibration register: ");
    Serial.println(request_reg_data(INA219_ADDRESS, 0x05), BIN);

    Serial.println("Setting calibration register");
    set_ina219_calibration_register(MAX_CURRENT);
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

// put function definitions here:

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

    // Wire.beginTransmission((uint8_t)slave_address);
    // Wire.write(0x03);  // Register address for reading voltage
    // Wire.endTransmission();

    // Wire.requestFrom((uint8_t)slave_address, (uint8_t)2);  // Read 2 bytes (1 registers)
    // if (Wire.available() >= 2) {
    // 	raw_readings[2] = (Wire.read() << 8) | Wire.read();  // Voltage
    // } else {
    // 	Serial.println("Error: Unable to read data from INA260");
    // }

    return raw_readings;
}

uint16_t request_reg_data(uint8_t slave_address, uint8_t register_address) {
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

// page 19 https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1692784130490&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-eu%2526utm_content%253DINA219%2526ds_k%253DINA219%2BDatasheet%2526DCM%253Dyes%2526gclid%253DCj0KCQjw3JanBhCPARIsAJpXTx7DPjhosIxfe6pl48GAchbIMAvx_KeAZJ5H3Dv8KqVLjmxZV7nUCesaAm1BEALw_wcB%2526gclsrc%253Daw.ds
void set_ina219_mode() {
    Serial.println("Setting INA219 mode");
    // default 111001 10011111
    //     new 111000 10001111
    // Prepare config register settings
    uint8_t high_byte = 0b00111000;
    uint8_t low_byte = 0b10001111;

    // Begin transmission with selected slave module
    Wire.beginTransmission(INA219_ADDRESS);

    // select config register
    Wire.write(0x00);

    // write new config
    Wire.write(high_byte);
    Wire.write(low_byte);

    // end transmission with selected slave module
    Wire.endTransmission();
}

// pg 12 https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1692784130490&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FINA219%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-eu%2526utm_content%253DINA219%2526ds_k%253DINA219%2BDatasheet%2526DCM%253Dyes%2526gclid%253DCj0KCQjw3JanBhCPARIsAJpXTx7DPjhosIxfe6pl48GAchbIMAvx_KeAZJ5H3Dv8KqVLjmxZV7nUCesaAm1BEALw_wcB%2526gclsrc%253Daw.ds
void set_ina219_calibration_register(float MAX_CURRENT) {
    // 1A max -> CURRENT_LSB = 30.517uA
    // cal_reg = 671.1 (671 truncated)
    // new cal_reg value binary is then 1010011110

    // pow(2,15) equals (1 << 15) shift
    // unsigned int register_range{1 << 15};

    double CURRENT_LSB = MAX_CURRENT / REGISTER_RANGE;  // gives 0.000030517578125
    double calibration_value = 0.04096 / (CURRENT_LSB * SHUNT_VAL);
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
    Wire.beginTransmission(INA219_ADDRESS);
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