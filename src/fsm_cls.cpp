#include "fsm_cls.h"

#include <Arduino.h>

/**
 * This class is used to create a state machine
 */

FsmCls::FsmCls() {
    pwm_state = 0;
    readings_state = 0;
}

/**
 * This function is used to set pwm_state to 0
 */
void FsmCls::reset_pwm() {
    pwm_state = 0;
}

/**
 * This function is used to set readings_state to 0
 */
void FsmCls::reset_readings() {
    readings_state = 0;
}

/**
 * This function is used to increment the pwm_state
 */
void FsmCls::increment_pwm() {
    pwm_state++;
}

/**
 * This function is used to increment the readings_state
 */
void FsmCls::increment_readings() {
    readings_state++;
}

/**
 * This function is used to retreive the pwm_state
 */
uint16_t FsmCls::get_pwm_state() {
    return pwm_state;
}

/**
 * This function is used to retreive the readings_state
 */
uint16_t FsmCls::get_readings_state() {
    return readings_state;
}

/**
 * This function is used to check if the pwm_state is less than the bound
 */
bool FsmCls::check_pwm_bound(uint16_t bound) {
    return pwm_state < bound;
}

/**
 * This function is used to check if the readings_state is less than the bound
 */
bool FsmCls::check_readings_bound(uint16_t bound) {
    return readings_state < bound;
}
