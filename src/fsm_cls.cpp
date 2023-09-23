#include "fsm_cls.h"

#include <Arduino.h>

/**
 * This class is used to create a state machine
 */

FsmCls::FsmCls(unsigned long delay_ms, uint16_t pwm_bnd, uint16_t readings_bnd, uint8_t pwm_port_n) {
    pwm_port = pwm_port_n;

    pwm_state = 0;
    readings_state = 0;

    timer = millis();
    my_delay_ms = delay_ms;

    pwm_bound = pwm_bnd;
    readings_bound = readings_bnd;
}

uint16_t FsmCls::get_pwm_state() {
    return pwm_state;
}

uint16_t FsmCls::get_readings_state() {
    return readings_state;
}

/**
 * This function is used to update the timer to current millis()
 */
void FsmCls::update_timer() {
    timer = millis();
}

/**
 * This function is used to set delay_start to delay_ms
 */
void FsmCls::set_delay(unsigned long delay_ms) {
    my_delay_ms = delay_ms;
    update_timer();
}

/**
 * This function is used to set the pwm_bound
 */
void FsmCls::set_pwm_bound(uint16_t bound) {
    if (bound > 255)
        bound = 255;

    if (bound < pwm_state) {
        pwm_state = bound;
        analogWrite(pwm_port, pwm_state);
    }
    pwm_bound = bound;
}

void FsmCls::set_readings_bound(uint16_t bound) {
    if (bound < readings_state)
        readings_state = 0;

    readings_bound = bound;
}

bool FsmCls::check_delay() {
    return millis() - timer > my_delay_ms;
}

/**
 * This function is used to increment states.
 * One step increments readings_state.
 * If readings_bound is reached, pwm_state is incremented until pwm_bound is reached.
 */
void FsmCls::step() {
    if (check_delay()) {
        if (readings_state < readings_bound) {
            readings_state++;
        } else {
            readings_state = 0;
            if (pwm_state < pwm_bound) {
                pwm_state++;
                analogWrite(pwm_port, pwm_state);
            }
        }
    }
}
