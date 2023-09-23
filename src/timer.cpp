#include "timer.h"

#include <Arduino.h>

/**
 * This class is used to create a timer
 *
 * @param delay_ms Time in milliseconds that can be used to keep a local delay (my_delay_ms)
 */
TimerCls::TimerCls() {
    timer = millis();
}

void TimerCls::upd_timer() {
	timer = millis();
}

unsigned long TimerCls::get_deltatime() {
    return millis() - timer;
}

/**
 * This function is used to create a stopwatch that resets when true is returned
 *
 * @param delay_millis Time in milliseconds
 * @return True if the time has passed
 */
bool TimerCls::stopwatch(unsigned long delay_ms) {
    if (millis() - timer > delay_ms) {
        timer = millis();
        return true;
    }
    return false;
}

/**
 * This function is used to create a stopwatch that does not reset when true is returned
 *
 * @param delay_millis Time in milliseconds
 */
bool TimerCls::stopwatch_no_reset(unsigned long delay_ms) {
    if (millis() - timer > delay_ms) {
        return true;
    }
    return false;
}