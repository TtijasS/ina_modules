#include "timer.h"

#include <Arduino.h>

/**
 * This class is used to create a timer
 */
TimerCls::TimerCls() {
    timer = millis();
}

/**
 * This function is used to create a stopwatch that resets when true is returned
 *
 * @param delay_millis Time in milliseconds
 * @return True if the time has passed
 */
bool TimerCls::stopwatch(unsigned long delay_millis) {
    if (millis() - timer > delay_millis) {
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
bool TimerCls::stopwatch_no_reset(unsigned long delay_millis) {
    if (millis() - timer > delay_millis) {
        return true;
    }
    return false;
}