#include "timer.h"

#include <Arduino.h>

/**
 * This class is used to create a timer
 */
Timercls::Timercls() {
    timer = millis();
}

/**
 * This function is used to create a stopwatch
 *
 * @param delay_millis Time in milliseconds
 * @return True if the time has passed
 */
bool Timercls::stopwatch(unsigned long delay_millis) {
    if (millis() - timer > delay_millis) {
        timer = millis();
        return true;
    }
    return false;
}