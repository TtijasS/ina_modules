#include "timer.h"

#include <Arduino.h>

Timercls::Timercls() {
    /**
     * This class is used to create a timer
     */
    timer = millis();
}

bool Timercls::stopwatch(unsigned long delay_millis) {
    /**
     * This function is used to create a stopwatch
     *
     * @param delay_millis Time in milliseconds
     * @return True if the time has passed
     */
    if (millis() - timer > delay_millis) {
        timer = millis();
        return true;
    }
    return false;
}