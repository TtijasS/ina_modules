#include "timer.h"

#include <Arduino.h>

Timercls::Timercls() {
    timer = millis();
}

bool Timercls::stopwatch(unsigned long delay_millis) {
    if (millis() - timer > delay_millis) {
        timer = millis();
        return true;
    }
    return false;
}