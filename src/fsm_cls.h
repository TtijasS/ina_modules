#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

class FsmCls {
   public:
    FsmCls();
    void reset_pwm();
    void reset_readings();
    uint16_t get_pwm_state();
    uint16_t get_readings_state();
    void increment_readings();
    void increment_pwm();
    bool check_pwm_bound(uint16_t bound);
    bool check_readings_bound(uint16_t bound);

   private:
    uint16_t pwm_state{};
    uint16_t readings_state{};
};

#endif