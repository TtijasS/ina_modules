#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

class FsmCls {
   public:
    FsmCls(unsigned long delay_ms, uint16_t pwm_bnd, uint16_t readings_bnd, uint8_t pwm_port_n);

    uint16_t get_pwm_state();
    uint16_t get_readings_state();

    bool check_delay();

    void set_delay(unsigned long delay_ms);
    void set_pwm_bound(uint16_t bound);
    void set_readings_bound(uint16_t bound);
    void update_timer();
    void step();

   private:
    uint8_t pwm_port{};
    uint16_t pwm_state{};
    uint16_t pwm_bound{};
    uint16_t readings_state{};
    uint16_t readings_bound{};
    unsigned long timer{};
    unsigned long my_delay_ms{};
};

#endif