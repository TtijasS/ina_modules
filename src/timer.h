#ifndef TIMER_H
#define TIMER_H

class TimerCls {
   public:
    TimerCls();
    void upd_timer();
    unsigned long get_deltatime();
    bool stopwatch(unsigned long delay_ms);
    bool stopwatch_no_reset(unsigned long delay_ms);

   private:
    unsigned long timer{};
    unsigned long my_delay_ms{};
};

#endif