#ifndef TIMER_H
#define TIMER_H

class TimerCls {
   public:
    TimerCls();
    bool stopwatch(unsigned long delay_millis);
    bool stopwatch_no_reset(unsigned long delay_millis);

   private:
    unsigned long timer{};
};

#endif