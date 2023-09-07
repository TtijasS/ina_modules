#ifndef TIMER_H
#define TIMER_H

class Timercls {
   public:
    Timercls();
    bool stopwatch(unsigned long delay_millis);

   private:
    unsigned long timer{};
};

#endif