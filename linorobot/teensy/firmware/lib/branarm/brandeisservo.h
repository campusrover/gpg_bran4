#ifndef BRANDEOSSERVO_H
#define BRANDEOSSERVO_H
#include "transition.h"


class BrandeisServo {
public:
    BrandeisServo();
    double current;
    double destination;
    double start_time_ms;
    double elapsed_time_ms;
    double duration_ms;
    double delta();
    void update_elapsed_time(int current_millis);
    double upadate_current_time();
    void set_variables(double destination, double duration_ms, double start_time_ms) : destination(destination), duration_ms(duration_ms), start_time_ms(start_time_ms);


    Transition tr;
};

#endif
