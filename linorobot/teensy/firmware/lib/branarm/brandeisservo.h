#ifndef BRANDSERVO_H
#define BRANDSERVO_H
#include "transition.h"


class BrandeisServo {
public:
    BrandeisServo();
    double current;
    double destination;
    double start_millis;
    double elapsed;
    double duration_ms;
    double delta();
    void calc_elapsed(int current_millis);
    double update_current();
    Transition tr;
};

#endif
