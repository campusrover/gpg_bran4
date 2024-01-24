#ifndef BRANDSERVO_H
#define BRANDSERVO_H
#include "transition.h"


class BrandeisServo {
public:
    float current;
    float destination;
    int start_millis;
    int elapsed;
    int duration_ms;

    float delta() {
        return destination - current;
    }
    void calc_elapsed(int current_millis);
    float update_current();
    Transition tr;
};

#endif