#include "brandeisservo.h"

BrandeisServo::BrandeisServo()
{
    current = 0;
    destination = 0;
    start_time_ms = 0;
    elapsed_time_ms = 0;
    tr = Transition();
}

double BrandeisServo::upadate_current_time()
{
    current = (float)tr.ease_in_out_cubic(elapsed_time_ms, current, delta(), duration_ms, start_time_ms, destination);
    return current;
}

void BrandeisServo::update_elapsed_time(int current_millis)
{
    elapsed_time_ms = current_millis - start_time_ms;
}

double BrandeisServo::delta()
{
    return destination - current;
}
