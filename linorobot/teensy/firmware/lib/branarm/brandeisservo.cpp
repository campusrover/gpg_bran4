#include "brandeisservo.h"

#include "ros/time.h"

BrandeisServo::BrandeisServo() {
  current = 0;
  destination = 0;
  start_millis = 0;
  elapsed = 0;
  tr = Transition();
}

float BrandeisServo::update_current() {
  current = tr.ease_in_out_cubic(elapsed, current, delta(), duration_ms, start_millis, destination);
  return current;
}

void BrandeisServo::calc_elapsed(int current_millis) { elapsed = current_millis - start_millis; }
