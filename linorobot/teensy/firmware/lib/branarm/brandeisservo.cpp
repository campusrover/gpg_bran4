#include "brandeisservo.h"

BrandeisServo::BrandeisServo() {
  current_time_ms = 0;
  target_angle = 0;
  start_time_ms = 0;
  elapsed_time_ms = 0;
  current_angle = 0;
  start_angle = 0;
  tr = Transition();
  change_in_value_angle = 0;
}

void BrandeisServo::set_variables(double curr_angle, double tar_angle,
                                  double dur_ms, double start_ms) {
  target_angle = tar_angle;
  duration_ms = dur_ms;
  start_time_ms = start_ms;
  current_angle = curr_angle;
  change_in_value_angle = target_angle - start_angle;
}

double BrandeisServo::loop_update_current_ms(int current_millis) {
  current_time_ms = current_millis;
  elapsed_time_ms = current_time_ms - start_time_ms;
  current_angle = no_easing();
  // char buffer[100];
  // sprintf(buffer, "tr %.1f %.1f %.1f %.1f %.1f %.1f -> %.1f", start_time_ms,
  //         current_time_ms, duration_ms, start_angle, target_angle,
  //         change_in_value_angle, current_angle);
  // node_handle->loginfo(buffer);
  return (current_angle);
}

void BrandeisServo::setup(ros::NodeHandle &nh) { node_handle = &nh; }

double BrandeisServo::no_easing() {
  return change_in_value_angle * ((current_time_ms-start_time_ms) / duration_ms) + start_angle;
}
