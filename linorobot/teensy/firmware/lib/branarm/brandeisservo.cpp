#include "brandeisservo.h"

BrandeisServo::BrandeisServo() {
  current_time_ms = 0;
  target_angle = 0;
  start_time_ms = 0;
  elapsed_time_ms = 0;
  current_angle = 0;
  start_angle = 0;
  change_in_value_angle = 0;
  moving = false;
}

void BrandeisServo::set_variables(double curr_angle, double tar_angle,
                                  long dur_ms, double start_ms) {
  target_angle = tar_angle;
  duration_ms = dur_ms;
  start_time_ms = start_ms;
  current_angle = curr_angle;
  start_angle = curr_angle;
  change_in_value_angle = target_angle - start_angle;
  moving = true;
}

double BrandeisServo::loop_update_current_ms(long curtime) {
  current_time_ms = curtime;
  elapsed_time_ms = current_time_ms - start_time_ms;
  current_angle = smooth_step();
  // char buffer[100];
  // sprintf(buffer, "tr %.1f %.1f %.1f %.1f %.1f %.1f -> %.1f", start_time_ms,
  //         current_time_ms, duration_ms, start_angle, target_angle,
  //         change_in_value_angle, current_angle);
  // node_handle->loginfo(buffer);
  if (elapsed_time_ms >= duration_ms || current_angle == target_angle) {
    moving = false;
    current_angle = target_angle;
  }
  return (current_angle);
}

double BrandeisServo::smooth_step() {
  double time_increment = static_cast<double>(current_time_ms - start_time_ms) / duration_ms;
  double smooth_amount =
      (time_increment) * (time_increment) * (3 - 2 * (time_increment));
  double new_angle =
      (target_angle * smooth_amount) + start_angle * (1 - smooth_amount);
  // char buffer[300];
  // sprintf(buffer,
  //         "curtime: %lu, starttime: %lu, dur: %lu, time_inc: %.1f, smooth_amount: "
  //         "%.1f, new_angle: %.1f",
  //         current_time_ms, start_time_ms, duration_ms, time_increment, smooth_amount,
  //         new_angle);
  // node_handle->loginfo(buffer);
  return new_angle;
}

void BrandeisServo::setup(ros::NodeHandle &nh) { node_handle = &nh; }
