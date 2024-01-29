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

void BrandeisServo::setup(ros::NodeHandle &nh) { node_handle = &nh; }

void BrandeisServo::setup_ease(double tar_angle, long curr_time_ms, long dur_ms) {
  start_angle = current_angle;
  start_time_ms = curr_time_ms;
  change_in_value_angle = tar_angle - start_angle;
  moving = change_in_value_angle != 0;
}

double BrandeisServo::compute_next_increment() {
  double time_increment =
      static_cast<double>(current_time_ms - start_time_ms) / duration_ms;
  double ease_amount = ease_in_out_quad_equation(time_increment)
  double new_angle =
      (target_angle * ease_amount) + start_angle * (1 - ease_amount);
  return new_angle;
  

double BrandeisServo::ease_in_out_quad_equation(long time_increment) {
  double term = -2 * time_increment + 2;
  double ease_amount = time_increment < 0.5
                             ? 2 * time_increment * time_increment
                             : 1 - (term * term) / 2;
  return ease_amount;
}

