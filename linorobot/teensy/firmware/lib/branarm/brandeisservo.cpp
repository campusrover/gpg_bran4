#include "brandeisservo.h"

BrandeisServo::BrandeisServo() {
  target_angle = 0;
  start_time_ms = 0;
  current_angle = 0;
  start_angle = 0;
  change_in_value_angle = 0;
  moving = false;
  test_mode = true;
  counter = 0;
  error_counter = 0;
}
void BrandeisServo::setup(ros::NodeHandle &nh, Adafruit_PWMServoDriver ser_drr,
                          int port, long max_ang, long min_ang,
                          double conv_scale, double conv_offs) {

  node_handle = &nh;
  servo_driver = ser_drr;
  port_num = port;

  max_angle = max_ang;
  min_angle = min_ang;
  convert_scale = conv_scale;
  convert_offset = conv_offs;
}

void BrandeisServo::setup_ease(double tar_angle, long curr_time_ms,
                               long dur_ms) {
  start_angle = current_angle;
  start_time_ms = curr_time_ms;
  change_in_value_angle = tar_angle - start_angle;
  moving = change_in_value_angle != 0;
  duration_ms = dur_ms;
}

double BrandeisServo::compute_next_increment(long current_time_ms) {
  double time_increment =
      static_cast<double>(current_time_ms - start_time_ms) / duration_ms;
  double ease_amount = quad_equation(time_increment);
  double new_angle =
      (target_angle * ease_amount) + start_angle * (1 - ease_amount);
  return new_angle;
}

void BrandeisServo::move(double deg) {
  if (deg < min_angle || deg > max_angle) {
    error_counter++;
  } else {
    moving = true;
    current_angle = deg;
    counter++;
    int deglen = (deg + convert_offset) * convert_scale;
    if (!test_mode) {
      servo_driver.setPWM(port_num, 0, deglen);
    }
  }
}

void BrandeisServo::status(char buffer[200]) {
  sprintf(buffer, "%d %f", port_num, current_angle);
}

double BrandeisServo::quad_equation(long time_increment) {
  double term = -2 * time_increment + 2;
  double ease_amount = time_increment < 0.5
                           ? 2 * time_increment * time_increment
                           : 1 - (term * term) / 2;
  return ease_amount;
}

double BrandeisServo::smooth_step_equation(long time_increment) {
  double smooth_amount =
      (time_increment) * (time_increment) * (3 - 2 * (time_increment));
  double ease_amount =
      (target_angle * smooth_amount) + start_angle * (1 - smooth_amount);
  return ease_amount;
}
