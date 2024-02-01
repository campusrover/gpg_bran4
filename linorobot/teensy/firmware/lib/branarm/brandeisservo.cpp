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
void BrandeisServo::setup(int id, ros::NodeHandle &nh, Adafruit_PWMServoDriver ser_drr,
                          int pin, long max_ang, long min_ang,
                          double conv_offs, double conv_scale, long park_ang) {
  id = id;
  node_handle = &nh;
  servo_driver = ser_drr;
  pin_number = pin;
  max_angle = max_ang;
  min_angle = min_ang;
  convert_scale = conv_scale;
  convert_offset = conv_offs;
  
  int park_pulse = (park_ang + conv_offs) * conv_scale;
  servo_driver.setPWM(pin_number, 0, park_pulse);
  LOG_INFO("setup [%d] %d %ld %ld %.1f %.1f", id, pin, max_ang, min_ang, conv_scale, convert_offset);
}

void BrandeisServo::setup_ease(double tar_angle) {
  start_time_ms = millis();
  start_angle = current_angle;
  change_in_value_angle = tar_angle - start_angle;

  // trying 1 second for every 30 degrees
  duration_ms = abs((change_in_value_angle * 1000) / 15);
  moving = change_in_value_angle != 0;
  target_angle = tar_angle;
  LOG_INFO("setup_ease [%d] %f %d %f %f", id, target_angle, moving, change_in_value_angle, start_angle);
}

double BrandeisServo::compute_next_increment(long current_time_ms) {
  double time_increment =
      static_cast<double>(current_time_ms - start_time_ms) / duration_ms;
  double ease_amount = quad_equation(time_increment);
  double new_angle =
      (target_angle * ease_amount) + start_angle * (1 - ease_amount);
  
  LOG_INFO("compute_next_increment [%d] %ld %.1f %.1f %.1f %.1f %.1f", id, current_time_ms, time_increment, ease_amount, start_angle, target_angle, new_angle);
  
  moving = time_increment <= 1.0;
  return new_angle;
}

void BrandeisServo::move(double deg) {
  if (deg < min_angle || deg > max_angle) {
    error_counter++;
  } else {
    current_angle = deg;
    counter++;
    int deglen = (deg + convert_offset) * convert_scale;
    if (!test_mode) {
      servo_driver.setPWM(pin_number, 0, deglen);
    }
    LOG_INFO("move: [%d] %.1f %d", id, deg, deglen);
  }
}

// Basic status of the servo. Name, current position, is it moving, and counters
void BrandeisServo::status(char buffer[200]) {
  sprintf(buffer, "[%d] %f %d %d", id, current_angle, counter, error_counter);
}

double BrandeisServo::quad_equation(double time_increment) {
  double term = -2 * time_increment + 2;
  double ease_amount = time_increment < 0.5
                           ? 2 * time_increment * time_increment
                           : 1 - (term * term) / 2;
  return ease_amount;
}

double BrandeisServo::smooth_step_equation(double time_increment) {
  double smooth_amount =
      (time_increment) * (time_increment) * (3 - 2 * (time_increment));
  double ease_amount =
      (target_angle * smooth_amount) + start_angle * (1 - smooth_amount);
  return ease_amount;
}
