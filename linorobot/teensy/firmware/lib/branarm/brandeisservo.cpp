#include "brandeisservo.h"
#include "branutils.h"

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
void BrandeisServo::setup(int id, ros::NodeHandle &nh,
                          Adafruit_PWMServoDriver ser_drr, int pin,
                          long max_ang, long min_ang, double conv_offs,
                          double conv_scale, long park_ang) {
  ident = id;
  node_handle = &nh;
  servo_driver = ser_drr;
  pin_number = pin;
  max_angle = max_ang;
  min_angle = min_ang;
  current_angle = park_ang;
  convert_scale = conv_scale;
  convert_offset = conv_offs;
  move(park_ang);
}

void BrandeisServo::setup_ease(double tar_angle, long dur) {

  start_time_ms = millis();
  start_angle = current_angle;
  duration_ms = max(dur, 1000); // 1 second minimm
  change_in_value_angle = tar_angle - start_angle;
  moving = change_in_value_angle != 0;
  target_angle = tar_angle;
  LOG_INFO("setup_ease [%d] %.1f %d %.1f %.1f %ld", ident, target_angle, moving,
           change_in_value_angle, start_angle, duration_ms);
}

double BrandeisServo::compute_next_increment(long current_time_ms) {
  double time_increment =
      static_cast<double>(current_time_ms - start_time_ms) / duration_ms;
  double ease_amount = quad_equation(time_increment);
  double new_angle =
      (target_angle * ease_amount) + start_angle * (1 - ease_amount);

  LOG_INFO("compute_next_increment [%d] %.1f %.1f %.1f %.1f %.1f", ident,
           time_increment, ease_amount, start_angle,
           target_angle, new_angle);

  moving = time_increment <= 1.0;
  return new_angle;
}

void BrandeisServo::move(double deg) {
  if (deg < min_angle || deg > max_angle) {
    error_counter++;
    LOG_ERROR("**** Invalid Move: [%d], %ld %ld %.3f", ident, min_angle,
             max_angle, deg);
  } else {
    current_angle = deg;
    counter++;
    int deglen = (deg + convert_offset) * convert_scale;
    if (!test_mode) {
      servo_driver.setPWM(pin_number, 0, deglen);
    }
    LOG_INFO("servo setPWM command: [%d] deg: %.1f pulse: %d", ident, deg, deglen);
  }
}

// Basic status of the servo. Name, current position, is it moving, and counters
void BrandeisServo::status() {
  LOG_INFO(buffer, "![%d] %f %d %d!", ident, current_angle, counter,
          error_counter);
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
