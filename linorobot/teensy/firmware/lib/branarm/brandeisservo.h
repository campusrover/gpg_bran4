#ifndef BRANDEOSSERVO_H
#define BRANDEOSSERVO_H
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>


class BrandeisServo {
public:
  int ident;
  ros::NodeHandle *node_handle;
  bool moving;
  bool test_mode;
  int counter;
  int error_counter;
  double current_angle;
  char buffer[300];
  void status();

  BrandeisServo();

  void setup(int serv_num, ros::NodeHandle &nh, Adafruit_PWMServoDriver ser_drr,
             int port, long max_ang, long min_ang, double conv_offset,
             double conv_scale, long park_deg);

  void setup_ease(double tar_angle, long duration);
  double compute_next_increment(long current_time_ms);
  void move(double deg);

private:
  long max_angle;
  long min_angle;
  double convert_scale;
  double convert_offset;

  long start_time_ms;
  long duration_ms;
  double target_angle;
  double start_angle;
  double change_in_value_angle;
  double quad_equation(double time_increment);
  double smooth_step_equation(double time_increment);
  Adafruit_PWMServoDriver servo_driver;
  int pin_number;
};

#endif
