#ifndef BRANDEOSSERVO_H
#define BRANDEOSSERVO_H
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>


class BrandeisServo {
public:
  ros::NodeHandle *node_handle;
  bool moving;
  bool test_mode;
  int counter;
  int error_counter;
  double current_angle;

  BrandeisServo();

  void setup(ros::NodeHandle &nh, Adafruit_PWMServoDriver ser_drr, int port,
             long max_ang, long min_ang, double conv_scale, double conv_offset);

  void setup_ease(double tar_angle, long curr_time_ms,
                               long dur_ms);
  double compute_next_increment(long current_time_ms);
  void move(double deg);
  void status(char buffer[200]);


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
  double quad_equation(long time_increment);
  double smooth_step_equation(long time_increment);

private:
  Adafruit_PWMServoDriver servo_driver;
  int port_num;
};

#endif
