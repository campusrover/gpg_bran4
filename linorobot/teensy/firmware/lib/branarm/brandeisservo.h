#ifndef BRANDEOSSERVO_H
#define BRANDEOSSERVO_H
#include "transition.h"
#include <ros.h>

class BrandeisServo {
public:
  Transition tr;
  ros::NodeHandle *node_handle;

  BrandeisServo();

  double current_time_ms;
  double current_angle;
  double target_angle;
  double start_time_ms;
  double elapsed_time_ms;
  double duration_ms;
  double start_angle;
  double change_in_value_angle;
  double loop_update_current_ms(int current_millis);
  void set_variables(double current_angle, double target_angle,
                     double duration_ms, double start_time_ms);
  void setup(ros::NodeHandle &nh);
  double no_easing();

};

#endif
