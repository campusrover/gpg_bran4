#ifndef BRANDEOSSERVO_H
#define BRANDEOSSERVO_H
#include "transition.h"
#include <ros.h>

class BrandeisServo {
public:
  ros::NodeHandle *node_handle;
  bool moving;

  BrandeisServo();

  long current_time_ms;
  long start_time_ms;
  long duration_ms;
  long elapsed_time_ms;
  double current_angle;
  double target_angle;
  double start_angle;
  double change_in_value_angle;
  double loop_update_current_ms(long current_millis);
  void set_variables(double current_angle, double target_angle,
                     long duration_ms, double start_time_ms);
  void setup(ros::NodeHandle &nh);
  double smooth_step();
  double easeInOutQuad();

};

#endif
