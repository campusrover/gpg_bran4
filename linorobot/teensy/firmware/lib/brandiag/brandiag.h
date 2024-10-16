#ifndef BRANDIAG_H
#define BRANDIAG_H

#include "lino_msgs/Diag.h"
#include <ros.h>
#include "branarm.h"
#include "branbuzz.h"

class BrandeisDiag {

public:
  BrandeisDiag();
  void command(const char *cmd, const char *sub, float arg1, float arg2, float arg3, BrandeisArm &the_arm, BrandeisBuzz &the_buzz);
  void setup(ros::NodeHandle &nh);

private:
  ros::NodeHandle *node_handle;
  void led_commands(const char *sub, float arg1, float arg2, float arg3);
  void beep_commands(const char *sub, float arg1, float arg2, float arg3);
  void help_commands(const char *sub, float arg1, float arg2, float arg3);
  char buffer[300];
  BrandeisBuzz* the_buzz;
};

#endif
