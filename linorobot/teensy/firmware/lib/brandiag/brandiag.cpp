#include "brandiag.h"
#include "branutils.h"
#include <cstring>

BrandeisDiag::BrandeisDiag() : node_handle(nullptr) {}

void BrandeisDiag::setup(ros::NodeHandle &nh) { node_handle = &nh; }

void BrandeisDiag::command(const char *cmd, const char *sub, float arg1,
                           float arg2, float arg3, BrandeisArm &the_arm,
                           BrandeisBuzz &buzz) {

  LOG_INFO("Diag: %s %s %.1f %.1f %.1f", cmd, sub, arg1, arg2, arg3);
  the_buzz = &buzz;
  if (strcmp(cmd, "led") == 0) {
    led_commands(sub, arg1, arg2, arg3);
  } else if (strcmp(cmd, "buzz") == 0) {
    beep_commands(sub, arg1, arg2, arg3);
  } else if (strcmp(cmd, "help") == 0) {
    help_commands(sub, arg1, arg2, arg3);
  } else if (strcmp(cmd, "arm") == 0) { 
    the_arm.arm_command(sub, arg1);
  }
};

void BrandeisDiag::led_commands(const char *sub, float arg1, float arg2,
                                float arg3) {
  LOG_INFO("Led: %s %.1f %.1f %.1f", sub, arg1, arg2, arg3);
}

// sub: "beep-beep"
// arg1: millisecond interval between beeps or zero to stop
void BrandeisDiag::beep_commands(const char *sub, float arg1, float arg2,
                                 float arg3) {
  LOG_INFO("Buzz: %s %.1f %.1f %.1f", sub, arg1, arg2, arg3);
  String string_sub(sub);
  the_buzz->set_state(string_sub, int(arg1));
}

void BrandeisDiag::help_commands(const char *sub, float arg1, float arg2,
                                 float arg3) {
  LOG_INFO("Help: %s %.1f %.1f %.1f", sub, arg1, arg2, arg3);
}
