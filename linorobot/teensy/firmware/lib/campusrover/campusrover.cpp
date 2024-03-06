#include "campusrover.h"



CampusRover::CampusRover() : node_handle(nulptr) {}

void CampusRover::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
  ros::Subscriber<lino_msgs::ArmMsg> cb_sub("cb", cr_cb);
  nh.subscribe(cr_cb);
}

void cr_cb(const lino_msgs::CampusRover &campusrover_msg) {
  const char *command = firmware_msg->command;
}

void CampusRover::loop() {}


