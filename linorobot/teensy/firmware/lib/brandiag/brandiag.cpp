#include "branutils.h"
#include "brandiag.h"
BrandeisDiag::BrandeisDiag() : node_handle(nullptr) {}

void BrandeisDiag::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
  ros::Subscriber<lino_msgs::DiagMsg> cb_sub("cb", diag_cb);
  nh.subscribe(diag_cb);
}

void diag_cb(const lino_msgs::CampusRover &campusrover_msg) {
  const char *command = firmware_msg->command;
}

void BrandeisDiag::loop() {}


