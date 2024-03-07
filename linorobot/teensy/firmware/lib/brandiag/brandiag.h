#ifndef BRANDIAG_H
#define BRANDIAG_H

#include <ros.h>

class BrandeisDiag {
public:
  BrandeisDiag();
  void setup(ros::NodeHandle &nh);
  void loop();

private:
  char buffer[300];
  ros::NodeHandle *node_handle;
  // ros::Subscriber<lino_msgs::CampusRover> armMsg_sub("cr", cr_cb);
  // void campusrover_cb(const lino_msgs::Cr &cr){};

};

#endif 