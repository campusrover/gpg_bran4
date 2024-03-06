#ifndef CAMPUSROVER_H
#define CAMPUSROVER_H
#include <Adafruit_MCP23X17.h>
#include <Arduino.h>
#include <Wire.h>

#define LOG_INFO(format, ...)                                                  \
  if (true) {                                                                  \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    node_handle->loginfo(buffer);                                              \
  }

#define LOG_ERROR(format, ...)                                                 \
  if (true) {                                                                  \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    node_handle->logerror(buffer);                                             \
  }

#define LOG_DEBUG(format, ...)                                                 \
  if (true) {                                                                  \
    sprintf(buffer, format, __VA_ARGS__);                                      \
    node_handle->logdebug(buffer);                                             \
  }

class CampusRover {
public:
  CampusRover();
  void setup(ros::NodeHandle &nh);
  void loop();

private:
  char buffer[300];
  ros::NodeHandle *node_handle;
  ros::Subscriber<lino_msgs::CampusRover> armMsg_sub("cr", cr_cb);
  void campusrover_cb(const lino_msgs::Cr &cr){};
}
#endif
