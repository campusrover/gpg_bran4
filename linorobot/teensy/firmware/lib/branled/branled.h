#ifndef BRANLED_H
#define BRANLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ros.h>

class BrandeisLED {
public:
    BrandeisLED();
    void setup(ros::NodeHandle& nh);
    void not_connected();
    void loop();
    long timer;
    
private:
    Adafruit_MCP23X17 mcp;
    ros::NodeHandle* node_handle;
    bool i2c_ok;
    bool every_other(int period_length_ms);
};

#endif