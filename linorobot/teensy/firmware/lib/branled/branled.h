#ifndef BRANLED_H
#define BRANLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ros.h>

class BrandeisLED {
public:
    BrandeisLED();
    void setup(ros::NodeHandle& nh, int rate);
    void set_state(String new_state);
    void loop();
    void left_right_alternate(int rate);
        
private:

    Adafruit_MCP23X17 mcp;
    ros::NodeHandle* node_handle;
    String state;
    bool i2c_ok;
    bool every_other(int period_length_ms);
    char buffer[300];
    int rate; // activate every "n" millseconds. Note that other intervals must be multiples of "n"
};

#endif