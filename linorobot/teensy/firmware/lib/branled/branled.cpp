#include "branled.h"
#include "campusrover.h"
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
 
#define LEFT_LED_PIN 1      // MCP23XXX pin Left LED is attached to
#define RIGHT_LED_PIN 2     // MCP23XXX pin Right LED is attached to

Adafruit_MCP23X17 mcp;

BrandeisLED::BrandeisLED() : node_handle(nullptr) {
};

void BrandeisLED::setup(ros::NodeHandle &nh) { 
    node_handle = &nh;
    Serial.begin(9600);
    i2c_ok = true;

    if (!mcp.begin_I2C(0x58)) {
        i2c_ok = false;
    }
      // configure LED pin for output
    mcp.pinMode(RIGHT_LED_PIN, OUTPUT);
    mcp.pinMode(LEFT_LED_PIN, OUTPUT);
 


  servo_driver = Adafruit_PWMServoDriver(0x40);
  iteration_time = millis();
  iteration_interval = 20; // 20 milli second
  state = "idle";

