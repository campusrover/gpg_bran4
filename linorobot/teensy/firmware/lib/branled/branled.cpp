#include "branled.h"
#include "campusrover.h"
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

#define LEFT_LED_PIN 1  // MCP23XXX pin Left LED is attached to
#define RIGHT_LED_PIN 2 // MCP23XXX pin Right LED is attached to

Adafruit_MCP23X17 mcp;

BrandeisLED::BrandeisLED() : node_handle(nullptr) {
  i2c_ok = true;

  if (!mcp.begin_I2C(0x58)) {
    i2c_ok = false;
  }
  // configure LED pin for output
  mcp.pinMode(RIGHT_LED_PIN, OUTPUT);
  mcp.pinMode(LEFT_LED_PIN, OUTPUT);
};

void BrandeisLED::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
}

void BrandeisLED::not_connected() {
  if (!i2c_ok) {
    return;
  }
  if (every_other(250)) {
    mcp.digitalWrite(LEFT_LED_PIN, HIGH);
    mcp.digitalWrite(RIGHT_LED_PIN, LOW);
  } else {
    mcp.digitalWrite(LEFT_LED_PIN, LOW);
    mcp.digitalWrite(RIGHT_LED_PIN, HIGH);
  }
}

bool BrandeisLED::every_other(int period_length_ms) {
  return (millis() / (2 * period_length_ms)) % 2 == 0;
}

void BrandeisLED::loop() {
  mcp.digitalWrite(RIGHT_LED_PIN, HIGH);
  mcp.digitalWrite(LEFT_LED_PIN, HIGH);
}
