#include "branbuzz.h"
#include "branutils.h"
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

BrandeisBuzz::BrandeisBuzz() : node_handle(nullptr) {
  Serial.begin(115200);
  Wire.begin();          // Join I2C bus
  Wire.setClock(400000); // sound effects require changing configuration quickly
  if (buzzer.begin() == false) {
    buzz_ok = false;
    LOG_ERROR("Failed to initialize buzzer");
  }
  LOG_INFO("Buzzer initialized ok");
};

void BrandeisBuzz::setup(ros::NodeHandle &nh)
  { 
    node_handle = &nh;
  }

void BrandeisBuzz::chirp(int millis) {
  
}
