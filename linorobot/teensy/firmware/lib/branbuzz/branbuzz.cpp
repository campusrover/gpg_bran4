#include "branbuzz.h"
#include "branutils.h"
#include <Adafruit_MCP23X17.h>
#include <Wire.h>
#include <cstring>

BrandeisBuzz::BrandeisBuzz() : node_handle(nullptr){
  rate = 0;
  state = "stop";
};

void BrandeisBuzz::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
  LOG_INFO("BrandeisBuzz::Setup called");
  Wire.begin(); // Join I2C bus
  // Wire.setClock(400000); // sound effects require changing configuration
  // quickly
  if (buzzer.begin() == false) {
    buzz_ok = false;
    LOG_ERROR("Failed to initialize buzzer");
  }
  LOG_INFO("Buzzer initialized ok");
}

void BrandeisBuzz::chirp(int millis) {
  play(SFE_QWIIC_BUZZER_NOTE_E7, millis, SFE_QWIIC_BUZZER_VOLUME_MID);
}

bool BrandeisBuzz::play(const uint16_t toneFrequency, const uint16_t duration,
                        const uint8_t volume) {
  sfeTkError_t err = buzzer.configureBuzzer(toneFrequency, duration, volume);
  if (err != kSTkErrOk)
    return false;
  err = buzzer.on();
  if (err != kSTkErrOk) {
    return false;
  }
  delay(duration);
  return true;
}

void BrandeisBuzz::set_state(String new_state, int new_rate) {
  if (new_state.equals("beep-beep")) {
    state = new_state;
    rate = new_rate;
    LOG_INFO("BranBuzz set_state %s rate is %d", new_state.c_str(), rate);
  } else if (new_state.equals("stop") || new_rate == 0) {
    state = "stop";
  } else {
    state = "error";
  }
}

bool BrandeisBuzz::once_per_ms(int interval) {
  long elapsed_time = millis() - previous_time;
  if (elapsed_time > interval) {
    previous_time = millis();
    return true;
  } else {
    return false;
  }
}

void BrandeisBuzz::loop() {
  if (!once_per_ms(rate) || state.equals("stop")) {
    return;
  } else if (state.equals("beep-beep")) {
    LOG_INFO("Chirp Called");
    chirp(150);
  }
}