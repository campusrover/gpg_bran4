#ifndef BRANBUZZ_H
#define BRANBUZZ_H

#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <SparkFun_Qwiic_Buzzer_Arduino_Library.h>

class BrandeisBuzz {
public:
    BrandeisBuzz();
    void setup(ros::NodeHandle& nh);
    void loop();
    void chirp(int millis);
    void set_state(String new_state, int new_rate);
    

private:
    QwiicBuzzer buzzer;
    ros::NodeHandle* node_handle;
    String state;
    bool buzz_ok;
    int rate;
    uint8_t volume;
    bool play(const uint16_t toneFrequency, const uint16_t duration, const uint8_t volume);
    bool every_other(int period_length_ms);
    char buffer[300];
};

#endif