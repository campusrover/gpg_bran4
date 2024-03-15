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
        
private:
    QwiicBuzzer buzzer;
    ros::NodeHandle* node_handle;
    bool buzz_ok;
    char buffer[300];
};

#endif