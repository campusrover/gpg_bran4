#ifndef BRANARM_H
#define BRANARM_H

#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>
#include "branarmconstants.h"
#include "brandeisservo.h"


class BrandeisArm {

public:
    void setup(ros::NodeHandle& nh);
    BrandeisArm();
    String getState(void);
    void arm_command(String command);
    void arm_command(String command, float arg);
    bool arm_motion_stopped(void);
    void loop(void);

private:
    Adafruit_PWMServoDriver servo_driver;
    float iteration_time;
    float iteration_interval;
    float iterations = 0;

    String state;
    ros::NodeHandle* node_handle;

    void elbow(float deg);
    void shoulder(float deg);
    void wrist(float deg);
    
    void elbow(int deg);
    void shoulder(int deg);
    void wrist(int deg);
    void claw(bool open_close); 

    void configure_ease_algorithm(long duration_in_ms);
    void movex();
    char buffer[300];

};

struct ArmPositions {
    String name;
    float shoulder;
    float elbow;
    float wrist;
    long duration;  // how long in milliseconds does the whole mpvement take
};

struct ServoInfo {
    BrandeisServo shoulder;
    BrandeisServo elbow;
    BrandeisServo wrist;
    BrandeisServo claw;
};

#endif
