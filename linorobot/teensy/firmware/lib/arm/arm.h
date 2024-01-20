#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>
#include "armconstants.h"

class Arm {

public:
    Arm();
    void setup(ros::NodeHandle& nh);
    String getState();
    void armCommand(String command);
    void loop();

private:
    Adafruit_PWMServoDriver ARM;
    float iteration_time;
    float iteration_interval;

    int destination_shoulder;
    int destination_wrist;
    int destination_elbow;

    float wristposition;
    float elbowposition;
    float shoulderposition;

    float CurrentClaw = ClawParkdeg;
    float CurrentWrist = WristParkdeg;
    float CurrentElbow = ElbowParkdeg;
    float CurrentShoulder = ShoulderParkdeg;

    float iterations = 0;

    int shouldercnt = 0;
    int elbowcnt = 0;
    int wristcnt = 0;
    float shoulderdelta = 0;
    float elbowdelta = 0;
    float wristdelta = 0;

    String state;
    ros::NodeHandle* nodeHandle;

    void elbow(float deg);
    void shoulder(float deg);
    void wrist(float deg);
    void calculateIterationDeltas();
    int move();
};

#endif
