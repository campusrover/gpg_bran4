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
    void armCommand(String command, float arg);
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

    float shoulderDelta = 0;
    float elbowDelta = 0;
    float wristDelta = 0;


    float currentClaw = CLAWPARKDEG;
    float currentWrist = WRISTPARKDEG;
    float currentElbow = ELBOWPARKDEG;
    float currentShoulder = SHOULDERPARKDEG;

    float iterations = 0;

    String state;
    ros::NodeHandle* nodeHandle;

    void elbow(float deg);
    void shoulder(float deg);
    void wrist(float deg);
    void elbow(int deg);
    void shoulder(int deg);
    void wrist(int deg);

    void openClaw();
    void closeClaw();
    void calculateIterationDeltas();
    int move();

    void traceOut(String msg);
};

#endif
