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
    void arm_command(String command);
    void arm_command(String command, float arg);
    void loop();

private:
    Adafruit_PWMServoDriver ARM;
    float iteration_time;
    float iteration_interval;

    int destination_shoulder;
    int destination_wrist;
    int destination_elbow;

    float wrist_position;
    float elbow_position;
    float shoulder_position;

<<<<<<< HEAD
    float current_claw = CLAWPARKDEG;
    float current_wrist = WR_PARK_DEG;
    float current_elbow = EL_PARK_DEG;
    float current_shoulder = SH_PARK_DEG;
=======
    float shoulderDelta = 0;
    float elbowDelta = 0;
    float wristDelta = 0;


    float currentClaw = CLAWPARKDEG;
    float currentWrist = WRISTPARKDEG;
    float currentElbow = ELBOWPARKDEG;
    float currentShoulder = SHOULDERPARKDEG;
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26

    float iterations = 0;

    String state;
    ros::NodeHandle* node_handle;

    void elbow(float deg);
    void shoulder(float deg);
    void wrist(float deg);
    void elbow(int deg);
    void shoulder(int deg);
    void wrist(int deg);
    void claw(bool open_close)    

    void open_claw();
    void close_claw();
    void calculate_iteration_deltas();
    int move();

    void traceOut(String msg);
};

#endif
