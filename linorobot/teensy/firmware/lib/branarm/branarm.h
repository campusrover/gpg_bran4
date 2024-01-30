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

    int destination_shoulder;
    int destination_wrist;
    int destination_elbow;

    float wrist_position;
    float elbow_position;
    float shoulder_position;

    float current_claw = CLAWPARKDEG;
    float current_wrist = WR_PARK_DEG;
    float current_elbow = EL_PARK_DEG;
    float current_shoulder = SH_PARK_DEG;

    float shoulderDelta = 0;
    float elbowDelta = 0;
    float wristDelta = 0;


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

    void open_claw();
    void close_claw();
    void calculate_iteration_deltas();
    void configure_ease_algorithm(long duration_in_ms);
    int move();
    void movex();
    void traceOut(String msg);
    void traceOut2(String msg, int mod);

};

struct ServoCoords {
    float shoulder;
    float elbow;
    float wrist;
};

struct ServoInfo {
    BrandeisServo shoulder;
    BrandeisServo elbow;
    BrandeisServo wrist;
    BrandeisServo claw;
};

enum ArmLocs {
    PARK,
    FLOOR_UP,
    STRAIGHTUP,
    VERT_HORIZ_HAND,
    ALL_FORWARD,
    FLOOR_DOWN,
    ALL_BACKWARD
};


#endif
