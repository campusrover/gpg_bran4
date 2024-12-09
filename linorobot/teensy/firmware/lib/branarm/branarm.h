#ifndef BRANARM_H
#define BRANARM_H

#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>
#include "branarmconstants.h"
#include "brandeisservo.h"


struct ArmPositions {
    String name;
    float shoulder;
    float elbow;
    float wrist;
};

class BrandeisArm {

public:
    BrandeisArm();
    void setup(ros::NodeHandle& nh);
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
    bool wait_for_servo();
    bool wait_for_servo2();

    long move_duration_heuristic(long new_shoulder, long new_elbow, long new_wrist);
    void configure_ease_algorithm(long duration_in_ms);
    void movex();
    char buffer[300];

    void moveprog();
    int arm_prog_angle[10];
    int arm_prog_servo[10];
    int arm_prog_pc;
    


};

#endif
