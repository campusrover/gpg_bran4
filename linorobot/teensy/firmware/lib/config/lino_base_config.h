#ifndef PLATFAM_CONFIG_H
#define PLATFAM_CONFIG_H

/*
Define what Robot. One of PLAT1, PLAT2, PLAT3, PLAT4, BRANBOT1, BRANBOT2
*/
<<<<<<< HEAD
#define BRANBOT2
=======
#define PLAT2
>>>>>>> 1949ced814aadffbe401cd58dbeb07969fb33594

#ifdef PLAT1
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
#endif

#ifdef PLAT2
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
  #define PLAT_HAS_LED
#endif

#ifdef PLAT3
  #define PLAT_HAS_CAMERA
  #define PLAT_HAS_BUZZER
  #define PLAT_HAS_LED
  #define PLAT_HAS_ARM
  #define USE_MPU9250_IMU
#endif

#ifdef PLAT4
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
#endif

#ifdef BRANBOT1
  #define PLAT_HAS_CAMERA
  #define PLAT_HAS_LED
  #define USE_20948_IMU
#endif

#ifdef BRANBOT2
  #define PLAT_HAS_CAMERA
  #define PLAT_HAS_LED
#endif


/* 
  Universal configuration of all the Platfam Robots.
  Note that these are derived from the linorobot.org lino_base_config.h
  Common settings across all robots:
*/
#define DEBUG 1                         // Debug Setting
#define LINO_BASE DIFFERENTIAL_DRIVE    // 2WD and Tracked robot w/ 2 motors

// PID Parameters
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.2 // D constant

#define USE_L298_DRIVER
#define MAX_RPM 100              // motor's maximum RPM
#define COUNTS_PER_REV 3200      // wheel encoder's no of ticks per rev

#define WHEEL_DIAMETER 0.144	    // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.28  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels.
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

// MOTOR1 = LEFT motor, MOTOR2 = RIGHT motor
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15
#define MOTOR2_ENCODER_A 12
#define MOTOR2_ENCODER_B 11
#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16
#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10

#define MOTOR_DRIVER L298
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 1
#define MOTOR1_PWM 22
#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 8
#define MOTOR2_PWM 5
#define MOTOR3_IN_A 23
#define MOTOR3_IN_B 0
#define MOTOR3_PWM 22
#define MOTOR4_IN_A 2
#define MOTOR4_IN_B 3
#define MOTOR4_PWM 4

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#endif