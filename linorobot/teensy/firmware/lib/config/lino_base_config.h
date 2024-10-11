#ifndef PLATFAM_CONFIG_H
#define PLATFAM_CONFIG_H

/*
Define what Robot. One of PLAT1, PLAT2, PLAT3, PLAT4, BRANBOT1
*/
#define BULLET

#ifdef PLAT1
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
#endif

#ifdef PLAT2
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
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

#ifdef BULLET
  #define PLAT_HAS_CAMERA
  #define USE_MPU9250_IMU
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

#ifdef BULLET
  #define USE_L298_DRIVER
  #define MAX_RPM 210              // motor's maximum RPM
  #define COUNTS_PER_REV 1180      // wheel encoder's no of ticks per rev
  #define WHEEL_DIAMETER 0.074	    // wheel's diameter in meters
  #define PWM_BITS 8                // PWM Resolution of the microcontroller
  #define LR_WHEELS_DISTANCE 0.205  // distance between left and right wheels
  #define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
  #define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering
#else
  #define USE_L298_DRIVER
  #define MAX_RPM 100              // motor's maximum RPM
  #define COUNTS_PER_REV 3200      // wheel encoder's no of ticks per rev
  #define WHEEL_DIAMETER 0.144	    // wheel's diameter in meters
  #define PWM_BITS 8                // PWM Resolution of the microcontroller
  #define LR_WHEELS_DISTANCE 0.28  // distance between left and right wheels
  #define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels.
  #define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering
#endif


// MOTOR1 = LEFT motor, MOTOR2 = RIGHT motor
#ifdef BULLET
  #define MOTOR1_ENCODER_A 11
  #define MOTOR1_ENCODER_B 12
  #define MOTOR2_ENCODER_A 15
  #define MOTOR2_ENCODER_B 14
#else
  #define MOTOR1_ENCODER_A 14
  #define MOTOR1_ENCODER_B 15
  #define MOTOR2_ENCODER_A 12
  #define MOTOR2_ENCODER_B 11
#endif

#ifdef BULLET
  #define MOTOR_DRIVER L298
  #define MOTOR1_IN_A 8
  #define MOTOR1_IN_B 6
  #define MOTOR1_PWM 5

  #define MOTOR2_IN_A 20
  #define MOTOR2_IN_B 1
  #define MOTOR2_PWM 21
#else
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1
  #define MOTOR1_PWM 22
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8
  #define MOTOR2_PWM 5
#endif

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#endif