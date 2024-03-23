#ifndef PLATFAM_CONFIG_H
#define PLATFAM_CONFIG_H

/* 
Universal configuration of all the Platfam Robots, plat1, plat2, plat3 and pla5. 
Note that these are derived from the linorobot.org lino_base_config.h
*/

/* NOTE: ONLY ONE OF PLAT1, PLAT2, PLAT3 or PLAT4 should be defined here */
#define PLAT1                           // Currently set up for PLAT1


#define DEBUG 1                         // Debug Setting
#define LINO_BASE DIFFERENTIAL_DRIVE    // 2WD and Tracked robot w/ 2 motors

// PID Parameters
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.2 // D constant

/* 
Parameters and settings that are specific to PLAT1
 */

#ifdef PLAT1
  #define USE_L298_DRIVER
  #define USE_MPU9250_IMU

  #define MAX_RPM 100              // motor's maximum RPM
  #define COUNTS_PER_REV 3200      // wheel encoder's no of ticks per rev

  #define WHEEL_DIAMETER 0.144	    // wheel's diameter in meters
  #define PWM_BITS 8                // PWM Resolution of the microcontroller
  #define LR_WHEELS_DISTANCE 0.28   // distance between left and right wheels
  #define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. 

  // MOTOR1 = LEFT motor, MOTOR2 = RIGHT motor

  #define MOTOR_DRIVER L298

  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1
  #define MOTOR1_PWM 22

  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8
  #define MOTOR2_PWM 5

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 3

  #define MOTOR1_ENCODER_A 14
  #define MOTOR1_ENCODER_B 15
  #define MOTOR2_ENCODER_A 12
  #define MOTOR2_ENCODER_B 11
  #define MOTOR3_ENCODER_A 17
  #define MOTOR3_ENCODER_B 16

  #define MOTOR4_ENCODER_A 9
  #define MOTOR4_ENCODER_B 10
  
  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#endif