#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "lino_base_config.h"

#include "Adafruit_AW9523.h"
#include "Adafruit_PWMServoDriver.h"
#include "Imu.h"
#include "Kinematics.h"
#include "Motor.h"
#include "PID.h"
#include "branarm.h"
#include "brandiag.h"
#include "branled.h"
#include "branutils.h"
#include "branbuzz.h"
#include "geometry_msgs/Twist.h"
#include "lino_msgs/ArmMsg.h"
#include "lino_msgs/Diag.h"
#include "lino_msgs/Imu.h"
#include "lino_msgs/PID.h"
#include "lino_msgs/Inst.h"
#include "lino_msgs/Velocities.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <Servo.h>
#include <Wire.h>
#include "Encoder.h"
#include "std_msgs/UInt16.h"

// Important Parameters
#define IMU_PUBLISH_RATE 20 // hz
#define COMMAND_RATE 20     // hz
#define DEBUG_RATE 60
#define CAMERA_SERVO_PIN 7
#define LEDLOOP_RATE 250 // ms

// Encoder Objects
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV);


// Motor Control Objects
Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A,
                             MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A,
                             MOTOR2_IN_B);
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A,
                             MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A,
                             MOTOR4_IN_B);

// PID objjects
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Kinematics Objects
Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

// Requested (or target) velocities)
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;
char buffer[500]; // for debug macros

BrandeisArm the_arm;
BrandeisLED the_led;
BrandeisDiag the_diag;
BrandeisBuzz the_buzz;

// callback function prototypes
void commandCallback(const geometry_msgs::Twist &cmd_msg);
void PIDCallback(const lino_msgs::PID &pid);
void armMsgCallback(const lino_msgs::ArmMsg &arm_msg);
void diagMsgCallback(const lino_msgs::Diag &diag_msg); 

// Variables for intrumentation
long m1_pid_error = 0;
long m2_pid_error = 0;
long m1_curr_rpm = 0;
long m2_curr_rpm = 0;

// ROSSERIAL Nodehandle
ros::NodeHandle nh;
ros::NodeHandle *node_handle = &nh;

// ROSSERIAL subscriber Objects
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);
ros::Subscriber<lino_msgs::ArmMsg> armMsg_sub("arm", armMsgCallback);
ros::Subscriber<lino_msgs::Diag> diag_sub("diag", diagMsgCallback); 

// Publisher Objects and messages
lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

lino_msgs::Inst inst_msg;
ros::Publisher inst_pub("inst", &inst_msg);

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.subscribe(armMsg_sub);
  nh.subscribe(diag_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  while (!nh.connected()) {
    nh.spinOnce();
    the_led.left_right_alternate(250);
  }
  LOG_INFO("CAMPUSROVER BASE CONNECTED!");
  delay(1);
  the_arm.setup(nh);
  the_led.setup(nh, LEDLOOP_RATE);
  the_diag.setup(nh);
  the_buzz.setup(nh);
}

void stopBase() {
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void publishIMU() {
  // pass accelerometer, gyroscope, accelerometer data to imu object
  raw_imu_msg.linear_acceleration = readAccelerometer();
  raw_imu_msg.angular_velocity = readGyroscope();
  raw_imu_msg.magnetic_field = readMagnetometer();

  // publish raw_imu_msg
  raw_imu_pub.publish(&raw_imu_msg);
}

// void publish_inst() {
//   // collect data for instrumentation message
//   inst_msg.l_encoder = motor1_encoder.read();
//   inst_msg.r_encoder = motor2_encoder.read();
//   inst_msg.l_piderror = m1_pid_error;
//   inst_msg.r_piderror = m2_pid_error;
//   inst_msg.l_rpm = m1_curr_rpm;
//   inst_msg.r_rpm = m2_curr_rpm;

//   // publish instrumentation message
//   inst_pub.publish(&inst_msg);
// }

void moveBase() {
  // get the required rpm for each motor based on required velocities, and base
  // used
  Kinematics::rpm req_rpm = kinematics.getRPM(
      g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  // get the current speed of each motor
  int current_rpm1 = motor1_encoder.getRPM();
  int current_rpm2 = motor2_encoder.getRPM();
  int current_rpm3 = motor3_encoder.getRPM();
  int current_rpm4 = motor4_encoder.getRPM();


  // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having
  // too much error the PWM value sent to the motor driver is the calculated PID
  // based on required RPM vs measured RPM
  motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
  motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));

  // Used when setting up new hardware to tell left from right and forward from
  // backward. motor1_controller.spin(50); // PITO LEFT MOTOR
  // motor2_controller.spin(100); // PITO RIGHT MOTOR

  m1_pid_error = req_rpm.motor1 - current_rpm1;
  m2_pid_error = req_rpm.motor2 - current_rpm2;
  m1_curr_rpm = current_rpm1;
  m2_curr_rpm = current_rpm2;

  Kinematics::velocities current_vel;
  current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);

  // pass velocities to publisher object
  raw_vel_msg.linear_x = current_vel.linear_x;
  raw_vel_msg.linear_y = current_vel.linear_y;
  raw_vel_msg.angular_z = current_vel.angular_z;

  // publish raw_vel_msg
  raw_vel_pub.publish(&raw_vel_msg);
}

void logIMUaddresses() {
  Wire.begin();
  accelerometer.initialize();
  gyroscope.initialize();
  magnetometer.initialize();
  LOG_INFO("IMU Addresses: %x %x %x", accelerometer.getDeviceID(),
           gyroscope.getDeviceID(), magnetometer.getDeviceID());
  LOG_INFO("IMU Testconnections: %x %x %x", accelerometer.testConnection(),
           gyroscope.testConnection(), magnetometer.testConnection());
}

void loop() {
  static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;
  static bool imu_is_initialized = false;

  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
    moveBase();
    prev_control_time = millis();
  }

  // this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400) {
    stopBase();
  }
  the_arm.loop();
  the_led.set_state("both_on");
  the_led.loop();
  the_buzz.loop();

  // this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE)) {
    if (!imu_is_initialized) {
      imu_is_initialized = initIMU();
      logIMUaddresses();
    } else {
      publishIMU();
    }
    prev_imu_time = millis();
  }

  // this block displays the encoder readings. change DEBUG to 0 if you don't
  // want to display
  if (DEBUG) {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE)) {
      // printDebug();
      prev_debug_time = millis();
    }
  }
  // call all the callbacks waiting to be called
  nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID &pid) {
  // callback function every time PID constants are received from lino_pid for
  // tuning this callback receives pid object where P,I, and D constants are
  // stored
  motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist &cmd_msg) {
  // callback function every time linear and angular speed is received from
  // 'cmd_vel' topic this callback function receives cmd_msg object where linear
  // and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;

  g_prev_command_time = millis();
}

void armMsgCallback(const lino_msgs::ArmMsg &arm_msg) {
  const char *command = arm_msg.command;
  the_arm.arm_command(command, arm_msg.arg1);
}

void diagMsgCallback(const lino_msgs::Diag &diag_msg) { 
  const char *cmd = diag_msg.command;
  const char *sub = diag_msg.subcommand;
  the_diag.command(cmd, sub, diag_msg.arg1, diag_msg.arg2, diag_msg.arg3, the_arm, the_buzz);
}

float mapFloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug() {
  char buffer[50];

  sprintf(buffer, "Encoders: %ld %ld", motor1_encoder.read(),
          motor2_encoder.read());
  nh.loginfo(buffer);
  sprintf(buffer, "Pid Errors: %ld %ld", m1_pid_error, m2_pid_error);
  nh.loginfo(buffer);
  sprintf(buffer, "Current RPM: %ld %ld", m1_curr_rpm, m2_curr_rpm);
  nh.loginfo(buffer);
}
