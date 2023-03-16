#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>
#include <Wire.h>
#include "ros.h"
#include "ros/time.h"

//Header file for subscribing for LED
#include "lino_msgs/Led.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"
#include "std_msgs/Bool.h"
// //(Pito) header for instrumentation
// #include "lino_msgs/Inst.h"
// //(Pito) Header for camera servo
// #include "std_msgs/UInt16.h"
#include "Adafruit_AW9523.h"
#include "Adafruit_PWMServoDriver.h"
#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"

#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 1
#define CAMERA_SERVO_PIN 7

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 

// Servo steering_servo;
Servo steering_servo;
// Servo camera_servo; // Pito added

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

//LED Test
Adafruit_AW9523 aw;
bool LED_on1 = false;
bool LED_blink1 = false;
bool LED_blink_on1 = false;

bool LED_on2 = false;
bool LED_blink2 = false;
bool LED_blink_on2 = false;

bool LED_on3 = false;
bool LED_blink3 = false;
bool LED_blink_on3 = false;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

// Pincer servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  170 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  300 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
uint16_t pulselen = SERVOMIN;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);
void LEDCallback(const lino_msgs::Led& led_msg);
void ServoCallback(const std_msgs::Bool& servo_msg);
//void CameraServoCallback(const std_msgs::UInt16& servo_msg); // Pito added

//Pito added
long m1_pid_error = 0;
long m2_pid_error = 0;
long m1_curr_rpm = 0;
long m2_curr_rpm = 0;


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);
ros::Subscriber<lino_msgs::Led> led_sub("led", LEDCallback);
ros::Subscriber<std_msgs::Bool> servo_sub("servo", ServoCallback);
//ros::Subscriber<std_msgs::UInt16> cam_sub("camera/servo", CameraServoCallback); // Pito added

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

// lino_msgs::Inst inst_msg;
// ros::Publisher inst_pub("inst", &inst_msg);


void setup()
{
    //steering_servo.attach(STEERING_PIN);
    //steering_servo.write(90); 

    //camera_servo.attach(STEERING_PIN); // Pito added
    //camera_servo.write(0); // Pito added
    Serial.begin(115200);
    while (!Serial) delay(1);  // wait for serial port to open
    
    Serial.println("Adafruit AW9523 GPIO Expander test!");

    if (! aw.begin(0x58)) {
        Serial.println("AW9523 not found? Check wiring!");
        while (1) delay(10);  // halt forever
    }

    pwm.begin();

    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    Serial.println("AW9523 found!");
    aw.pinMode(1, OUTPUT);
    aw.pinMode(2, OUTPUT);
    aw.pinMode(3, OUTPUT);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.subscribe(led_sub);
    nh.subscribe(servo_sub);
    //nh.subscribe(cam_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    //nh.advertise(inst_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    char buffer[50];
    sprintf (buffer, "PID %f %f %f", K_P, K_D, K_I);
    nh.loginfo(buffer);
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;
    

    // This block sets the servo to the desired position
    pwm.setPWM(servonum, 0, pulselen);
    // char buffer[50];
    // sprintf(buffer, "&&&&&&&&&&& Pulse Length: %d", pulselen);
    // nh.loginfo(buffer);
    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        LEDUpdate();
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }


    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        nh.loginfo("Checking IMU.");

        char buffer[50];
        sprintf(buffer, "*********** IMU Address: %d", getIMUaddrs());
        nh.loginfo(buffer);
        // aw.digitalWrite(LedPin, LOW);
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
    
}

// void CameraServoCallback(const std_msgs::UInt16& servo_msg)
// {
//     char buffer[50];
//     sprintf (buffer,   "Cam servo %ld", servo_msg.data);
//     nh.loginfo(buffer);
//     sprintf (buffer,   "Cam attached? %ld", camera_servo.attached());
//     nh.loginfo(buffer);

//     camera_servo.write(servo_msg.data);
//     sprintf (buffer,   "Cam servo read %ld", camera_servo.read());
//     nh.loginfo(buffer);

// }

void PIDCallback(const lino_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void LEDCallback(const lino_msgs::Led& led_msg)
{   
    if (led_msg.wire == 1){
        LED_on1 = led_msg.on;
        LED_blink1 = led_msg.blink;
    } else if (led_msg.wire == 2){
        LED_on2 = led_msg.on;
        LED_blink2 = led_msg.blink;
    } else if (led_msg.wire == 3){
        LED_on3 = led_msg.on;
        LED_blink3 = led_msg.blink;
    }
    
}


void LEDUpdate()
{
    if (LED_on1 && !LED_blink1)
    {
        aw.digitalWrite(1, LOW);
    } else if (LED_on1 && LED_blink1)
    {   
        if (LED_blink_on1){
            aw.digitalWrite(1, LOW);
            LED_blink_on1 = false;
        } else{
            aw.digitalWrite(1, HIGH);
            LED_blink_on1 = true;
        }
    } else if (!LED_on1){
        aw.digitalWrite(1, HIGH);
    }


    if (LED_on2 && !LED_blink2)
    {
        aw.digitalWrite(2, LOW);
    } else if (LED_on2 && LED_blink2)
    {   
        if (LED_blink_on2){
            aw.digitalWrite(2, LOW);
            LED_blink_on2 = false;
        } else{
            aw.digitalWrite(2, HIGH);
            LED_blink_on2 = true;
        }
    } else if (!LED_on2){
        aw.digitalWrite(2, HIGH);
    }


    if (LED_on3 && !LED_blink3)
    {
        aw.digitalWrite(3, LOW);
    } else if (LED_on3 && LED_blink3)
    {   
        if (LED_blink_on3){
            aw.digitalWrite(3, LOW);
            LED_blink_on3 = false;
        } else{
            aw.digitalWrite(3, HIGH);
            LED_blink_on3 = true;
        }
    } else if (!LED_on3){
        aw.digitalWrite(3, HIGH);
    }
}



void ServoCallback(const std_msgs::Bool& servo_msg)
{
    if (servo_msg.data){
        pulselen = SERVOMAX;
    } else{
        pulselen = SERVOMIN;
    }
}


void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error 
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    // motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    // motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));  

// motor1_controller.spin(50); // PITO LEFT MOTOR
// motor2_controller.spin(100); // PITO RIGHT MOTOR

    // Pito added this
    m1_pid_error = req_rpm.motor1 - current_rpm1;
    m2_pid_error = req_rpm.motor2 - current_rpm2;
    m1_curr_rpm = current_rpm1;
    m2_curr_rpm = current_rpm2;

    Kinematics::velocities current_vel;

    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        
        current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);

//     //collect data for instrumentation message
//     inst_msg.l_encoder = motor1_encoder.read();
//     inst_msg.r_encoder = motor2_encoder.read();
//     inst_msg.l_piderror = m1_pid_error;
//     inst_msg.r_piderror = m2_pid_error;
//     inst_msg.l_rpm = m1_curr_rpm;
//     inst_msg.r_rpm = m2_curr_rpm;

//     // publish instrumentation message
//     inst_pub.publish(&inst_msg);
// 
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}

float steer(float steering_angle)
{
    //steering function for ACKERMANN base
    float servo_steering_angle;

    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);

    steering_servo.write(servo_steering_angle);

    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug()
{
    char buffer[50];

    sprintf (buffer,   "Encoders: %ld %ld", motor1_encoder.read(), motor2_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer,   "Pid Errors: %ld %ld", m1_pid_error, m2_pid_error);
    nh.loginfo(buffer);
    sprintf (buffer,   "Current RPM: %ld %ld", m1_curr_rpm, m2_curr_rpm);
    nh.loginfo(buffer);
}


