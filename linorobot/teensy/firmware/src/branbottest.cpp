#include "lino_base_config.h"
#include "Motor.h"
#include "Encoder.h"
#include <Wire.h>

#define COMMAND_RATE 1 // hz

void setup()
{
  Wire.begin();

  Serial.begin(9600);
<<<<<<< HEAD
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nBranBot Test");
  analogWriteFrequency(MOTOR1_PWM, 4482);
  analogWriteFrequency(MOTOR2_PWM, 4482);
=======
  while (!Serial)
    Serial.println("\nBranBot Test");
>>>>>>> 163d735075699f6029f54661cc6771eab7fcbbd1
}

// Encloder Objects

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);

// Motor Control Objects
Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A,
                             MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A,
                             MOTOR2_IN_B);

void test_encoders() {
  Serial.print("Motor1 encoder: ");
  Serial.println(motor1_encoder.read());
  Serial.print("Motor2 encoder: ");
  Serial.println(motor2_encoder.read());
}

<<<<<<< HEAD
void test_motors()
{
  motor1_controller.spin(-200);
  motor2_controller.spin(100);
=======
void test_motors() {
  
<<<<<<< HEAD
  motor1_controller.spin(200);
  motor2_controller.spin(1);
=======
  motor1_controller.spin(-100);
  motor2_controller.spin(200);
>>>>>>> 163d735075699f6029f54661cc6771eab7fcbbd1

>>>>>>> 35f6828e6dc91563a9aebe26eb808e354571a3a5
}

static unsigned long prev_control_time = -(COMMAND_RATE * 1000);

void loop()
{
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    prev_control_time = millis();

    test_encoders();
    test_motors();
  };
}
