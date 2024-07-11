#include "lino_base_config.h"
#include "Motor.h"
#include "Encoder.h"
#include <Wire.h>

#define COMMAND_RATE 1 // hz

void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial)
    Serial.println("\nBranBot Test");
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

void test_motors()
{
  motor1_controller.spin(-200);
  motor2_controller.spin(100);
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
