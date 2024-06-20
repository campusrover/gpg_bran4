#include "lino_base_config.h"
#include "Motor.h"
#include "Encoder.h"

#define COMMAND_RATE 20     // hz


void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
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

void loop() {
  static unsigned long prev_control_time = 0;
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
    test_encoder();
    prev_control_time = millis();
  }

  void test_encoder() {
    Serial.print("Motor1: ");
    Serial.println(motor1_encoder.read());
    Serial.print("Motor2: ");
    Serial.println(motor2_encoder.read());
  }
  
