/******************************************************************************
  Example_01_Buzz

  This example shows how to turn the buzzer on and off.
  Much like the classic "blink LED sketch" this will buzz
  the buzzer once every second.

  By Pete Lewis @ SparkFun Electronics
  December 2023

  Based on code originally written by Fischer Moseley @ SparkFun Electronics
  Original Creation Date: June 28, 2019

  SparkFun code, firmware, and software is released under the MIT License.
        Please see LICENSE.md for further details.

  Hardware Connections:
  Connect QWIIC cable from Arduino to Qwiic Buzzer

  Distributed as-is; no warranty is given.
******************************************************************************/

#include <SparkFun_Qwiic_Buzzer_Arduino_Library.h>
QwiicBuzzer buzzer;

// Define the siren pattern table
struct SoundPattern {
  int frequency;
  int duration;
};

const SoundPattern sirenPattern[] = {
    {1000, 500}, {1200, 500}, {1400, 500}, {1600, 500},
    {1400, 500}, {1200, 500}, {1000, 500}, {800, 500},
    {1000, 500}, {1200, 500}, {1400, 500}, {1600, 500},
    {1400, 500}, {1200, 500}, {1000, 500}, {800, 500}};

const SoundPattern chirpPattern[] = {{1000, 15}, {2000, 15}, {3000, 15},
                                     {4000, 15}, {5000, 15}, {4000, 15},
                                     {3000, 15}, {2000, 15}};

const int SirenpatternLength = sizeof(sirenPattern) / sizeof(SoundPattern);
const int ChirppatternLength = sizeof(chirpPattern) / sizeof(SoundPattern);
bool err; // used for checking for errors

void setup() {
  Serial.begin(115200);
  Serial.println("My Test");
  Wire.begin(); // Join I2C bus

  // check if buzzer will connect over I2C
  if (buzzer.begin() == false) {
    Serial.println("Device did not connect! Freezing.");
    while (1)
      ;
  }
  Serial.println("Buzzer connected.");
}

void loop() {
  for (int i = 0; i < ChirppatternLength; i++) {
    err = buzzer.configureBuzzer(chirpPattern[i].frequency, 0, 3);
    // Check whether the write was successful
    if (err != kSTkErrOk)
      return;

    err = buzzer.on();
    // Check whether the write was successful
    if (err != kSTkErrOk)
      return;

    delay(chirpPattern[i].duration);
  };
  buzzer.off();
  Serial.println("again.");
  delay(2000);
};