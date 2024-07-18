#include "Imu.h"
#include <Wire.h>
#define USE_20948_IMU
#define IMU_PUBLISH_RATE 2 // hz

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("\nBranBot Test");
}

void loop()
{
  static unsigned long prev_imu_time = 0;
  static bool imu_is_initialized = false;
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    if (!imu_is_initialized)
    {
      imu_is_initialized = initIMU();
    }
    else
    {
      geometry_msgs::Vector3 accel;
      accel = readAccelerometer();
      geometry_msgs::Vector3 gyro;
      gyro = readGyroscope();
      geometry_msgs::Vector3 mag;
      mag = readMagnetometer();
    }
    prev_imu_time = millis();
  }
}