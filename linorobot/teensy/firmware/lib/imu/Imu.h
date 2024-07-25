#ifndef _IMU2_H_ 
#define _IMU2_H_

#include "I2Cdev.h"
#include "imu_config.h"
#include "lino_base_config.h"

#include <Wire.h>
#include "geometry_msgs/Vector3.h"

#ifdef USE_20948_IMU

bool initIMU()
{
<<<<<<< HEAD
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    Wire.begin();
    Wire.setSDA(18);
    Wire.setSCL(19);

    bool ret;
    
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    ret = magnetometer.testConnection();
    if(!ret)
        return false;
=======
  // Serial.begin(115200);
  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // Serial.println("Adafruit Initializing!");

  // Try to initialize!
  if (!icm.begin_I2C())
  {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    // Serial.println("Failed to find ICM20948 chip. Trying again in 1 second");
    // while (1)
    // {
    //   delay(1000);
    // }
    return false;
  }

  // Serial.println("ICM20948 Found!");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  // Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange())
  {
  case ICM20948_ACCEL_RANGE_2_G:
    // Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    // Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    // Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    // Serial.println("+-16G");
    break;
  }
  // Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  // Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange())
  {
  case ICM20948_GYRO_RANGE_250_DPS:
    // Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    // Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    // Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    // Serial.println("2000 degrees/s");
    break;
  }
>>>>>>> 1949ced814aadffbe401cd58dbeb07969fb33594

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  // Serial.print("Accelerometer data rate divisor set to: ");
  // Serial.println(accel_divisor);
  // Serial.print("Accelerometer data rate (Hz) is approximately: ");
  // Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  // Serial.print("Gyro data rate divisor set to: ");
  // Serial.println(gyro_divisor);
  // Serial.print("Gyro data rate (Hz) is approximately: ");
  // Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  // Serial.print("Magnetometer data rate set to: ");
  // switch (icm.getMagDataRate())
  {
    // case AK09916_MAG_DATARATE_SHUTDOWN:
    //   Serial.println("Shutdown");
    //   break;
    // case AK09916_MAG_DATARATE_SINGLE:
    //   Serial.println("Single/One shot");
    //   break;
    // case AK09916_MAG_DATARATE_10_HZ:
    //   Serial.println("10 Hz");
    //   break;
    // case AK09916_MAG_DATARATE_20_HZ:
    //   Serial.println("20 Hz");
    //   break;
    // case AK09916_MAG_DATARATE_50_HZ:
    //   Serial.println("50 Hz");
    //   break;
    // case AK09916_MAG_DATARATE_100_HZ:
    //   Serial.println("100 Hz");
    //   break;
    // }
    // Serial.println();
    return true;
  }
}

geometry_msgs::Vector3 readAccelerometer()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  icm.getEvent(&accel, &gyro, &temp, &mag);
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  geometry_msgs::Vector3 accel_vector;
  int16_t ax, ay, az;
  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;
  accel_vector.x = ax;
  accel_vector.y = ay;
  accel_vector.z = az;
  return accel_vector;
}

geometry_msgs::Vector3 readGyroscope()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

  geometry_msgs::Vector3 gyro_vector;

  gyro_vector.x = gyro.gyro.x;
  gyro_vector.y = gyro.gyro.y;
  gyro_vector.z = gyro.gyro.z;

  return gyro_vector;
}

geometry_msgs::Vector3 readMagnetometer()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  geometry_msgs::Vector3 mag_vector;
  int16_t mx, my, mz;
  mx = mag.magnetic.x;
  my = mag.magnetic.y;
  mz = mag.magnetic.z;
  mag_vector.x = mx;
  mag_vector.y = my;
  mag_vector.z = mz;
  return mag_vector;
}

#else  // all but USE_20948_IMU
bool initIMU()
{
  Wire.begin();
  bool ret;

  accelerometer.initialize();
  ret = accelerometer.testConnection();
  if (!ret)
    return false;

  gyroscope.initialize();
  ret = gyroscope.testConnection();
  if (!ret)
    return false;

  magnetometer.initialize();
  ret = magnetometer.testConnection();
  if (!ret)
    return false;

  return true;
}

int getIMUaddrs()
{
  Wire.begin();
  accelerometer.initialize();
  int ret1 = accelerometer.getDeviceID();
  gyroscope.initialize();
  int ret2 = gyroscope.getDeviceID();
  magnetometer.initialize();
  int ret3 = magnetometer.getDeviceID();
  return (ret1 * 100000 + ret2 * 1000 + ret3);
}

geometry_msgs::Vector3 readAccelerometer()
{
  geometry_msgs::Vector3 accel;
  int16_t ax, ay, az;

  accelerometer.getAcceleration(&ax, &ay, &az);

  accel.x = ax * (double)ACCEL_SCALE * G_TO_ACCEL;
  accel.y = ay * (double)ACCEL_SCALE * G_TO_ACCEL;
  accel.z = az * (double)ACCEL_SCALE * G_TO_ACCEL;

  return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
  geometry_msgs::Vector3 gyro;
  int16_t gx, gy, gz;

  gyroscope.getRotation(&gx, &gy, &gz);

  gyro.x = gx * (double)GYRO_SCALE * DEG_TO_RAD;
  gyro.y = gy * (double)GYRO_SCALE * DEG_TO_RAD;
  gyro.z = gz * (double)GYRO_SCALE * DEG_TO_RAD;

  return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
  geometry_msgs::Vector3 mag;
  int16_t mx, my, mz;

  magnetometer.getHeading(&mx, &my, &mz);

  mag.x = mx * (double)MAG_SCALE * UTESLA_TO_TESLA;
  mag.y = my * (double)MAG_SCALE * UTESLA_TO_TESLA;
  mag.z = mz * (double)MAG_SCALE * UTESLA_TO_TESLA;

  return mag;
}
#endif // all but USE_20948_IMU

#endif