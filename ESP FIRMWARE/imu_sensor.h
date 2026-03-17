#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

bool  initializeIMU();
void  updateIMU();
void  IMU_GetRawAccel(float &x, float &y, float &z);
void  getRawGyro(float &x, float &y, float &z);
float IMU_GetTemperature();
void  getIMUOrientation(float &pitch, float &roll, float &yaw);
void  ZeroIMU();

#endif
