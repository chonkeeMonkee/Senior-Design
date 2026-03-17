#include "imu_sensor.h"
#include <Wire.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_MPU6050 mpu;

static float currentPitch = 0.0;
static float currentRoll = 0.0;
static float currentYaw = 0.0;
static unsigned long lastTime = 0;

static float rawAccelX, rawAccelY, rawAccelZ;
static float rawGyroX, rawGyroY, rawGyroZ;
static float currentTemp;

bool initializeIMU() {
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!mpu.begin()) {
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    lastTime = millis();
    return true;
}

void updateIMU() {
    sensors_event_t a, g, temp;
    bool ok = mpu.getEvent(&a, &g, &temp);

    if (!ok) {
        static uint8_t failures = 0;
        if (++failures >= 3) {
            failures = 0;
            Wire.end();
            delay(5);
            Wire.begin(I2C_SDA, I2C_SCL);
            delay(10);
            mpu.begin();
        }
        return;
    }

    rawAccelX = a.acceleration.x;
    rawAccelY = a.acceleration.y;
    rawAccelZ = a.acceleration.z;

    rawGyroX = g.gyro.x;
    rawGyroY = g.gyro.y;
    rawGyroZ = g.gyro.z;

    currentTemp = temp.temperature;

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    if (dt > 0.1f) dt = 0.1f;

    float gyroRateX = rawGyroX * 180.0f / PI;
    float gyroRateY = rawGyroY * 180.0f / PI;
    float gyroRateZ = rawGyroZ * 180.0f / PI;

    float accelRoll  = atan2(rawAccelX, rawAccelZ) * 180.0f / PI;
    float accelPitch = atan2(-rawAccelY, sqrt(rawAccelX * rawAccelX + rawAccelZ * rawAccelZ)) * 180.0f / PI;

    currentPitch = 0.96f * (currentPitch + gyroRateX * dt) + 0.04f * accelPitch;
    currentRoll  = 0.96f * (currentRoll  - gyroRateY * dt) + 0.04f * accelRoll;

    currentYaw -= gyroRateZ * dt;
}

void IMU_GetRawAccel(float &x, float &y, float &z) {
    x = rawAccelX;
    y = rawAccelY;
    z = rawAccelZ;
}

void getRawGyro(float &x, float &y, float &z) {
    x = rawGyroX;
    y = rawGyroY;
    z = rawGyroZ;
}

float IMU_GetTemperature() {
    return currentTemp;
}

static float PITCH_OFFSET = 4.46f;
static float ROLL_OFFSET  = 0.46f;

void getIMUOrientation(float &pitch, float &roll, float &yaw) {
    pitch = currentPitch - PITCH_OFFSET;
    roll  = currentRoll  - ROLL_OFFSET;
    yaw   = currentYaw;
}

void ZeroIMU() {
    PITCH_OFFSET = currentPitch;
    ROLL_OFFSET  = currentRoll;
    currentYaw   = 0.0f;
}
