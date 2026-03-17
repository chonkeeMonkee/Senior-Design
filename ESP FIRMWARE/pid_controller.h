#pragma once

struct PID {
    float kp;
    float ki;
    float kd;
    float maxIntegral;
    float maxOutput;

    float integral;
    float previousMeasurement;
    bool  firstRun;
};

float updatePID(PID &pid, float setpoint, float measurement, float dt);

void resetPID(PID &pid);
