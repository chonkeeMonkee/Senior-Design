    #include "pid_controller.h"
    #include <math.h>

    float clampf(float value, float low, float high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    float updatePID(PID &pid, float setpoint, float measurement, float dt) {
        if (dt <= 0.0f)
            return 0.0f;

        float error = setpoint - measurement;

        float pTerm = pid.kp * error;

        pid.integral += error * dt;
        pid.integral  = clampf(pid.integral, -pid.maxIntegral, pid.maxIntegral);
        float iTerm   = pid.ki * pid.integral;

        float dTerm = 0.0f;
        if (!pid.firstRun) {
            float dMeasurement = (measurement - pid.previousMeasurement) / dt;
            dTerm = -pid.kd * dMeasurement;
        }
        pid.previousMeasurement = measurement;
        pid.firstRun        = false;

        float output = pTerm + iTerm + dTerm;
        return clampf(output, -pid.maxOutput, pid.maxOutput);
    }

    void resetPID(PID &pid) {
        pid.integral        = 0.0f;
        pid.previousMeasurement = 0.0f;
        pid.firstRun        = true;
    }
