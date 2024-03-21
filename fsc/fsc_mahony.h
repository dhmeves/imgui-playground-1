#pragma once
#include <math.h>

class fsc_mahony
{
public:
    fsc_mahony();
    fsc_mahony(float prop_gain, float int_gain);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    static float invSqrt(float x);
    void computeAngles();
    float getRoll() {
        if (!anglesComputed)
            computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed)
            computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed)
            computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    void getQuaternion(float* w, float* x, float* y, float* z) {
        *w = q0;
        *x = q1;
        *y = q2;
        *z = q3;
    }
    void getGravityVector(float* x, float* y, float* z) {
        if (!anglesComputed)
            computeAngles();
        *x = grav[0];
        *y = grav[1];
        *z = grav[2];
    }
private:
    float twoKp; // 2 * proportional gain (Kp)
    float twoKi; // 2 * integral gain (Ki)
    float q0, q1, q2,
        q3; // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy,
        integralFBz; // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
    float grav[3];
    bool anglesComputed = false;
};
