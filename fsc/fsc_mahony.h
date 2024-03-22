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
        *w = qW;
        *x = qX;
        *y = qY;
        *z = qZ;
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
    float qW, qX, qY,
        qZ; // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy,
        integralFBz; // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
    float grav[3];
    bool anglesComputed = false;
};
