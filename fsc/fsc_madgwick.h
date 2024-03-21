#pragma once

//#include "Adafruit_AHRS_FusionInterface.h"
#include <math.h>

class fsc_madgwick // old mahony
{
public:
    fsc_madgwick();
    fsc_madgwick(float gain);
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
    float beta; // algorithm gain
    float q0;
    float q1;
    float q2;
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll, pitch, yaw;
    float grav[3];
    bool anglesComputed = false;
};
