#pragma once

//#include "Adafruit_AHRS_FusionInterface.h"
#include <math.h>
#include <TimersAndCalculations.h>

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

class fsc_madgwick // old mahony
{
public:
    fsc_madgwick();
    fsc_madgwick(float gain);
    //void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void updateIMU(Quaternion* q, float gx, float gy, float gz, float ax, float ay, float az, float dt);
    //static float invSqrt(float x);
    void computeIMUAngles(Quaternion q, float* roll, float* pitch, float* yaw, Quaternion* grav);
    //void computeAngles();
    float getRoll() {
        if (!anglesComputed)
            //computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed)
            //computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed)
            //computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    void getQuaternion(Quaternion q, float* w, float* x, float* y, float* z) {
        *w = q.w;
        *x = q.x;
        *y = q.y;
        *z = q.z;
    }
    void setQuaternion(Quaternion* q, float w, float x, float y, float z) {
        q->w = w;
        q->x = x;
        q->y = y;
        q->z = z;
    }
    void getGravityVector(float* x, float* y, float* z) {
        if (!anglesComputed)
            //computeAngles();
        *x = grav[0];
        *y = grav[1];
        *z = grav[2];
    }
private:
    float beta; // algorithm gain
    //float qW;
    //float qX;
    //float qY;
    //float qZ; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll, pitch, yaw;
    float grav[3];
    bool anglesComputed = false;
};
