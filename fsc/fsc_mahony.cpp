#include "fsc_mahony.h"

#define DEFAULT_SAMPLE_FREQ 100.0f // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f)     // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)     // 2 * integral gain

fsc_mahony::fsc_mahony() : fsc_mahony(twoKpDef, twoKiDef) {}

fsc_mahony::fsc_mahony(float prop_gain, float int_gain) {
    twoKp = prop_gain; // 2 * proportional gain (Kp)
    twoKi = int_gain;  // 2 * integral gain (Ki)
    qW = 1.0f;
    qX = 0.0f;
    qY = 0.0f;
    qZ = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = false;
    invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
}

void fsc_mahony::updateIMU(float gx, float gy, float gz, float ax,
    float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = qX * qZ - qW * qY;
        halfvy = qW * qX + qY * qZ;
        halfvz = qW * qW - 0.5f + qZ * qZ;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            // integral error scaled by Ki
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = qW;
    qb = qX;
    qc = qY;
    qW += (-qb * gx - qc * gy - qZ * gz);
    qX += (qa * gx + qc * gz - qZ * gy);
    qY += (qa * gy - qb * gz + qZ * gx);
    qZ += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(qW * qW + qX * qX + qY * qY + qZ * qZ);
    qW *= recipNorm;
    qX *= recipNorm;
    qY *= recipNorm;
    qZ *= recipNorm;
    anglesComputed = 0;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float fsc_mahony::invSqrt(float x) {
    float halfx = 0.5f * x;
    union {
        float f;
        long i;
    } conv = { x };
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    return conv.f;
}


void fsc_mahony::computeAngles() {
    roll = atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY);
    pitch = asinf(-2.0f * (qX * qZ - qW * qY));
    yaw = atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);
    grav[0] = 2.0f * (qX * qZ - qW * qY);
    grav[1] = 2.0f * (qW * qX + qY * qZ);
    grav[2] = 2.0f * (qX * qW - 0.5f + qZ * qZ);
    anglesComputed = 1;
}
