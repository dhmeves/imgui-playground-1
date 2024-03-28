#include "fsc_madgwick.h"


#define sampleFreqDef 100.0f // sample frequency in Hz
#define betaDef 0.1f         // 2 * proportional gain ORIGINAL VALUE: 0.2

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

fsc_madgwick::fsc_madgwick() : fsc_madgwick(betaDef) {}

fsc_madgwick::fsc_madgwick(float gain)
{
    beta = gain;
    //q->w = 1.0f;
    //q->x = 0.0f;
    //q->y = 0.0f;
    //q->z = 0.0f;
    invSampleFreq = 1.0f / sampleFreqDef;
    anglesComputed = false;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void fsc_madgwick::updateIMU(Quaternion* q, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qW, _2qX, _2qY, _2qZ, _4qW, _4qX, _4qY, _8qX, _8qY, qWqW, qXqX, qYqY,
        qZqZ;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q->x * gx - q->y * gy - q->z * gz);
    qDot2 = 0.5f * (q->w * gx + q->y * gz - q->z * gy);
    qDot3 = 0.5f * (q->w * gy - q->x * gz + q->z * gx);
    qDot4 = 0.5f * (q->w * gz + q->x * gy - q->y * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2qW = 2.0f * q->w;
        _2qX = 2.0f * q->x;
        _2qY = 2.0f * q->y;
        _2qZ = 2.0f * q->z;
        _4qW = 4.0f * q->w;
        _4qX = 4.0f * q->x;
        _4qY = 4.0f * q->y;
        _8qX = 8.0f * q->x;
        _8qY = 8.0f * q->y;
        qWqW = q->w * q->w;
        qXqX = q->x * q->x;
        qYqY = q->y * q->y;
        qZqZ = q->z * q->z;

        // Gradient decent algorithm corrective step
        s0 = _4qW * qYqY + _2qY * ax + _4qW * qXqX - _2qX * ay;
        s1 = _4qX * qZqZ - _2qZ * ax + 4.0f * qWqW * q->x - _2qW * ay - _4qX +
            _8qX * qXqX + _8qX * qYqY + _4qX * az;
        s2 = 4.0f * qWqW * q->y + _2qW * ax + _4qY * qZqZ - _2qZ * ay - _4qY +
            _8qY * qXqX + _8qY * qYqY + _4qY * az;
        s3 = 4.0f * qXqX * q->z - _2qX * ax + 4.0f * qYqY * q->z - _2qY * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
            s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q->w += qDot1 * dt;
    q->x += qDot2 * dt;
    q->y += qDot3 * dt;
    q->z += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w *= recipNorm;
    q->x *= recipNorm;
    q->y *= recipNorm;
    q->z *= recipNorm;
    anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//
//float fsc_madgwick::invSqrt(float x) {
//    float halfx = 0.5f * x;
//    union {
//        float f;
//        long i;
//    } conv = { x };
//    conv.i = 0x5f3759df - (conv.i >> 1);
//    conv.f *= 1.5f - (halfx * conv.f * conv.f);
//    conv.f *= 1.5f - (halfx * conv.f * conv.f);
//    return conv.f;
//}

//-------------------------------------------------------------------------------------------
// TODO - RM: ORIGINAL FUNCTION - BRING BACK WHEN YOU'RE DONE SCREWING AROUND
//void fsc_madgwick::computeAngles() {
//    roll = fsc_atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY);
//    pitch = asinf(-2.0f * (qX * qZ - qW * qY));
//    yaw = fsc_atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);
//    grav[0] = 2.0f * (qX * qZ - qW * qY);
//    grav[1] = 2.0f * (qW * qX + qY * qZ);
//    grav[2] = 2.0f * (qX * qW - 0.5f + qZ * qZ);
//    anglesComputed = 1;
//}

void fsc_madgwick::computeIMUAngles(Quaternion q, float* roll, float* pitch, float* yaw, Quaternion* grav)
{
    *roll = RAD_TO_DEG * fsc_atan2f(q.w * q.x + q.y * q.z, 0.5f - q.x * q.x - q.y * q.y);

    *pitch = RAD_TO_DEG * fsc_asinf(-2.0f * (q.x * q.z - q.w * q.y));
    grav->x = 2.0f * (q.x * q.z - q.w * q.y);
    grav->y = 2.0f * (q.w * q.x + q.y * q.z);
    grav->z = 2.0f * (q.x * q.w - 0.5f + q.z * q.z);
    if (q.z < 0)
    {
    }
    //*pitch = RAD_TO_DEG * fsc_asinf((-2.0f * (q.x * q.z - q.w * q.y)));
    *yaw = RAD_TO_DEG * fsc_atan2f(q.x * q.y + q.w * q.z, 0.5f - q.y * q.y - q.z * q.z);
    //*grav[0] = 2.0f * (q.x * q.z - q.w * q.y);
    //*grav[1] = 2.0f * (q.w * q.x + q.y * q.z);
    //*grav[2] = 2.0f * (q.x * q.w - 0.5f + q.z * q.z);
    anglesComputed = 1;
}

// THIS IS JUST SCREWING AROUND 
//void fsc_madgwick::computeAngles() {
//    yaw = atan2f(2 * qY * qW - 2 * qX * qZ, 1 - 2 * qY * qY - 2 * qZ * qZ); // HEADING
//    pitch = asinf(2 * qX * qY + 2 * qZ * qW); // ATTITDUE
//    roll = atan2f(2 * qX * qW - 2 * qY * qZ, 1 - 2 * qX * qX - 2 * qZ * qZ); // BANK
//
//    //roll = atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY); // BANK
//    //pitch = asinf(-2.0f * (qX * qZ - qW * qY)); // ATTITUDE
//    //yaw = atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);
//    grav[0] = 2.0f * (qX * qZ - qW * qY);
//    grav[1] = 2.0f * (qW * qX + qY * qZ);
//    grav[2] = 2.0f * (qX * qW - 0.5f + qZ * qZ);
//    anglesComputed = 1;
//}
