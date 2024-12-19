// Dear ImGui: standalone example application for DirectX 11

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>
#include <math.h>
#include <fstream>

// START - 3D PROJECTION
#include "ImGuizmo.h" // 3D projection
bool useWindow = true;
int gizmoCount = 1;
static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);


float camDistance = 8.f;

float objectMatrix[4][16] = {
  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  2.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  2.f, 0.f, 2.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  0.f, 0.f, 2.f, 1.f }
};

static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };


void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition);
void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16);
void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16);
void LookAt(const float* eye, const float* at, const float* up, float* m16);
void Normalize(const float* a, float* r);
float Dot(const float* a, const float* b);
void Cross(const float* a, const float* b, float* r);

int WriteOutFile(std::string str, uint8_t* iv, const char* path, uint16_t crc, long len)
{
    int output = -1;
    std::ofstream outfile(path, std::ios::binary);
    //long aesLen = TaC.GetAESLength(len);
    if (outfile.is_open())
    {
        outfile << "/* WARNING: ALTERATIONS TO THIS FILE MAY RESULT IN CORRUPTION */\n/* EDIT AT YOUR OWN RISK */" << std::endl; // Write the encryption heading
        //ADDING IV TO FILE
        //outfile << encStrIvLine << std::endl;
        //for (int i = 0; i < AES_BLOCKLEN; i++)
        //{
        //    outfile << std::dec << std::to_string(iv[i]) << std::endl;// << std::endl;
        //}
        outfile << std::endl;
        // ADDING LENGTH OF JSON TO FILE
        //outfile << encStrLengthLine << std::endl;
        outfile << len << std::endl;
        // ADDING CRC OF (PLAINTEXT) JSON TO FILE
        //outfile << encStrCrcLine << std::endl;
        //outfile << crc << std::endl;
        // ADDING START OF ENCRYPTION STRING TO FILE
        //outfile << encStrTriggerLine << std::endl;
        // ADDING ENCRYPTED STRING TO FILE
        //std::string s = "";
        //std::string s(encryptedStr.begin(), encryptedStr.end());
        //long len = s.length();
        printf("string length: %d\n", len);
        outfile.write(str.c_str(), len);
        outfile.close();
        output = 0;
    }
    return output;
}

// END - 3D PROJECTION

#include "TimersAndCalculations.h" // fsc library

//START - PCAN - USB READING
#include "03_ManualRead.h"

#include <math.h>
//#define PI 3.14159265


//#define FSC_MAHONY // Pick what algo you want for IMU algorithm
#define FSC_MADGWICK // Pick what algo you want for IMU algorithm

#if defined(FSC_MAHONY)
#include "fsc_mahony.h"
fsc_mahony imu;
#elif defined(FSC_MADGWICK)
#include "fsc_madgwick.h"
fsc_madgwick imu;
#endif

ManualRead CAN;


Quaternion imuC = { 1.0f, 0.0f, 0.0f, 0.0f };

Quaternion AverageQuaternion(struct Quaternion* cumulative, Quaternion newRotation, Quaternion firstRotation, int addAmount);
Quaternion NormalizeQuaternion(Quaternion q);
Quaternion InverseSignQuaternion(Quaternion q);
bool AreQuaternionsClose(Quaternion q1, Quaternion q2);
float qDot(Quaternion q1, Quaternion q2);


//Get an average (mean) from more than two quaternions (with two, slerp would be used).
//Note: this only works if all the quaternions are relatively close together.
//Usage:
//-Cumulative is an external Vector4 which holds all the added x y z and w components.
//-newRotation is the next rotation to be added to the average pool
//-firstRotation is the first quaternion of the array to be averaged
//-addAmount holds the total amount of quaternions which are currently added
//This function returns the current average quaternion
Quaternion AverageQuaternion(Quaternion cumulative[], Quaternion newRotation, Quaternion firstRotation, int addAmount)
{
    //float w = 0.0f;
    //float x = 0.0f;
    //float y = 0.0f;
    //float z = 0.0f;
    Quaternion avg = { 0, 0, 0, 0 };

    for (int i = 0; i < addAmount; i++)
    {
        if (!AreQuaternionsClose(cumulative[i], cumulative[0])) {

            cumulative[i] = InverseSignQuaternion(cumulative[i]);
        }
        avg.w += cumulative[i].w;
        avg.x += cumulative[i].x;
        avg.y += cumulative[i].y;
        avg.z += cumulative[i].z;
    }

    avg.w *= (1.0 / 10);
    avg.x *= (1.0 / 10);
    avg.y *= (1.0 / 10);
    avg.z *= (1.0 / 10);


    NormalizeQuaternion(avg);

    //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
    //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
    //if (!AreQuaternionsClose(newRotation, cumulative[0])) {

    //    newRotation = InverseSignQuaternion(newRotation);
    //}
    //Quaternion q;
    //Average the values
    //float addDet = 1.0f / (float)addAmount;
    //cumulative.w += newRotation.w;
    //q.w = cumulative.w * addDet;
    //cumulative.x += newRotation.x;
    //q.x = cumulative.x * addDet;
    //cumulative.y += newRotation.y;
    //q.y = cumulative.y * addDet;
    //cumulative.z += newRotation.z;
    //q.z = cumulative.z * addDet;

    //note: if speed is an issue, you can skip the normalization step
    //NormalizeQuaternion(avg);
    return avg;// NormalizeQuaternion(x, y, z, w);
}

Quaternion NormalizeQuaternion(Quaternion q)
{
    Quaternion newQ = q;
    float lengthD = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    newQ.w *= lengthD;
    newQ.x *= lengthD;
    newQ.y *= lengthD;
    newQ.z *= lengthD;

    return newQ;
}

//Changes the sign of the quaternion components. This is not the same as the inverse.
Quaternion InverseSignQuaternion(Quaternion q)
{
    Quaternion inverse = { -q.x, -q.y, -q.z, -q.w };
    return inverse;
}

//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool AreQuaternionsClose(Quaternion q1, Quaternion q2)
{
    float dot = qDot(q1, q2);

    if (dot < 0.0f)
        return false;
    else
        return true;
}

float qDot(Quaternion q1, Quaternion q2)
{
    float dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    return acos(dot);
}







const TPCANHandle PcanHandle1 = PCAN_USBBUS1;
//TPCANMsg CANMsg;
//TPCANTimestamp CANTimeStamp;


#define SENSOR_MM7_C_TELEGRAM_1_ID 0x374 // ACTUAL SENSOR C
#define SENSOR_MM7_C_TELEGRAM_2_ID 0x378
#define SENSOR_MM7_C_TELEGRAM_3_ID 0x37C

//#define SENSOR_MM7_C_TELEGRAM_1_ID 0x174 // JUST FOR TESTING - TODO - RM: REMOVE WHEN FINISHED TESTING
//#define SENSOR_MM7_C_TELEGRAM_2_ID 0x178
//#define SENSOR_MM7_C_TELEGRAM_3_ID 0x17C

//#define SENSOR_MM7_C_TELEGRAM_1_ID 0x274 // JUST FOR TESTING - TODO - RM: REMOVE WHEN FINISHED TESTING
//#define SENSOR_MM7_C_TELEGRAM_2_ID 0x278
//#define SENSOR_MM7_C_TELEGRAM_3_ID 0x27C

typedef struct // Same as Codesys SPN configuration, location of byte, bit inside that byte, and how long the value is
{
    uint8_t byte;
    uint8_t bit;
    uint8_t len;
} SPN_Config;

const SPN_Config SENSOR_MM7_TX1_YAW_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX1_CLU_STAT = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX1_YAW_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX1_TEMP_RATE_Z = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX1_AY = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX1_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX1_AY_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX1_CRC = { 7, 0, 8 };

const SPN_Config SENSOR_MM7_TX2_ROLL_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX2_CLU_STAT5 = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX2_ROLL_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX2_CLU_DIAG = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX2_AX = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX2_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX2_AX_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX2_CRC = { 7, 0, 8 };

const SPN_Config SENSOR_MM7_TX3_PITCH_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX3_HW_INDEX = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX3_PITCH_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX3_RESERVED = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX3_AZ = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX3_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX3_AZ_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX3_CRC = { 7, 0, 8 };


float GetAngularRateFromMM7Raw(uint16_t input);
float GetAccelerationFromMM7Raw(uint16_t input);
int16_t GetTemperatureFromMM7Raw(uint8_t input);
int ExtractBoolFromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, bool* output, SPN_Config spnConfig);
int ExtractUint16FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint16_t* output, SPN_Config spnConfig);
int ExtractUint8FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint8_t* output, SPN_Config spnConfig);


float CalculateRollEuler(float x, float y, float z)
{
    float miu = 0.001;
    int sign = 1;
    if (z < 0)
        sign = -1;
    float yaw = (180.0 / PI) * atan2(-y, sign * sqrt(z * z + miu * x * x));
    {
        //if (z < 0)
        //{
        //    yaw = -yaw;
        //    if (yaw > 0)
        //        yaw = 180 - yaw;
        //    else
        //        yaw = -180 - yaw;
        //}
    }
    return yaw;
}

float CalculateYawEuler(float x, float y, float z)
{
    float miu = 0.001;
    int sign = 1;
    if (x < 0)
        sign = -1;
    return (180.0 / PI) * atan2(-y, sign * sqrt(x * x + miu * z * z));
}

float CalculatePitchEuler(float x, float y, float z)
{
    //float miu = 0.001;
    //int sign = 1;
    //if (x < 0)
    //    sign = -1;
    float pitch = -(180.0 / PI) * atan2(x, sqrt(y * y + z * z));
    {
        //if (z < 0)
        //{
        //    if (pitch > 0)
        //        pitch = 180 - pitch;
        //    else
        //        pitch = -180 - pitch;
        //}
    }
    return pitch;
}

/**
 * @brief Slope-intercept linear scaling function, option to clip output to minOut~maxOut range.  y = mx + b.
 *        Note that `minOut` does NOT have to be the smaller number if a inverse slope is required. Execution
 *        time: approx 8us when `clipOutput` = `TRUE`.
 *
 * @param[in] input input val
 * @param[in] minIn minimum input val
 * @param[in] maxIn maximum input val
 * @param[in] minOut minimum output val
 * @param[in] maxOut maximum output val
 * @param[in] clipOutput `TRUE`: if output is found to be outside the range of minOut~maxOut, clip to the max. or min.
 *                    `FALSE`: Scale with no adjustments if output is outside range.
 * @return scaled output
 */
double scale(double input, double minIn, double maxIn, double minOut, double maxOut, bool clipOutput)
{
    if (minIn == maxIn)
    {
        return 0.0; // let's not divide by 0 :)
    }
    double slope = ((maxOut - minOut) / (maxIn - minIn));
    double intercept = (minOut - (minIn * slope));
    double output = ((slope * input) + intercept);
    if (clipOutput) //  DON'T ALLOW OUTPUT OUTSIDE RANGE
    {
        double minVal = (minOut < maxOut) ? minOut : maxOut; //	FIND MIN/MAX VALUES - INCASE WE ARE INVERSELY SCALING
        double maxVal = (minOut > maxOut) ? minOut : maxOut;
        if (output > maxVal)
            output = maxVal;
        if (output < minVal)
            output = minVal;
    }
    return output;
}

/**
 * @brief millisecond-time-based function that calculates an output given a current value, a setpoint, value range,
 * and time parameters. The output will follow a maximum slope calculated by using the min/max values and the rampTime
 *
 * @param[in,out] *currentlVal - passes current value in and the calculated new value out.
 * @param[in] setpoint - Where `*currentVal` should be after ramp is complete
 * @param[in] min - minimum value that `*currentVal` can be
 * @param[in] max - maximum value that `*currentVal` can be
 * @param[in,out] *prevTime - pointer that keeps track of the last time we calculated a value
 * @param[in] rampTime - (ms) how long the ramp should take from `min` to `max`
 * @return
 */
double RampScale(double currentVal, double setpoint, double min, double max, uint64_t* prevTime, uint64_t rampTime, bool* finishedRamp)
{
    *finishedRamp = FALSE;
    double minVal = min;//(min < max) ? min : max;	//	FIND MIN/MAX VALUES - INCASE WE ARE INVERSELY SCALING
    double maxVal = max;// (min > max) ? min : max;
    uint64_t now = millis();
    uint64_t timeSince = now - *prevTime;
    *prevTime = now;
    if (rampTime == 0) // let's not divide by 0
    {
        rampTime = 1;
    }
    double maxIncrement = (int)(((maxVal - minVal) / rampTime)* (double)timeSince); // FLOATS MAKE NEGATIVE NUMBERS SAD!
    printf("maxIncrement: %f\n", maxIncrement);
    double increment = setpoint - currentVal;
    double absIncrement = increment;
    bool goUp;
    if (increment == 0)
    {
        *finishedRamp = TRUE; // if we aren't incrementing, we're done ramping
        return currentVal;
    }
    if (increment > 0) // have to increase to reach setpoint
    {
        goUp = TRUE;
    }
    else // have to decrease to reach setpoint
    {
        absIncrement = -increment;
        goUp = FALSE;
    }

    if (absIncrement < maxIncrement)
    {
        currentVal += increment;
    }
    else if (goUp)
    {
        currentVal += maxIncrement;
    }
    else
    {
        currentVal -= maxIncrement;
    }

    if (currentVal > maxVal) // limit output to between inMin and inMax
    {
        currentVal = maxVal;
        *finishedRamp = TRUE; // if we are clipping, we're done ramping
    }
    if (currentVal < minVal)
    {
        currentVal = minVal;
        *finishedRamp = TRUE;
    }

    if (currentVal == setpoint) // if we are exactly equal, we're done ramping
    {
        *finishedRamp = TRUE;
    }
    return currentVal;
}

#define CMD_NO_CHANGE 0
#define CMD_DECREMENT 1
#define CMD_INCREMENT 2

/**
 * @brief Increments/Decrements a given value by a certain amount up or down given a tri-state bool
 *      Also clips to a min/max value.
 *
 * @param[inout] value* pointer to value that will be incremented/decremented
 * @param[in] increDecrement tri-state input, 1 = decrement, 2 = increment, anything else = don't change
 * @param[in] incrementVal How much to increment/decrement
 * @param[in] minRange minimum value to clip `value` to if decremented below
 * @param[in] maxRange maximum value to clip `value` to if incremented above
 */
void IncrementValue(int16_t* value, uint8_t increDecrement, uint16_t incrementVal, int32_t minRange, int32_t maxRange)
{
    // sint32 output = input; // Set output to input so we don't send garbage if we don't change anything.
    if (increDecrement == CMD_DECREMENT)
    {
        *value -= incrementVal;
        if (*value < minRange) // clip value if outside maxRange
        {
            *value = minRange;
        }
    }
    else if (increDecrement == CMD_INCREMENT)
    {
        *value += incrementVal;
        if (*value > maxRange) // clip value if outside maxRange
        {
            *value = maxRange;
        }
    }
}


typedef struct
{
    // INPUTS
    int16_t joystickVal;
    int16_t posMin_mA;
    int16_t posMax_mA;
    int16_t negMin_mA;
    int16_t negMax_mA;
    int16_t rampStartTime;
    int16_t rampStopTime;
    int16_t deadband;
    int16_t controlModeMult;

    // PRIVATE/INTERNAL
    int8_t setpoint;        // calculated setpoint (ramped)
    int8_t currentSetpoint; // calculated end-of-ramp setpoint
    int8_t prevSetpoint;    // setpoint from previous loop
    uint64_t prevTime;
    bool finishedRamp;

    // OUTPUTS
    int16_t outputPositive_mA;
    int16_t outputNegative_mA;

} rampedOutput_ts;

#define COIL_CURRENT_0_MA 0

#define MIN_SINT8_SETPNT -127//(int8_t)-127
#define MAX_SINT8_SETPNT 127//(int8_t)127

#define NEG_THOUSAND -1000.0f
#define NEG_HUNDRED -100.0f
#define NEG_TWO -2.0f
#define ZERO_FLT 0.0f
#define ONE_HALF 0.5f
#define ONE_FLT 1.0f
#define THREE_HALFS 1.5f
#define TWO 2.0f
#define FOUR 4.0f
#define EIGHT 8.0f
#define POS_HUNDRED 100.0f
#define POS_THOUSAND 1000.0f

#define SCANRECO_LINK_STATUS_RUNNING 0x01
#define SCANRECO_WDT_TIMEOUT 1000
#define SCANRECO_INPUT_IDLE 127
#define SCANRECO_INPUT_MIN 0
#define SCANRECO_INPUT_MAX 254
#define SCANRECO_INPUT_BTN_NOT_PRESSED 0x00
#define SCANRECO_INPUT_BTN_PRESSED 0x01

void RampedOutput(rampedOutput_ts* rampedVals)
{
    // Sensitivity % F-R
    int16_t sensitivity_H = 127 + ((127 * rampedVals->deadband) / 100); // 10% GAP sensitivity max (127-140)
    int16_t sensitivity_L = 127 - ((127 * rampedVals->deadband) / 100); // 10% GAP sensitivity max (115-127)
    bool inNeutral = false;

    // Adjust for deadband and control aggression
    if (rampedVals->joystickVal > 127)//sensitivity_H)
    {
        rampedVals->currentSetpoint = (int8_t)scale(rampedVals->joystickVal, SCANRECO_INPUT_IDLE/*sensitivity_H*/, SCANRECO_INPUT_MAX, ZERO_FLT, (MAX_SINT8_SETPNT * rampedVals->controlModeMult) / POS_HUNDRED, TRUE);
    }
    else if (rampedVals->joystickVal < 127)//sensitivity_L)
    {
        rampedVals->currentSetpoint = (int8_t)scale(rampedVals->joystickVal, SCANRECO_INPUT_IDLE/*sensitivity_L*/, SCANRECO_INPUT_MIN, ZERO_FLT, (MIN_SINT8_SETPNT * rampedVals->controlModeMult) / POS_HUNDRED, TRUE);
    }
    else
    {
        inNeutral = true;
        rampedVals->currentSetpoint = 0;
    }

    // Ramp setpoint
    if (inNeutral) // stop ramp time
    {
        rampedVals->setpoint = RampScale(rampedVals->prevSetpoint, rampedVals->currentSetpoint, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT, &rampedVals->prevTime, rampedVals->rampStopTime, &rampedVals->finishedRamp);
    }
    else // start ramp time
    {
        rampedVals->setpoint = RampScale(rampedVals->prevSetpoint, rampedVals->currentSetpoint, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT, &rampedVals->prevTime, rampedVals->rampStartTime, &rampedVals->finishedRamp);
    }

    // since setpoint is an int, make sure our difference isn't too small to change, and if it is increment/decrement by 1
    if (rampedVals->setpoint == rampedVals->prevSetpoint && rampedVals->setpoint != rampedVals->currentSetpoint)
    {
        int16_t setpoint = rampedVals->setpoint;
        if (rampedVals->setpoint < rampedVals->currentSetpoint)
        {
            IncrementValue(&setpoint, CMD_INCREMENT, 1, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT);
        }
        else if (rampedVals->setpoint > rampedVals->currentSetpoint)
        {
            IncrementValue(&setpoint, CMD_DECREMENT, 1, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT);
        }
        rampedVals->setpoint = setpoint;
    }

    // Apply proportional current to coils based on setpoint
    if (rampedVals->setpoint > 0)
    {
        rampedVals->outputPositive_mA = (int16_t)scale(rampedVals->setpoint, 0, MAX_SINT8_SETPNT, rampedVals->posMin_mA, rampedVals->posMax_mA, TRUE);
        rampedVals->outputNegative_mA = COIL_CURRENT_0_MA;
    }
    else if (rampedVals->setpoint < 0)
    {
        rampedVals->outputPositive_mA = COIL_CURRENT_0_MA;
        rampedVals->outputNegative_mA = (int16_t)scale(rampedVals->setpoint, 0, MIN_SINT8_SETPNT, rampedVals->negMin_mA, rampedVals->negMax_mA, TRUE);
    }
    else
    {
        rampedVals->outputPositive_mA = COIL_CURRENT_0_MA;
        rampedVals->outputNegative_mA = COIL_CURRENT_0_MA;
    }

    // canDebugBuf6[0] = rampedVals->setpoint;
    // canDebugBuf6[1] = rampedVals->currentSetpoint;
    // canDebugBuf6[2] = rampedVals->prevSetpoint;
    // canDebugBuf6[3] = rampedVals->finishedRamp;3

    rampedVals->prevSetpoint = rampedVals->setpoint; // make sure to update prevSetpoint
}



float MM7_C_YAW_RATE;
float MM7_C_ROLL_RATE;
float MM7_C_PITCH_RATE;
float MM7_C_AX;
float MM7_C_AY;
float MM7_C_AZ;
int16_t MM7_C_TEMP;
//END - PCAN - USB READING

// Data
static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Main code
int main(int, char**)
{
    // Create application window
    //ImGui_ImplWin32_EnableDpiAwareness();
    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"ImGui Example", nullptr };
    ::RegisterClassExW(&wc);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"Dear ImGui DirectX11 Example", WS_OVERLAPPEDWINDOW, 100, 100, 1280, 800, nullptr, nullptr, wc.hInstance, nullptr);

    // Initialize Direct3D
    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    // Show the window
    ::ShowWindow(hwnd, SW_MAXIMIZE);
    ::UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_pcan_window = false;
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {
        const uint32_t NUM_RATES = 1000;
        static float rollRates[NUM_RATES], rollRateAvg, rollRateMin, rollRateMax, pitchRates[NUM_RATES], pitchRateAvg, pitchRateMin, pitchRateMax, yawRates[NUM_RATES], yawRateAvg, yawRateMin, yawRateMax;

        // START - GRAB CAN DATA
        TPCANMsg CANMsg;
        TPCANTimestamp CANTimeStamp;

        // We execute the "Read" function of the PCANBasic   
        //printf("YAW RATE: %f\tPITCH RATE: %f\tROLL RATE: %f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
        //printf("AY: %f\tAX: %f\tAZ: %f\t\n", MM7_C_AY, MM7_C_AX, MM7_C_AZ);
        //printf("\n~~~~~~~~~~~~~~~~~~~~\n");
        TPCANStatus stsResult = CAN_Read(PcanHandle1, &CANMsg, &CANTimeStamp); // PCAN_ERROR_QRCVEMPTY;
        if (stsResult != PCAN_ERROR_QRCVEMPTY)
        {
            switch (CANMsg.ID)
            {
            case SENSOR_MM7_C_TELEGRAM_1_ID:
            {
                uint16_t yawRate;
                uint16_t acceleration;
                uint8_t temperature;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &yawRate, SENSOR_MM7_TX1_YAW_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX1_AY);
                ExtractUint8FromCanTelegram(CANMsg.DATA, 8, &temperature, SENSOR_MM7_TX1_TEMP_RATE_Z);

                MM7_C_YAW_RATE = GetAngularRateFromMM7Raw(yawRate);
                MM7_C_AY = GetAccelerationFromMM7Raw(acceleration);
                MM7_C_TEMP = GetTemperatureFromMM7Raw(temperature);
                break;
            }
            case SENSOR_MM7_C_TELEGRAM_2_ID:
            {
                uint16_t rollRate;
                uint16_t acceleration;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &rollRate, SENSOR_MM7_TX2_ROLL_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX2_AX);

                MM7_C_ROLL_RATE = GetAngularRateFromMM7Raw(rollRate);
                MM7_C_AX = GetAccelerationFromMM7Raw(acceleration);
                break;
            }
            case SENSOR_MM7_C_TELEGRAM_3_ID:
            {

                uint16_t pitchRate;
                uint16_t acceleration;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &pitchRate, SENSOR_MM7_TX3_PITCH_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX3_AZ);
                MM7_C_PITCH_RATE = GetAngularRateFromMM7Raw(pitchRate);
                MM7_C_AZ = GetAccelerationFromMM7Raw(acceleration);
                break;
            }
            default:
            {
                break;
            }
            }
            imu.updateIMU(&imuC, MM7_C_PITCH_RATE, -MM7_C_ROLL_RATE, MM7_C_YAW_RATE, -MM7_C_AY, -MM7_C_AX, MM7_C_AZ, .0019); // TODO - NOTE THAT SAMPLE RATE IS TRIPLED (0.0033 INSTEAD OF 0.01) DUE TO GETTING 3 AXISES IN 1 MESSAGE EACH!!

            static uint32_t counter = 0;
            rollRates[counter] = MM7_C_ROLL_RATE;
            pitchRates[counter] = MM7_C_PITCH_RATE;
            yawRates[counter] = MM7_C_YAW_RATE;

            double rollRatesSum = 0, pitchRatesSum = 0, yawRatesSum = 0;
            for (int i = 0; i < NUM_RATES; i++)
            {
                rollRatesSum += rollRates[i];
                pitchRatesSum += pitchRates[i];
                yawRatesSum += yawRates[i];
            }
            rollRateAvg = rollRatesSum / NUM_RATES;
            pitchRateAvg = pitchRatesSum / NUM_RATES;
            yawRateAvg = yawRatesSum / NUM_RATES;

            counter++;
            if (counter > NUM_RATES - 1)
            {
                counter = 0;
            }



            if (MM7_C_ROLL_RATE < rollRateMin)
                rollRateMin = MM7_C_ROLL_RATE;
            if (MM7_C_ROLL_RATE > rollRateMax)
                rollRateMax = MM7_C_ROLL_RATE;
            if (MM7_C_PITCH_RATE < pitchRateMin)
                pitchRateMin = MM7_C_PITCH_RATE;
            if (MM7_C_PITCH_RATE > pitchRateMax)
                pitchRateMax = MM7_C_PITCH_RATE;
            if (MM7_C_YAW_RATE < yawRateMin)
                yawRateMin = MM7_C_YAW_RATE;
            if (MM7_C_YAW_RATE > yawRateMax)
                yawRateMax = MM7_C_YAW_RATE;

            //imu.updateIMU(0,0,0, MM7_C_ROLL_RATE, MM7_C_PITCH_RATE, MM7_C_YAW_RATE, MM7_C_AX, MM7_C_AY, MM7_C_AZ); // TODO - NOTE THAT SAMPLE RATE IS TRIPLED (0.0033 INSTEAD OF 0.01) DUE TO GETTING 3 AXISES IN 1 MESSAGE EACH!!
        }
        // END - GRAB CAN DATA

        // START - IMGUI LOOP
        static uint64_t prevRenderTime = 0;
        const uint64_t renderTimeout = 16;  //  16ms is about 60fps
        static bool renderFrame = true;
        if (Timer(prevRenderTime, renderTimeout, true))
        {
            renderFrame = !renderFrame;
        }
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
        if (renderFrame)
        {
            MSG msg;
            while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
            {
                ::TranslateMessage(&msg);
                ::DispatchMessage(&msg);
                if (msg.message == WM_QUIT)
                    done = true;
            }
            if (done)
                break;

            // Handle window resize (we don't resize directly in the WM_SIZE handler)
            if (g_ResizeWidth != 0 && g_ResizeHeight != 0)
            {
                CleanupRenderTarget();
                g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);
                g_ResizeWidth = g_ResizeHeight = 0;
                CreateRenderTarget();
            }

            // Start the Dear ImGui frame
            ImGui_ImplDX11_NewFrame();
            ImGui_ImplWin32_NewFrame();
            ImGui::NewFrame();


            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_pcan_window)
                //ImGui::ShowDemoWindow(&show_pcan_window);
                ImGui::SetNextWindowSize(ImVec2(350, 350), ImGuiCond_Appearing);
            {
                ImGui::Begin("Dual Track", (bool*)true);
                ImGui::Button("JOYSTICK");
                if (ImGui::IsItemActive())
                    ImGui::GetForegroundDrawList()->AddLine(io.MouseClickedPos[0], io.MousePos, ImGui::GetColorU32(ImGuiCol_Button), 4.0f); // Draw a line between the button and the mouse cursor

                // Drag operations gets "unlocked" when the mouse has moved past a certain threshold
                // (the default threshold is stored in io.MouseDragThreshold). You can request a lower or higher
                // threshold using the second parameter of IsMouseDragging() and GetMouseDragDelta().
                ImVec2 value_raw = ImGui::GetMouseDragDelta(0, 0.0f);
                ImVec2 value_with_lock_threshold = ImGui::GetMouseDragDelta(0);
                ImVec2 mouse_delta = io.MouseDelta;
                //ImGui::Text("GetMouseDragDelta(0):");
                //ImGui::Text("  w/ default threshold: (%.1f, %.1f)", value_with_lock_threshold.x, value_with_lock_threshold.y);
                ImGui::Text("(x,y): (%.1f, %.1f)", value_raw.x, value_raw.y);
                int leftVal = value_raw.x;
                int rightVal = -value_raw.y; // negative so that Y-up is positive

                const float DEG_PER_RAD = 57.2958f;
                double radius = sqrt(leftVal * leftVal + rightVal * rightVal);
                if (radius > 100)
                    radius = 100;
                double theta = atan2(leftVal, rightVal) * DEG_PER_RAD; // X value first because we want up to be 0deg

                if (theta < -180.)
                    theta = -180.;
                if (theta > 180.)
                    theta = 180.;
                ImGui::Text("radius: %f", radius);
                ImGui::Text("theta: %f", theta);

                float leftTrack = 0;
                float rightTrack = 0;
                int index = 0;

                //radius = 100; // TODO - RM: JUST FOCUS ON THETA FOR RIGHT NOW
                //leftTrack = radius * (45 - theta * 90) / 45;
                //rightTrack = 100, 2 * radius + leftTrack;

                if (theta >= 0. && theta <= 90.) // FRONT-RIGHT
                {
                    index = 1;
                    leftTrack = radius;
                    rightTrack = radius * cos(2 * theta / DEG_PER_RAD);
                }
                else if (theta <= 0. && theta >= -90.) // FRONT-LEFT
                {
                    index = 2;
                    leftTrack = radius * cos(2 * theta / DEG_PER_RAD);
                    rightTrack = radius;
                }
                else if (theta >= 90. && theta <= 180.) // BACK-RIGHT
                {
                    index = 3;
                    leftTrack = -radius * cos(2 * theta / DEG_PER_RAD);
                    rightTrack = -radius;
                }
                else if (theta <= -90. && theta >= -180.) // BACK-LEFT
                {
                    index = 4;
                    leftTrack = -radius;
                    rightTrack = -radius * cos(2 * theta / DEG_PER_RAD);
                }

                float leftAbs = abs(leftTrack);
                float rightAbs = abs(rightTrack);
                float thetaCalc = 0;

                if (leftAbs < rightAbs)
                {
                    if (leftTrack < rightTrack)
                    {
                        thetaCalc = 1; // FRONT-LEFT
                        thetaCalc = -DEG_PER_RAD * (acos(leftTrack / rightTrack)) / 2.0;
                    }
                    else
                    {
                        thetaCalc = 2; // BACK-RIGHT
                        thetaCalc = 90.0 + DEG_PER_RAD * (acos(-leftTrack / rightTrack)) / 2.0;
                    }
                }
                else if (rightAbs < leftAbs)
                {
                    if (leftTrack < rightTrack)
                    {
                        thetaCalc = 3; // BACK-LEFT
                        thetaCalc = -90.0 - DEG_PER_RAD * (acos(-rightTrack / leftTrack)) / 2.0;
                    }
                    else
                    {
                        thetaCalc = 4; // FRONT-RIGHT
                        thetaCalc = DEG_PER_RAD * (acos(rightTrack / leftTrack)) / 2.0;
                    }
                }
                else
                {
                    if (leftTrack > 0.0)
                    {
                        if (rightTrack > 0.0)
                        {
                            thetaCalc = 0.; // foward
                        }
                        else
                        {
                            thetaCalc = 90.; // right
                        }
                    }
                    else
                    {
                        if (rightTrack > 0.0)
                        {
                            thetaCalc = -90.; // left
                        }
                        else
                        {
                            thetaCalc = 180; // back
                        }
                    }
                }
                //ImGui::Text("index: %d", index);
                ImGui::Text("thetaCalc: %f", thetaCalc);
                ImGui::Text("LEFT");
                ImGui::SameLine();
                ImGui::Text("   RIGHT");

                // IN ABSOLUTE VALUE
                ImGui::SameLine();
                ImGui::Text("LEFT");
                ImGui::SameLine();
                ImGui::Text("   RIGHT");

                ImGui::VSliderFloat("##int", ImVec2(50, 200), &leftTrack, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &rightTrack, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &leftAbs, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &rightAbs, -100.f, 100.f);
                ImGui::End();
            }

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                ImGui::Begin("PCAN USB");                          // Create a window called "Hello, world!" and append into it.TPCANMsg CANMsg;
                const int NUM_QUAT = 10;
                static bool firstLoop = true;
                static int quatCounter = 0;
                static Quaternion cumulative[NUM_QUAT];
                static Quaternion firstQuaternion;
                Quaternion currQuaternion;
                Quaternion averageQuaternion = { 0,0,0,0 };

                imu.getQuaternion(imuC, &currQuaternion.w, &currQuaternion.x, &currQuaternion.y, &currQuaternion.z);
                cumulative[quatCounter] = imuC;
                if (firstLoop)
                {
                    firstLoop = false;
                    firstQuaternion = imuC;// currQuaternion;
                }
                else
                {
                    averageQuaternion = AverageQuaternion(cumulative, imuC/*currQuaternion*/, firstQuaternion, NUM_QUAT);
                }
                quatCounter++;
                if (quatCounter >= NUM_QUAT)
                {
                    quatCounter = 0;
                }
                //imuC = averageQuaternion;
                imu.setQuaternion(&imuC, currQuaternion.w, currQuaternion.x, currQuaternion.y, currQuaternion.z);

                ImGui::PlotLines("rollRates", rollRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));
                ImGui::PlotLines("yawRates", yawRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));
                ImGui::PlotLines("pitchRates", pitchRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));

                ImGui::Text("YAW RATE: %3.f\tPITCH RATE: %3.f\tROLL RATE: %3.f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
                ImGui::Text("AX:\t%f\tAT:\t%f\tAZ:\t%f\t\n", MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                float gravX, gravY, gravZ;
                imu.getGravityVector(&gravX, &gravY, &gravZ);
                ImGui::Text("gX:\t%f\tgY:\t%f\tgZ:\t%f\t\n", gravX, gravY, gravZ);
                float qW, qX, qY, qZ;
                imu.getQuaternion(imuC, &qW, &qX, &qY, &qZ);
                ImGui::Text("qW: %f\tqX: %f\tqY: %f\tqZ: %f\t\n", qW, qX, qY, qZ);
                ImGui::Text("cqW: %f\tcqX: %f\tcqY: %f\tcqZ: %f\t\n", averageQuaternion.w, averageQuaternion.x, averageQuaternion.y, averageQuaternion.z);
                //static float rollRateMin, rollRateMax, pitchRateMin, pitchRateMax, yawRateMin, yawRateMax;

                ImGui::Text("rollRateMaxDiff: %f\n", rollRateMax - rollRateMin);
                ImGui::Text("pitchRateMaxDiff: %f\n", pitchRateMax - pitchRateMin);
                ImGui::Text("yawRateMaxDiff: %f\n", yawRateMax - yawRateMin);

                ImGui::Text("rollRateMin: %f\n", rollRateMin);
                ImGui::Text("pitchRateMin: %f\n", pitchRateMin);
                ImGui::Text("yawRateMin: %f\n", yawRateMin);

                ImGui::Text("rollRateMax: %f\n", rollRateMax);
                ImGui::Text("pitchRateMax: %f\n", pitchRateMax);
                ImGui::Text("yawRateMax: %f\n", yawRateMax);

                ImGui::Text("rollRateAvg: %f\n", rollRateAvg);
                ImGui::Text("pitchRateAvg: %f\n", pitchRateAvg);
                ImGui::Text("yawRateAvg: %f\n", yawRateAvg);
                // START - 3D DISPLAY

                int matId = 0;
                ImGuizmo::SetID(matId);
                // start - disregard this stuff?
                float matrixTranslation[3], matrixRotation[3], matrixScale[3];
                ImGuizmo::DecomposeMatrixToComponents(objectMatrix[matId], matrixTranslation, matrixRotation, matrixScale);
                ImGui::InputFloat3("Tr", matrixTranslation);
                ImGui::InputFloat3("Rt", matrixRotation);
                ImGui::InputFloat3("Sc", matrixScale);
                ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, objectMatrix[matId]);
                // end - disregard this stuff?
                //imu.updateIMU(MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE, MM7_C_AX, MM7_C_AY, MM7_C_AZ, 0.01); // this is done when we get data
                float yaw;// = imu.getYaw();
                float roll;// = imu.getRoll();
                float pitch;// = imu.getPitch();
                Quaternion grav;// = imu.getPitch();

                imu.computeIMUAngles(imuC, &roll, &pitch, &yaw, &grav);

                //if (MM7_C_AZ < 0)
                //{
                //    if (pitch < 0)
                //        pitch = -180 - pitch;
                //    else
                //        pitch = 180 - pitch;
                //}

                //float yaw = CalculateYawEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                //float roll = CalculateRollEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                //float pitch = CalculatePitchEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);

                ImGui::Text("gX:\t%f\tgY:\t%f\tgZ:\t%f\t\n", grav.x, grav.y, grav.z);

                ImGui::Text("YAW: %f", yaw);
                ImGui::Text("ROLL: %f", roll);
                ImGui::Text("PITCH: %f", pitch);
                matrixRotation[0] = roll;
                matrixRotation[1] = yaw;
                matrixRotation[2] = pitch;

                ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, objectMatrix[matId]);
                static float cameraView[16] =
                { 1.f, 0.f, 0.f, 0.f,
                  0.f, 1.f, 0.f, 0.f,
                  0.f, 0.f, 1.f, 0.f,
                  0.f, 0.f, 0.f, 1.f };

                static int lastUsing = 0;

                static float cameraProjection[16];

                static float fov = 27.f;
                Perspective(fov, io.DisplaySize.x / io.DisplaySize.y, 0.1f, 100.f, cameraProjection);

                static float camYAngle = 165.f / 180.f * 3.14159f;
                static float camXAngle = 32.f / 180.f * 3.14159f;

                bool viewDirty = false;
                static bool firstFrame = true;


                viewDirty |= ImGui::SliderFloat("Distance", &camDistance, 1.f, 10.f);
                if (viewDirty || firstFrame)
                {
                    float eye[] = { cosf(camYAngle) * cosf(camXAngle) * camDistance, sinf(camXAngle) * camDistance, sinf(camYAngle) * cosf(camXAngle) * camDistance };
                    float at[] = { 0.f, 0.f, 0.f };
                    float up[] = { 0.f, 1.f, 0.f };
                    LookAt(eye, at, up, cameraView);
                    firstFrame = false;
                }

                EditTransform(cameraView, cameraProjection, objectMatrix[matId], lastUsing == matId);
                if (ImGuizmo::IsUsing())
                {
                    lastUsing = matId;
                }
                // END - 3D DISPLAY

                ImGui::End();
            }

            static bool show_PID_window = true;
            // 3. Show a PID loop window
            if (show_PID_window)
            {
                ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_Appearing);
                ImGui::Begin("PID Window", &show_PID_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    if (ImGui::Button("Convert DBC to JSON"))
                    {
                        // RUNS DBCC.EXE TO CONVERT OUR `input.dbc` INTO A .JSON FILE!
                        LPCSTR open = "runas";
                        LPCSTR executable = "dbcc.exe";
                        LPCSTR parameters = "-j input.dbc";
                        LPCSTR path = "dbcc\\";
                        //LPCSTR filepath = "dbcc\\putty.exe";
                        printf("%d\n", ShellExecuteA(NULL, open, executable, parameters, path, SW_SHOWNORMAL)); // RETURNS 42 IF GOOD!
                    }
                    if (ImGui::Button("Open Putty!"))
                    {
                        // RUNS DBCC.EXE TO CONVERT OUR `input.dbc` INTO A .JSON FILE!
                        LPCSTR open = "runas";
                        LPCSTR executable = "dbcc\\putty.exe";
                        //LPCSTR path = "dbcc\\";
                        //LPCSTR filepath = "dbcc\\putty.exe";
                        printf("%d\n", ShellExecuteA(NULL, open, executable, NULL, NULL, SW_SHOWNORMAL)); // RETURNS 42 IF GOOD!
                    }
                    if (ImGui::Button("Create an arbitrary C-File!"))
                    {
                        std::string string = "test_check_check";
                        uint8_t iv[] = {1,2};
                        const char path[] = "test.c";
                        uint16_t crc = 1234;
                        long len = string.length();
                        WriteOutFile(string, iv, path, crc, len);
                    }
                    ImGui::Text("Hello from PID window!");
                    static int joystickVal = 0;
                    ImGui::SliderInt("Joystick Val", &joystickVal, SCANRECO_INPUT_MIN, SCANRECO_INPUT_MAX);
                    static int startRampTime = 0;
                    ImGui::SliderInt("Start Ramp Time (ms)", &startRampTime, 0, 5000);
                    static int stopRampTime = 0;
                    ImGui::SliderInt("Stop Ramp Time (ms)", &stopRampTime, 0, 5000);

                    static rampedOutput_ts aux1Vals;
                    aux1Vals.controlModeMult = 100; // percentage to ramp to
                    aux1Vals.deadband = 0;
                    aux1Vals.joystickVal = joystickVal;
                    aux1Vals.posMin_mA = 800;  // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.posMax_mA = 1600; // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.negMin_mA = 800;  // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.negMax_mA = 1600; // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.rampStartTime = startRampTime;
                    aux1Vals.rampStopTime = stopRampTime;
                    RampedOutput(&aux1Vals);
                    int outputFwd;
                    int outputRev;
                    outputFwd = aux1Vals.outputPositive_mA;
                    outputRev = aux1Vals.outputNegative_mA;

                    if (outputFwd)
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(1.f, 0.f, 0.f));
                    }
                    else if (outputRev)
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(0.f, 1.f, 0.f));
                    }
                    else
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(1.f, 1.f, 1.f));
                    }

                    const int NUM_VALUES = 90;
                    static float values[NUM_VALUES] = {};
                    static int values_offset = 0;
                    float average = 0.0f;
                    static int counter = 0;

                    static uint64_t prevUpdateVal = 0;
                    const uint64_t updateValTimeout = 50;
                    if (io.MousePos.y > -10000 && io.MousePos.y < 10000)
                    {
                        if (Timer(prevUpdateVal, updateValTimeout, true))
                        {
                            if (outputFwd)
                            {
                                values[counter] = outputFwd;
                            }
                            else
                            {
                                values[counter] = outputRev;
                            }
                            values[counter] = aux1Vals.setpoint;
                            if (counter < NUM_VALUES - 1)
                            {
                                counter++;
                            }
                            // push everything back
                            if (counter == NUM_VALUES - 1)
                            {
                                for (int i = 1; i < NUM_VALUES; i++)
                                {
                                    values[i - 1] = values[i];
                                }
                            }
                        }
                    }
                    for (int n = 0; n < IM_ARRAYSIZE(values); n++)
                        average += values[n];
                    average /= (float)IM_ARRAYSIZE(values);
                    char overlay[32];
                    sprintf(overlay, "val %.0f", values[counter]);

                    if (ImGui::BeginTable("table_nested1", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                    {
                        ImGui::TableSetupColumn("Graph");
                        ImGui::TableSetupColumn("Values");
                        ImGui::TableHeadersRow();

                        ImGui::TableNextColumn();

                        float minVal = -127;// 0;// getMin(values, NUM_VALUES);
                        float maxVal = 127;// aux1Vals.posMax_mA;// getMax(values, NUM_VALUES);
                        const float PLOT_HEIGHT = 160.0f;
                        ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, minVal, maxVal, ImVec2(0, PLOT_HEIGHT));


                        //ImGui::Text("A0 Row 0");
                        ImGui::TableNextColumn();
                        //ImGui::Text("A1 Row 0");
                        {
                            float rows_height = 80 / 3;
                            if (ImGui::BeginTable("table_nested2", 1, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                            {
                                ImGui::TableSetupColumn("B0");

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("max: %d", (int)maxVal);

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                if (counter < NUM_VALUES - 1)
                                {
                                    ImGui::Text("val: %d", (int)values[counter - 1]);
                                }
                                else
                                {
                                    ImGui::Text("val: %d", (int)values[counter]);
                                }

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("min: %d", (int)minVal);

                                ImGui::EndTable();
                            }
                        }
                        //ImGui::TableNextColumn(); ImGui::Text("A0 Row 1");
                        //ImGui::TableNextColumn(); ImGui::Text("A1 Row 1");
                        ImGui::EndTable();
                    }
                    ImGui::PopStyleColor();
                    //ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, getMin(values, NUM_VALUES), getMax(values, NUM_VALUES), ImVec2(0, 80.0f));
                    //ImGui::SameLine();
                    ImGui::Text("max");
                }
                if (ImGui::Button("Close Me"))
                    show_PID_window = false;
                ImGui::End();
            }







            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_demo_window)
                ImGui::ShowDemoWindow(&show_demo_window);

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                static float f = 0.0f;
                static int counter = 0;

                ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

                ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
                ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
                ImGui::Checkbox("Another Window", &show_another_window);

                ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
                ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

                if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                    counter++;
                ImGui::SameLine();
                ImGui::Text("counter = %d", counter);

                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
                ImGui::End();
            }

            // 3. Show another simple window.
            if (show_another_window)
            {
                ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                ImGui::Text("Hello from another window!");
                if (ImGui::Button("Close Me"))
                    show_another_window = false;
                ImGui::End();
            }

            // Rendering
            ImGui::Render();
            const float clear_color_with_alpha[4] = { clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w };
            g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
            g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

            //g_pSwapChain->Present(1, 0); // Present with vsync
            g_pSwapChain->Present(0, 0); // Present without vsync
        }
        renderFrame = false;
        // END - IMGUI LOOP
    }

    // Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);

    return 0;
}

// Helper functions

bool CreateDeviceD3D(HWND hWnd)
{
    // Setup swap chain
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
    //createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
    HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res == DXGI_ERROR_UNSUPPORTED) // Try high-performance WARP software driver if hardware is not available.
        res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res != S_OK)
        return false;

    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D()
{
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget()
{
    ID3D11Texture2D* pBackBuffer;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
    pBackBuffer->Release();
}

void CleanupRenderTarget()
{
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg)
    {
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED)
            return 0;
        g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
        g_ResizeHeight = (UINT)HIWORD(lParam);
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}



/**
 * @brief Returns angular rate in degrees/s, LSB = 0.005 deg/s
 *
 * @param[in] input raw value from MM7 sensor (YAW_RATE, ROLL_RATE, PITCH_RATE)
 * @return angular rate in deg/s
 */
float GetAngularRateFromMM7Raw(uint16_t input)
{
    if (input == 0xFFFF)
    {
        return -999.9F; // Sensor will never send 0xFFFF except in error state, output an equally outrageous value
    }
    float output = (input - 0x8000) * 0.005;
    return output;
}

/**
 * @brief Returns acceleration in m/s^2, LSB = 0.00125 m/s^2
 *
 * @param[in] input raw value from MM7 sensor (AX, AY, AZ)
 * @return Acceleration in m/s^2
 */
float GetAccelerationFromMM7Raw(uint16_t input)
{
    if (input == 0xFFFF)
    {
        return -999.9F; // Sensor will never send 0xFFFF except in error state, output an equally outrageous value
    }
    float output = (input - 0x8000) * 0.00125;
    return output;
}

/**
 * @brief Returns temperature of MM7 sensor value in degrees Celsius
 *
 * @param[in] input raw value from MM7 sensor (TEMP_RATE_Z)
 * @return temperature in degrees Celsius
 */
int16_t GetTemperatureFromMM7Raw(uint8_t input)
{
    if (input == 0xFF)
    {
        return -999; // 0xFF means CRC-error or temperature is below -50C
    }
    if (input == 0xC9)
    {
        return 999; // 0xC9 means temperature is above 150C
    }
    int16_t output = input - 50;
    return output;
}

/**
 * @brief Extracts a value of type bool at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractBoolFromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, bool* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = (telegram[spnConfig.byte] >> spnConfig.bit) & mask;
    if (val)
        *output = TRUE;
    else
        *output = FALSE;
    return 0;
}

/**
 * @brief Extracts a value of type uint16 at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractUint16FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint16_t* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = 0;
    uint8_t numBytes = 1 + (spnConfig.bit + spnConfig.len) / 8;
    i = 0;
    for (i; i < numBytes; i++)
    {
        val |= telegram[spnConfig.byte + i] << (8 * i);
    }
    *output = (val >> spnConfig.bit) & mask;
    return 0;
}

/**
 * @brief Extracts a value of type uint8 at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractUint8FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint8_t* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = 0;
    uint8_t numBytes = 1 + (spnConfig.bit + spnConfig.len) / 8;
    i = 0;
    for (i; i < numBytes; i++)
    {
        val |= telegram[spnConfig.byte + i] << (8 * i);
    }
    *output = (val >> spnConfig.bit) & mask;
    return 0;
}


void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition)
{
    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
    static bool useSnap = false;
    static float snap[3] = { 1.f, 1.f, 1.f };
    static float bounds[] = { -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f };
    static float boundsSnap[] = { 0.1f, 0.1f, 0.1f };
    static bool boundSizing = false;
    static bool boundSizingSnap = false;

    if (editTransformDecomposition)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_T))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        if (ImGui::IsKeyPressed(ImGuiKey_E))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) // r Key
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::RadioButton("Universal", mCurrentGizmoOperation == ImGuizmo::UNIVERSAL))
            mCurrentGizmoOperation = ImGuizmo::UNIVERSAL;
        float matrixTranslation[3], matrixRotation[3], matrixScale[3];
        //ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);
        //ImGui::InputFloat3("Tr", matrixTranslation);
        //ImGui::InputFloat3("Rt", matrixRotation);
        //ImGui::InputFloat3("Sc", matrixScale);
        //ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

        if (mCurrentGizmoOperation != ImGuizmo::SCALE)
        {
            if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
                mCurrentGizmoMode = ImGuizmo::LOCAL;
            ImGui::SameLine();
            if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
                mCurrentGizmoMode = ImGuizmo::WORLD;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_S))
            useSnap = !useSnap;
        ImGui::Checkbox("##UseSnap", &useSnap);
        ImGui::SameLine();

        switch (mCurrentGizmoOperation)
        {
        case ImGuizmo::TRANSLATE:
            ImGui::InputFloat3("Snap", &snap[0]);
            break;
        case ImGuizmo::ROTATE:
            ImGui::InputFloat("Angle Snap", &snap[0]);
            break;
        case ImGuizmo::SCALE:
            ImGui::InputFloat("Scale Snap", &snap[0]);
            break;
        }
        ImGui::Checkbox("Bound Sizing", &boundSizing);
        if (boundSizing)
        {
            ImGui::PushID(3);
            ImGui::Checkbox("##BoundSizing", &boundSizingSnap);
            ImGui::SameLine();
            ImGui::InputFloat3("Snap", boundsSnap);
            ImGui::PopID();
        }
    }

    ImGuiIO& io = ImGui::GetIO();
    float viewManipulateRight = io.DisplaySize.x;
    float viewManipulateTop = 0;
    static ImGuiWindowFlags gizmoWindowFlags = 0;
    if (useWindow)
    {
        ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_Appearing);
        ImGui::SetNextWindowPos(ImVec2(400, 20), ImGuiCond_Appearing);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImVec4)ImColor(0.35f, 0.3f, 0.3f));
        ImGui::Begin("Gizmo", 0, gizmoWindowFlags);
        ImGuizmo::SetDrawlist();
        float windowWidth = (float)ImGui::GetWindowWidth();
        float windowHeight = (float)ImGui::GetWindowHeight();
        ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, windowWidth, windowHeight);
        viewManipulateRight = ImGui::GetWindowPos().x + windowWidth;
        viewManipulateTop = ImGui::GetWindowPos().y;
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        gizmoWindowFlags = ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(window->InnerRect.Min, window->InnerRect.Max) ? ImGuiWindowFlags_NoMove : 0;
    }
    else
    {
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    }

    ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);
    ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], gizmoCount);
    ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, matrix, NULL, useSnap ? &snap[0] : NULL, boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);

    ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);

    if (useWindow)
    {
        ImGui::End();
        ImGui::PopStyleColor(1);
    }
}


void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16)
{
    float ymax, xmax;
    ymax = znear * tanf(fovyInDegrees * 3.141592f / 180.0f);
    xmax = ymax * aspectRatio;
    Frustum(-xmax, xmax, -ymax, ymax, znear, zfar, m16);
}


void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16)
{
    float temp, temp2, temp3, temp4;
    temp = 2.0f * znear;
    temp2 = right - left;
    temp3 = top - bottom;
    temp4 = zfar - znear;
    m16[0] = temp / temp2;
    m16[1] = 0.0;
    m16[2] = 0.0;
    m16[3] = 0.0;
    m16[4] = 0.0;
    m16[5] = temp / temp3;
    m16[6] = 0.0;
    m16[7] = 0.0;
    m16[8] = (right + left) / temp2;
    m16[9] = (top + bottom) / temp3;
    m16[10] = (-zfar - znear) / temp4;
    m16[11] = -1.0f;
    m16[12] = 0.0;
    m16[13] = 0.0;
    m16[14] = (-temp * zfar) / temp4;
    m16[15] = 0.0;
}


void LookAt(const float* eye, const float* at, const float* up, float* m16)
{
    float X[3], Y[3], Z[3], tmp[3];

    tmp[0] = eye[0] - at[0];
    tmp[1] = eye[1] - at[1];
    tmp[2] = eye[2] - at[2];
    Normalize(tmp, Z);
    Normalize(up, Y);

    Cross(Y, Z, tmp);
    Normalize(tmp, X);

    Cross(Z, X, tmp);
    Normalize(tmp, Y);

    m16[0] = X[0];
    m16[1] = Y[0];
    m16[2] = Z[0];
    m16[3] = 0.0f;
    m16[4] = X[1];
    m16[5] = Y[1];
    m16[6] = Z[1];
    m16[7] = 0.0f;
    m16[8] = X[2];
    m16[9] = Y[2];
    m16[10] = Z[2];
    m16[11] = 0.0f;
    m16[12] = -Dot(X, eye);
    m16[13] = -Dot(Y, eye);
    m16[14] = -Dot(Z, eye);
    m16[15] = 1.0f;
}

void Normalize(const float* a, float* r)
{
    float il = 1.f / (sqrtf(Dot(a, a)) + FLT_EPSILON);
    r[0] = a[0] * il;
    r[1] = a[1] * il;
    r[2] = a[2] * il;
}

float Dot(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross(const float* a, const float* b, float* r)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}
