#include "TimersAndCalculations.h"

//uint64_t PROGRAM_START = 0;
auto startTime = std::chrono::steady_clock::now(); //steady clock is great for timers, not great for epoch

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
 * @return If no errors, returns resistance in ohms. If error, returns `-1` (very large unsigned number)
 */
double scale(double input, double minIn, double maxIn, double minOut, double maxOut, bool clipOutput)
{
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

// curve MUST be a function (only 1 output for a given input)
double scaleToCurve(CURVE_T curve, double input, bool clipOutput)
{
    int indexFound = -1;
    int curveTDirection = 0; // 0 means neither increasing or decreasing

    // check direction of curve.t (increasing or decreasing)// first check if we are increasing or decreasing between indices
    if (curve.t[1] > curve.t[0]) // curve.t increases as index increases
    {
        curveTDirection = 1;
    }
    else if (curve.t[1] < curve.t[0]) // curve.t decreases as index increases
    {
        curveTDirection = -1;
    }

    // check bounds before anything else
    if (curveTDirection > 0)
    {
        if (input < curve.t[0])
        {
            indexFound = 1;
        }
        else if (input > curve.t[curve.length - 1])
        {
            indexFound = curve.length - 1;
        }
    }
    else if (curveTDirection < 0)
    {
        if (input > curve.t[0])
        {
            indexFound = 1;
        }
        else if (input < curve.t[curve.length - 1])
        {
            indexFound = curve.length - 1;
        }
    }

    // check inside curve so we can linearly interpolate (only if we are within curve.t bounds!)
    if (indexFound == -1)
    {
        int i;
        for (i = 1; i < curve.length; i++)
        {
            // first check if we are increasing or decreasing between indices
            if (curveTDirection > 0) // curve.t increases as index increases
            {
                if (input <= curve.t[i] && input >= curve.t[i - 1])
                {
                    indexFound = i;
                    break;
                }
            }
            else if (curveTDirection < 0) // curve.t decreases as index increases
            {
                if (input >= curve.t[i] && input <= curve.t[i - 1])
                {
                    indexFound = i;
                    break;
                }
            }
        }
    }

    if (indexFound != -1) // make sure we found something
    {
        double output = curve.y[0];
        double maxIn = curve.t[indexFound];
        double minIn = curve.t[indexFound - 1];
        double maxOut = curve.y[indexFound];
        double minOut = curve.y[indexFound - 1];
        output = scale(input, minIn, maxIn, minOut, maxOut, clipOutput); // yes, input/output ranges are supposed to be the same
        return output;
    }
    return 0; // return 0 if we never found an index
}
//{
//    int i = 0;
//    int indexFound = 0;
//    for (i = 1; i < curve->length; i++)
//    {
//        if ((curve->t[i] < input) && (curve->t[i - 1] > input))
//        {
//            indexFound = i;
//            break;
//        }
//    }
//
//    double output = curve->y[0];
//    double maxOut = curve->y[indexFound];
//    double minOut = curve->y[indexFound - 1];
//    output = scale(input, minIn, maxIn, minOut, maxOut, true);
//    return output;
//}

//  COUNTS HOW MANY BITS ARE EQUAL TO 1 IN A VARIABLE WITH LENGTH UP TO 32 BITS
//  SHAMELESSLY STOLEN FROM THE INTERNET
int NumberOfSetBits(uint32_t i)
{
    // Java: use int, and use >>> instead of >>. Or use Integer.bitCount()
    // C or C++: use uint32_t
    i = i - ((i >> 1) & 0x55555555);                // add pairs of bits
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333); // quads
    i = (i + (i >> 4)) & 0x0F0F0F0F;                // groups of 8
    return (i * 0x01010101) >> 24;                  // horizontal sum of bytes
}

int ThreeWayXOR(int a, int b, int c)
{
    if ((bool(a) + bool(b) + bool(c)) == 1) // convert to bool and check if there is a single input that's true
    {
        if (a)
            return 1;
        if (b)
            return 2;
        if (c)
            return 3;
    }
    return 0;
}

float fsc_fabs(float f)
{
    if (f < 0)
        return -f;
    return f;
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    union
    {
        float f;
        long i;
    } conv = { x };
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    return conv.f;
}

float fsc_sqrt(float x)
{
    return (1.0f / invSqrt(x)); // This is probably the fastest way to approximate this...
}

float fsc_asinf(float x)
{
    // https://developer.download.nvidia.com/cg/asin.html
    float negate = (x < 0) ? -1.0f : 1.0f;
    x = fsc_fabs(x);
    float ret = -0.0187293;
    ret *= x;
    ret += 0.0742610;
    ret *= x;
    ret -= 0.2121144;
    ret *= x;
    ret += 1.5707288;
    ret = PI_2 - fsc_sqrt(1.0 - x) * ret;
    return ret * negate;
}

float fsc_atan2f(float y, float x)
{
    // http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    // https://dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization/
    const float ONEQTR_PI = PI / 4.0;
    const float THRQTR_PI = 3.0 * PI / 4.0;
    float r, angle;
    float abs_y = fsc_fabs(y) + 1e-10f; // kludge to prevent 0/0 condition
    if (x < 0.0f)
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI;
    }
    else
    {
        r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI;
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    if (y < 0.0f)
        return (-angle); // negate if in quad III or IV
    else
        return (angle);
}

long long epoch()
{
    long long milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return milliseconds_since_epoch;
}

long long epochMillis()
{
    //returning epoch from 1970
    long long milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return milliseconds_since_epoch;
}

//return a 64 bit millis() since the program started. This makes the output very similar to (not) Arduino's hours() function
uint64_t hours()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::hours>(now - startTime).count();
    uint64_t ms = (uint64_t)now_ms;
    return ms;
}

//return a 64 bit millis() since the program started. This makes the output very similar to (not) Arduino's minutes() function
uint64_t minutes()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::minutes>(now - startTime).count();
    uint64_t ms = (uint64_t)now_ms;
    return ms;
}

//return a 64 bit millis() since the program started. This makes the output very similar to (not) Arduino's seconds() function
uint64_t seconds()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    uint64_t ms = (uint64_t)now_ms;
    return ms;
}

//return a 64 bit millis() since the program started. This makes the output very similar to Arduino's millis() function
uint64_t millis()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    uint64_t ms = (uint64_t)now_ms;
    return ms;
}

//return a 64 bit micros() since the program started. This makes the output very similar to Arduino's micros() function
uint64_t micros()
{
    auto now = std::chrono::steady_clock::now();
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count();
    uint64_t us = (uint64_t)now_us;
    return us;
}

//return a 64 bit nanos() since the program started. This makes the output very similar to (not) Arduino's nanos() function
uint64_t nanos()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startTime).count();
    uint64_t ns = (uint64_t)now_ns;
    return  ns;
}

// Timer that measures in millis and triggers on or after (timeout + prevTime)
bool Timer(uint64_t& prevTime, uint64_t timeout, bool resetPrevTime, uint64_t current_time, bool useFakeMillis)
{
    if (!useFakeMillis) //  IF WE DON'T SAY TO USE OUR GIVEN MILLIS, LET'S CHECK MILLIS OURSELVES!
    {
        current_time = millis();
    }
    if ((current_time - prevTime) >= timeout) //typecast to create a massive positive number when current_time rolls over to 0
    {
        if (resetPrevTime)
        {
            prevTime = millis();
        }
        return true;
    }
    return false;
}

double ConvertToFahrenheit(double celsius)
{
    return (celsius * 9.0 / 5.0) + 32.0;
}

//  BuildCrc16: USES THE CRC CHECKSUM ALGORITHM THAT EKM METERING USES.  WE ARE NOT USING EKM METERING FOR FUSION (YET), BUT IT IS CONSISTENT WITH WHAT WE HAVE USED AND WE KNOW IT WORKS
uint16_t TimersAndCalculations::BuildCrc16(const char* input, long input_len)
{
    uint16_t crc = 0xFFFF;
    if (input_len > 0)
    {
        //printf("char array for CRC:");
        for (long i = 1; i < input_len; i++) // first uint8_t never used for CRC calc in EKM
        {
            //printf("%d", input[i]);
            uint8_t index = (crc ^ input[i]) & 0xFF;
            uint16_t temp_crc = CRCTABLE[index];
            crc = (crc >> 8) ^ temp_crc;
        }
        //printf("\n");
    }
    crc = (crc << 8) | (crc >> 8);
    crc &= 0x7F7F;
    return crc;
}

//  BuildCrc16: USES THE CRC CHECKSUM ALGORITHM THAT EKM METERING USES.  WE ARE NOT USING EKM METERING FOR FUSION (YET), BUT IT IS CONSISTENT WITH WHAT WE HAVE USED AND WE KNOW IT WORKS
uint16_t TimersAndCalculations::BuildCrc16(char input[], uint8_t input_len)
{
    uint16_t crc = 0xFFFF;
    if (input_len > 0)
    {
        printf("\nByte Array for CRC:");
        for (uint8_t i = 1; i < input_len; i++) // first uint8_t never used for CRC calc in EKM
        {
            printf("%d, ", input[i]);
            uint8_t index = (crc ^ input[i]) & 0xFF;
            uint16_t temp_crc = CRCTABLE[index];
            crc = (crc >> 8) ^ temp_crc;
        }
        printf("\n");
    }
    crc = (crc << 8) | (crc >> 8);
    crc &= 0x7F7F;
    return crc;
}

//  BuildCrc8: USES THE 16-BIT CRC CHECKSUM ALGORITHM THAT EKM METERING USES AND ONLY TAKING THE LSB, THUS MAKING IT 8-BIT.  CRUDE BUT SEEMS TO WORK FINE FOR WHAT WE WANT.
uint8_t TimersAndCalculations::BuildCrc8(char input[], uint8_t input_len)
{
    uint16_t crc = 0xFFFF;
    if (input_len > 0)
    {
        for (uint8_t i = 1; i < input_len; i++) // first uint8_t never used for CRC calc in EKM
        {
            uint8_t index = (crc ^ input[i]) & 0xFF;
            uint16_t temp_crc = CRCTABLE[index];
            crc = (crc >> 8) ^ temp_crc;
        }
    }
    crc = (crc << 8) | (crc >> 8);
    crc &= 0x7F7F;
    uint8_t result = crc & 0xFF;    //  ONLY TAKE THE LSB - THE REALLY POOR MAN'S WAY OF MAKING A 8-BIT CRC
    return crc;
}

//  TimeUntilTimer:  USES MILLIS TO COUNT THINGS, RETURNS THE AMOUNT OF TIME LEFT UNTIL startTime + timeout, AUTO-RESETING IF resetPrevTime = true
//  OVERFLOW-FRIENDLY! :D
//  currentTime is optional, use for applications where timing doesn't have to be exact or when we have to repeat a lot of timing functions using
//  approximately the same current time
uint32_t TimersAndCalculations::TimeUntilTimer(uint32_t& startTime, uint32_t timeout, bool resetPrevTime, uint32_t current_time, bool useFakeMillis)
{
    uint32_t endTime = startTime + timeout;	   //  IF THIS OVERFLOWS, THAT'S OKAY
    uint32_t timeLeft = 0;
    if (!useFakeMillis)	   //  IF WE DON'T SAY TO USE OUR GIVEN MILLIS, LET'S CHECK MILLIS OURSELVES!
    {
        current_time = millis();
    }
    if ((uint32_t)(current_time - startTime) > timeout)	   // typecast to create a massive positive number when current_time rolls over to 0
    {
        if (resetPrevTime)
        {
            startTime = current_time;
        }
    }
    else
    {
        timeLeft = endTime - current_time;
    }
    return timeLeft;
}

float getMin(float arr[], size_t lenArray)
{
    float output = arr[0];
    for (size_t i = 1; i < lenArray; i++)
    {
        if (arr[i] < output)
        {
            output = arr[i];
        }
    }
    return output;
}

float getMax(float arr[], size_t lenArray)
{
    float output = arr[0];
    for (size_t i = 1; i < lenArray; i++)
    {
        if (arr[i] > output)
        {
            output = arr[i];
        }
    }
    return output;
}
