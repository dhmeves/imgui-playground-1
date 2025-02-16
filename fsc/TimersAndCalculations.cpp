#include "TimersAndCalculations.h"

//uint64_t PROGRAM_START = 0;
auto startTime = std::chrono::steady_clock::now(); //steady clock is great for timers, not great for epoch

bool ThreeWayXOR(int a, int b, int c)
{
    return (bool(a) + bool(b) + bool(c)) == 1; // convert to bool and check if there is a single input that's true
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
    if ((uint64_t)(current_time - prevTime) >= timeout) //typecast to create a massive positive number when current_time rolls over to 0
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
