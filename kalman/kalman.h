#pragma once
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>

typedef struct
{
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate = 0;
    float last_estimate = 0;
    float kalman_gain = 0;
} kalman_ts;


class Kalman
{
public:

    float updateEstimate(float mea, kalman_ts * filter);
};
