#pragma once
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>


static float _err_measure;
static float _err_estimate;
static float _q;
static float _current_estimate = 0;
static float _last_estimate = 0;
static float _kalman_gain = 0;

float updateEstimate(float mea);
