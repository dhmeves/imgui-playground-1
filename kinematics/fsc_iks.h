#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include "TimersAndCalculations.h"

typedef enum IK_CONVERGENCE_E
{
    CONVERGES,
    CONVERGENCE_NOT_POSSIBLE
};

typedef struct FSCIKS_T
{

} fsciks_ts;

const int NUM_LINKS = 3;

class Fsciks
{
public:
    struct AngularConstraint 
    {
        float min_angle; // The minimum angle of in degrees
        float max_angle; // The maximum angle in degrees
    };

    struct PolarCoords
    {
    };

    /// Joint struct
    struct Joint 
    {
        float length; // length from this joint to previous

        float r; // polar coordinates
        float theta; // polar coordinates
        float x; // cartesian coordiantes
        float y; // cartesian coordiantes
        float angle; /// angle of joint relative to parent arm in degrees
        AngularConstraint constraint;/// The angular constraint of the joint
    };

    struct Arm
    {
        Joint joints[NUM_LINKS + 1];
        float gripperAngle;
        float prevTargetX;
        float prevTargetY;
    };

    void fsciks_init(Arm* arm);

    IK_CONVERGENCE_E calcP1(Arm* arm);
    IK_CONVERGENCE_E calcP2(Arm* arm);
    float getAngle(Arm arm, unsigned int joint);

private:
};
