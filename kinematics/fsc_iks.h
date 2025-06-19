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
        float targetX;
        float targetY;
        float prevTargetX;
        float prevTargetY;
    };

    //int GripperAngle;
    //int linkStrokeW[] = { 28, 18, 12 };
    //int linkColor[] = { #00D000, #0000FF, #FF0000 };
    //int lengths[NUM_LINKS] = { 0, 39, 36, 10 };
    //int h0 = 180; //Arm Origen in Screen Coordinates
    //int v0 = 360; //Arm Origen in Screen Coordinates
    //int currGripper = 0;

    //float x[NUM_LINKS]; //horizontal coordinate, corresponds to J0, J1, J2, J3, etc
    //float y[NUM_LINKS]; //vertical coordinate
    //float a[NUM_LINKS];  //angle for the link, reference is previous link
    //float tx, ty; //target coordinate for the actuator (gripper)
    //float tx0, ty0; //Previous target coordinate for the actuator (gripper) that was inside the robot reach
    //float l12;
    //float a12;

    void fsciks_init(Arm* arm);

    IK_CONVERGENCE_E calcP1(Arm* arm);
    IK_CONVERGENCE_E calcP2(Arm* arm);
private:
};
