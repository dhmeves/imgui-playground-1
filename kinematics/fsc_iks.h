#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include "TimersAndCalculations.h"

const int NUM_POLYGON_CORNERS = 20;// how many corners the polygon has

typedef enum IK_CONVERGENCE_E
{
    CONVERGES,
    CONVERGENCE_NOT_POSSIBLE
};

typedef struct FSCIKS_T
{
} fsciks_ts;

typedef struct point_ts
{
    float x;
    float y;
};

typedef struct polygon_ts
{
    point_ts pnt[NUM_POLYGON_CORNERS];
};

const int NUM_LINKS = 3;

class Fsciks
{
public:
    struct angularConstraint_ts
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
        float length; // length from this joint to previous joint

        float r; // polar coordinates
        float theta; // polar coordinates
        point_ts point;
        float angle; /// angle of joint relative to parent arm in degrees
        angularConstraint_ts angularConstraint;/// The angular constraint of the joint
    };

    struct Arm
    {
        Joint joints[NUM_LINKS + 1];
        float gripperAngle;
        point_ts prevTarget;
        point_ts target;
        polygon_ts goZone; // Polygon where arm is allowed to go
    };

    void fsciks_init(Arm* arm);

    IK_CONVERGENCE_E calcP1(Arm* arm);
    IK_CONVERGENCE_E calcP2(Arm* arm);
    IK_CONVERGENCE_E calcArm(Arm* arm);
    float getAngle(Arm arm, unsigned int joint);

    void precalcPolygonValues(polygon_ts polygon);
    bool pointInPolygon(point_ts point, polygon_ts polygon, int num_points);

    float dist2(point_ts a, point_ts b);
    float distPointToSegment(point_ts point, point_ts a, point_ts b);
    float distPointToPolygon(point_ts point, polygon_ts polygon, int num_points);

private:
    float  constant[NUM_POLYGON_CORNERS]; //storage for precalculated constants
    float  multiple[NUM_POLYGON_CORNERS]; //storage for precalculated multipliers
};
