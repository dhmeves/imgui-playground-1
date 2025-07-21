#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include "TimersAndCalculations.h"
#include "motionController_2D.h"

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

typedef struct {
    float position;
    float velocity;
    float acceleration;
} MotionState;

typedef struct {
    float j_max;
    float a_max;
    float v_max;
    float total_time;
    float t[7]; // t[0] to t[6] are duration of the 7 S-curve phases
} ScurveProfile;

typedef struct {
    float current_time;
    float dt;
    MotionState state;
    ScurveProfile profile;
    float direction;
} ScurveGenerator;

const int NUM_LINKS = 3;

//void scurve_step(ScurveGenerator* gen);
//void compute_scurve_profile(ScurveProfile* p, float distance, float j_max, float a_max, float v_max);

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
        float x; // cartesian coordiantes
        float y; // cartesian coordiantes
        float angle; /// angle of joint relative to parent arm in degrees
        angularConstraint_ts angularConstraint;/// The angular constraint of the joint
    };

    struct Arm
    {
        Joint joints[NUM_LINKS + 1];
        float gripperAngle;
        float prevTargetX;
        float prevTargetY;
        polygon_ts goZone; // Polygon where arm is allowed to go
    };

    void fsciks_init(Arm* arm);

    IK_CONVERGENCE_E calcP1(Arm* arm);
    IK_CONVERGENCE_E calcP2(Arm* arm);
    IK_CONVERGENCE_E calcArm(Arm* arm);
    float getAngle(Arm arm, unsigned int joint);

    void precalcPolygonValues(polygon_ts polygon);
    bool pointInPolygon(polygon_ts polygon, int num_points, point_ts p);

    float dist2(point_ts a, point_ts b);
    float point_to_segment_distance(point_ts p, point_ts v, point_ts w);
    float distance_to_polygon(polygon_ts polygon, int num_points, point_ts p);

    bool check_arm_angle(Arm arm, unsigned int joint);
    bool check_arm_angles(Arm arm);
    IK_CONVERGENCE_E validate_and_constrain_target(Arm arm, mc2D_vec2_t* target, float angle_buffer);

private:
    float  constant[NUM_POLYGON_CORNERS]; //storage for precalculated constants
    float  multiple[NUM_POLYGON_CORNERS]; //storage for precalculated multipliers
};
