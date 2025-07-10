#include "fsc_iks.h"

void Fsciks::fsciks_init(Arm* arm)
{
    // setup fsciks here
}

IK_CONVERGENCE_E Fsciks::calcP2(Arm* arm)
{
    arm->joints[2].x = arm->joints[3].x - cos(arm->gripperAngle) * arm->joints[3].length;
    arm->joints[2].y = arm->joints[3].y - sin(arm->gripperAngle) * arm->joints[3].length;
    arm->joints[2].r = sqrt(fsc_sqf(arm->joints[2].x) + fsc_sqf(arm->joints[2].y));
    if (arm->joints[2].r > arm->joints[0].length + arm->joints[1].length + arm->joints[2].length) // check if solution possible TODO - RM: THIS WILL NOT WORK WITH ANGLE CONSTRAINTS!
    {
        return CONVERGENCE_NOT_POSSIBLE;
    }
    return CONVERGES;
}

IK_CONVERGENCE_E Fsciks::calcP1(Arm* arm)
{
    arm->joints[2].theta = atan2(arm->joints[2].y, arm->joints[2].x);
    arm->joints[1].angle = acos((fsc_sqf(arm->joints[1].length) + fsc_sqf(arm->joints[2].r) - fsc_sqf(arm->joints[2].length)) 
        / (2 * arm->joints[1].length * arm->joints[2].r)) + arm->joints[2].theta;
    arm->joints[1].x = arm->joints[1].length * cos(arm->joints[1].angle);
    arm->joints[1].y = arm->joints[1].length * sin(arm->joints[1].angle);
    return CONVERGES;
}

IK_CONVERGENCE_E Fsciks::calcArm(Arm* arm)
{
    IK_CONVERGENCE_E p2Converges = calcP2(arm);
    if (p2Converges)
    {
        calcP1(arm);
    }
    else
    {
        return CONVERGENCE_NOT_POSSIBLE;
    }
    return CONVERGES;
}

float Fsciks::getAngle(Arm arm, unsigned int joint)
{
    if (joint > NUM_LINKS)
    {
        return 0; // don't overstep our array bounds yo
    }
    if (joint == 0)
    {
        return atan2(arm.joints[1].y - arm.joints[0].y, arm.joints[1].x - arm.joints[0].x);
    }
    return atan2(arm.joints[joint + 1].y - arm.joints[joint].y, arm.joints[joint + 1].x - arm.joints[joint].x)
        - atan2(arm.joints[joint].y - arm.joints[joint - 1].y, arm.joints[joint].x - arm.joints[joint - 1].x);
}

void Fsciks::precalcPolygonValues(polygon_ts polygon)
{
    int   i;
    int j = NUM_POLYGON_CORNERS - 1;
    for (i = 0; i < NUM_POLYGON_CORNERS; i++)
    {
        if (polygon.y[j] == polygon.y[i])
        {
            constant[i] = polygon.x[i];
            multiple[i] = 0;
        }
        else
        {
            constant[i] = polygon.x[i] - (polygon.y[i] * polygon.x[j]) / (polygon.y[j] - polygon.y[i]) + (polygon.y[i] * polygon.x[i]) / (polygon.y[j] - polygon.y[i]);
            multiple[i] = (polygon.x[j] - polygon.x[i]) / (polygon.y[j] - polygon.y[i]);
        }
        j = i;
    }
}

bool Fsciks::pointInPolygon(polygon_ts polygon, float x, float y)
{

    bool oddNodes = false, current = polygon.y[NUM_POLYGON_CORNERS - 1] > y, previous;
    for (int i = 0; i < NUM_POLYGON_CORNERS; i++)
    {
        previous = current; current = polygon.y[i] > y; if (current != previous) oddNodes ^= y * multiple[i] + constant[i] < x;
    }
    return oddNodes;
}

void compute_scurve_profile(ScurveProfile* p, float distance, float j_max, float a_max, float v_max)
{
    // Assume symmetric profile
    p->j_max = j_max;
    p->a_max = a_max;
    p->v_max = v_max;

    float tj = a_max / j_max; // Time to ramp acceleration
    float ta = tj;            // For symmetric accel/decel
    float tv = 0.0f;          // Cruise duration

    float d_a = 2.0f * (a_max * tj + 0.5f * j_max * tj * tj * tj); // Distance during accel + decel

    // If distance too short for cruise
    if (distance < d_a) {
        tj = cbrtf(distance / (4.0f * j_max)); // re-calculate jerk-limited profile
        ta = tj;
        tv = 0;
    }
    else {
        float d_cruise = distance - d_a;
        tv = d_cruise / v_max;
    }

    p->t[0] = tj;
    p->t[1] = ta - tj;
    p->t[2] = tj;
    p->t[3] = tv;
    p->t[4] = tj;
    p->t[5] = ta - tj;
    p->t[6] = tj;

    p->total_time = 0.0f;
    for (int i = 0; i < 7; ++i)
        p->total_time += p->t[i];
}

void scurve_step(ScurveGenerator* gen)
{
    float t = gen->current_time;
    float j = 0.0f;
    float sum_t = 0.0f;
    const float* phases = gen->profile.t;

    if (t < (sum_t += phases[0])) j = gen->profile.j_max;
    else if (t < (sum_t += phases[1])) j = 0.0f;
    else if (t < (sum_t += phases[2])) j = -gen->profile.j_max;
    else if (t < (sum_t += phases[3])) j = 0.0f;
    else if (t < (sum_t += phases[4])) j = -gen->profile.j_max;
    else if (t < (sum_t += phases[5])) j = 0.0f;
    else if (t < (sum_t += phases[6])) j = gen->profile.j_max;
    else j = 0.0f;

    // Integrate motion state
    float dt = gen->dt;
    gen->state.acceleration += j * dt;
    gen->state.velocity += gen->state.acceleration * dt;
    gen->state.position += gen->state.velocity * dt;

    gen->current_time += dt;
}
