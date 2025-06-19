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
