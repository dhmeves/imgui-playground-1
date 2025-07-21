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
    if (p2Converges == CONVERGES)
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
        if (polygon.pnt[j].y == polygon.pnt[i].y)
        {
            constant[i] = polygon.pnt[i].x;
            multiple[i] = 0;
        }
        else
        {
            constant[i] = polygon.pnt[i].x - (polygon.pnt[i].y * polygon.pnt[j].x) / (polygon.pnt[j].y - polygon.pnt[i].y) + (polygon.pnt[i].y * polygon.pnt[i].x) / (polygon.pnt[j].y - polygon.pnt[i].y);
            multiple[i] = (polygon.pnt[j].x - polygon.pnt[i].x) / (polygon.pnt[j].y - polygon.pnt[i].y);
        }
        j = i;
    }
}

bool Fsciks::pointInPolygon(polygon_ts polygon, int num_points, point_ts p)
{

    bool oddNodes = false;
    bool current = polygon.pnt[num_points - 1].y > p.y;
    bool previous;

    for (int i = 0; i < num_points; i++)
    {
        previous = current;
        current = polygon.pnt[i].y > p.y;
        if (current != previous)
        {
            oddNodes ^= p.y * multiple[i] + constant[i] < p.x;
        }
    }
    return oddNodes;
}

// Utility: Squared distance between two points
float Fsciks::dist2(point_ts a, point_ts b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Utility: Minimum distance from point p to segment [v, w]
float Fsciks::point_to_segment_distance(point_ts p, point_ts v, point_ts w)
{
    float l2 = dist2(v, w);
    if (l2 == 0.0)
    {
        return sqrt(dist2(p, v)); // v == w
    }

    // Project point onto segment, then clamp t to [0,1]
    float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
    if (t < 0.0)
    {
        return sqrt(dist2(p, v));
    }
    else if (t > 1.0)
    {
        return sqrt(dist2(p, w));
    }

    point_ts projection = {
        v.x + t * (w.x - v.x),
        v.y + t * (w.y - v.y)
    };

    return sqrt(dist2(p, projection));
}

// Main function: Distance from point to polygon
float Fsciks::distance_to_polygon(polygon_ts polygon, int num_points, point_ts p)
{
    float min_dist = FLT_MAX;
    for (int i = 0; i < num_points; ++i)
    {
        point_ts v = polygon.pnt[i];
        point_ts w = polygon.pnt[(i + 1) % num_points];
        float d = point_to_segment_distance(p, v, w);
        if (d < min_dist) min_dist = d;
    }
    if (pointInPolygon(polygon, num_points, p)) // if we're inside the polygon, return a negative number
    {
        return -min_dist;
    }
    else
    {
        return min_dist;
    }
}


// Check if arm configuration is valid
bool Fsciks::check_arm_angle(Arm arm, unsigned int joint)
{
    if (joint < NUM_LINKS)
    {
        float minVal = (arm.joints[joint].angularConstraint.min_angle < arm.joints[joint].angularConstraint.max_angle) ? arm.joints[joint].angularConstraint.min_angle : arm.joints[joint].angularConstraint.max_angle; //	Find min/max values incase someone gave us parameters in the wrong order
        float maxVal = (arm.joints[joint].angularConstraint.min_angle > arm.joints[joint].angularConstraint.max_angle) ? arm.joints[joint].angularConstraint.min_angle : arm.joints[joint].angularConstraint.max_angle;
        float angle = getAngle(arm, joint) * RAD_TO_DEG;

        if (angle < minVal || angle > maxVal)
        {
            return false;
        }
        return true;
    }
    return false;
}

// Check if arm configuration is valid
bool Fsciks::check_arm_angles(Arm arm)
{
    for (int i = 0; i < NUM_LINKS; i++)
    {
        if (!check_arm_angle(arm, i))
        {
            return false;
        }
    }
    return true;
}

// Modified motion control with IK validation
IK_CONVERGENCE_E Fsciks::validate_and_constrain_target(Arm arm, mc2D_vec2_t* target, float angle_buffer)
{
    // Try the target position
    Arm test_arm = arm;
    test_arm.joints[3].x = target->x;
    test_arm.joints[3].y = target->y;
    //test_arm.gripperAngle = gripper_angle;

    IK_CONVERGENCE_E result = calcArm(&test_arm);

    if (result != CONVERGES)
    {
        return result;
    }

    // Check joint angles with buffer
    bool angles_valid = true;
    bool near_limit = false;

    for (int i = 0; i < NUM_LINKS; i++) {
        float angle = getAngle(test_arm, i) * RAD_TO_DEG;

        // Normalize angle to [-π, π]
        //while (angle > M_PI) angle -= 2 * M_PI;
        //while (angle < -M_PI) angle += 2 * M_PI;

        float min_limit = (arm.joints[i].angularConstraint.min_angle < arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;
        float max_limit = (arm.joints[i].angularConstraint.min_angle > arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;

        // Check if we're outside limits
        if (angle < min_limit || angle > max_limit) {
            angles_valid = false;
            break;
        }

        // Check if we're within buffer zone of limits
        if (angle < min_limit + angle_buffer ||
            angle > max_limit - angle_buffer) {
            near_limit = true;
        }
    }

    if (angles_valid && !near_limit) {
        // Safe zone - allow free movement
        return CONVERGES;
    }

    if (!angles_valid) {
        // Already outside limits - pull back to last valid position
        mc2D_vec2_t current = { arm.joints[3].x, arm.joints[3].y };
        *target = current;  // Stay at current position
        return CONVERGES;
    }

    // We're in the buffer zone - allow movement away from limit only
    // Check if movement would take us further into limit
    mc2D_vec2_t current = { arm.joints[3].x, arm.joints[3].y };
    mc2D_vec2_t movement = {
        target->x - current.x,
        target->y - current.y
    };

    // Test a small step in the movement direction
    test_arm = arm;
    test_arm.joints[3].x = current.x + movement.x * 0.1;
    test_arm.joints[3].y = current.y + movement.y * 0.1;

    if (calcArm(&test_arm) == CONVERGES) {
        // Check if this movement makes angles worse
        bool getting_worse = false;
        for (int i = 0; i < NUM_LINKS; i++) {
            float current_angle = getAngle(arm, i) * RAD_TO_DEG;
            float new_angle = getAngle(test_arm, i) * RAD_TO_DEG;

            // Normalize
            //while (current_angle > M_PI) current_angle -= 2 * M_PI;
            //while (current_angle < -M_PI) current_angle += 2 * M_PI;
            //while (new_angle > M_PI) new_angle -= 2 * M_PI;
            //while (new_angle < -M_PI) new_angle += 2 * M_PI;

            //float min_limit = arm_limits->limits[i].min_angle;
            //float max_limit = arm_limits->limits[i].max_angle;

            float min_limit = (arm.joints[i].angularConstraint.min_angle < arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;
            float max_limit = (arm.joints[i].angularConstraint.min_angle > arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;

            // Check if we're moving toward a limit
            if (current_angle < min_limit + angle_buffer && new_angle < current_angle) {
                getting_worse = true;
                break;
            }
            if (current_angle > max_limit - angle_buffer && new_angle > current_angle) {
                getting_worse = true;
                break;
            }
        }

        if (getting_worse) {
            // Don't allow movement toward limit
            *target = current;
        }
    }

    return CONVERGES;
}


// Modified motion control with IK validation
//IK_CONVERGENCE_E Fsciks::validate_and_constrain_target(Arm arm, mc2D_vec2_t* target)
//{
//    // Try the target position
//    Arm test_arm = arm;
//    test_arm.joints[3].x = target->x;
//    test_arm.joints[3].y = target->y;
//    //test_arm.gripperAngle = gripper_angle;
//
//    IK_CONVERGENCE_E result = calcArm(&test_arm);
//
//    if (result != CONVERGES)
//    {
//        return result;
//    }
//
//    // Check joint angles
//    if (!check_arm_angles(test_arm))
//    {
//        // Target would violate joint limits
//        // Binary search for valid position along line from current to target
//        mc2D_vec2_t current = { arm.joints[3].x, arm.joints[3].y };
//        mc2D_vec2_t valid_target = current;
//
//        float t_min = 0.0;
//        float t_max = 1.0;
//
//        // Binary search for maximum valid distance
//        for (int iter = 0; iter < 10; iter++) {
//            float t = (t_min + t_max) / 2.0;
//
//            test_arm = arm;
//            test_arm.joints[3].x = current.x + t * (target->x - current.x);
//            test_arm.joints[3].y = current.y + t * (target->y - current.y);
//
//            if (calcArm(&test_arm) == CONVERGES && check_arm_angles(test_arm))
//            {
//                t_min = t;
//                valid_target.x = test_arm.joints[3].x;
//                valid_target.y = test_arm.joints[3].y;
//            }
//            else {
//                t_max = t;
//            }
//        }
//
//        // Update target to valid position
//        *target = valid_target;
//        return CONVERGES;
//    }
//
//    return CONVERGES;
//}
