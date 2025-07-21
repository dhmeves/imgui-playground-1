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


IK_CONVERGENCE_E Fsciks::validate_and_constrain_target_with_decel(
    Arm arm,
    mc2D_vec2_t* target,
    mc2D_vec2_t current_pos,
    mc2D_vec2_t current_vel,
    mc2D_constraints_t* motion_limits,
    joint_limit_state_t* limit_state)
{

    // First, check current arm state
    Arm current_arm = arm;
    joint_limit_state_t new_limit_state = { 0 };

    // Check each joint's current angle and proximity to limits
    for (int i = 0; i < NUM_LINKS; i++)
    {
        float angle = getAngle(current_arm, i) * RAD_TO_DEG;
        //while (angle > M_PI) angle -= 2 * M_PI;
        //while (angle < -M_PI) angle += 2 * M_PI;
        float min_limit = (arm.joints[i].angularConstraint.min_angle < arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle; //	Find min/max values incase someone gave us parameters in the wrong order
        float max_limit = (arm.joints[i].angularConstraint.min_angle > arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;
        //float min_limit = arm_limits->limits[i].min_angle;
        //float max_limit = arm_limits->limits[i].max_angle;

        // Calculate stopping angle based on current angular velocity
        // Estimate angular velocity from cartesian velocity (simplified)
        float angular_vel_estimate = vec2_magnitude(current_vel) / arm.joints[i].length;
        float stopping_angle = RAD_TO_DEG * (angular_vel_estimate * angular_vel_estimate) / (2.0 * motion_limits->max_accel / arm.joints[i].length);

        // Add safety margin
        float safety_buffer = 2.0f; // ~2 degrees
        float decel_buffer = stopping_angle + safety_buffer;

        // Check proximity to limits
        if (angle <= min_limit + decel_buffer) {
            new_limit_state.at_limit[i] = true;
            new_limit_state.limit_direction[i] = -1;
        }
        else if (angle >= max_limit - decel_buffer) {
            new_limit_state.at_limit[i] = true;
            new_limit_state.limit_direction[i] = 1;
        }
    }

    // Try the target position
    Arm test_arm = arm;
    test_arm.joints[3].x = target->x;
    test_arm.joints[3].y = target->y;
    //test_arm.gripperAngle = gripper_angle;

    IK_CONVERGENCE_E result = calcArm(&test_arm);
    if (result != CONVERGES) {
        return result;
    }

    // Check if target would violate constraints or make them worse
    bool target_valid = true;
    bool improving = false;

    for (int i = 0; i < NUM_LINKS; i++) {
        float current_angle = getAngle(current_arm, i) * RAD_TO_DEG;
        float new_angle = getAngle(test_arm, i) * RAD_TO_DEG;

        // Normalize
        //while (current_angle > M_PI) current_angle -= 2 * M_PI;
        //while (current_angle < -M_PI) current_angle += 2 * M_PI;
        //while (new_angle > M_PI) new_angle -= 2 * M_PI;
        //while (new_angle < -M_PI) new_angle += 2 * M_PI;

        float min_limit = (arm.joints[i].angularConstraint.min_angle < arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle; //	Find min/max values incase someone gave us parameters in the wrong order
        float max_limit = (arm.joints[i].angularConstraint.min_angle > arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;
        //float min_limit = arm_limits->limits[i].min_angle;
        //float max_limit = arm_limits->limits[i].max_angle;

        // Check if we're outside limits
        if (current_angle < min_limit || current_angle > max_limit) {
            // We're in violation - check if target improves situation
            if ((current_angle < min_limit && new_angle > current_angle) ||
                (current_angle > max_limit && new_angle < current_angle)) {
                improving = true;
            }
            else {
                target_valid = false;
                break;
            }
        }

        // If at limit, check movement direction
        if (new_limit_state.at_limit[i])
        {
            float angle_change = new_angle - current_angle;

            // Don't allow movement further into limit
            if ((new_limit_state.limit_direction[i] < 0 && angle_change < -0.001) ||
                (new_limit_state.limit_direction[i] > 0 && angle_change > 0.001)) {
                target_valid = false;
                break;
            }
        }
    }

    // Handle different cases
    if (target_valid || improving) {
        *limit_state = new_limit_state;
        return CONVERGES;
    }

    // Target would violate constraints - find safe target
    // Use gradient descent to find nearest valid position
    mc2D_vec2_t safe_target = current_pos;
    mc2D_vec2_t search_dir = {
        target->x - current_pos.x,
        target->y - current_pos.y
    };

    // If we're against a limit, modify search direction to move away
    if (target_valid == false) {
        // Binary search for valid position, but bias toward escaping limits
        float t = 0.5;
        float t_step = 0.25;

        for (int iter = 0; iter < 8; iter++) {
            test_arm = arm;
            test_arm.joints[3].x = current_pos.x + t * search_dir.x;
            test_arm.joints[3].y = current_pos.y + t * search_dir.y;

            if (calcArm(&test_arm) == CONVERGES) {
                bool valid = true;
                bool escaping = false;

                for (int i = 0; i < NUM_LINKS; i++) {
                    float test_angle = getAngle(test_arm, i);
                    float curr_angle = getAngle(current_arm, i);

                    // Normalize
                    //while (test_angle > M_PI) test_angle -= 2 * M_PI;
                    //while (test_angle < -M_PI) test_angle += 2 * M_PI;
                    //while (curr_angle > M_PI) curr_angle -= 2 * M_PI;
                    //while (curr_angle < -M_PI) curr_angle += 2 * M_PI;

                    float min_limit = (arm.joints[i].angularConstraint.min_angle < arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle; //	Find min/max values incase someone gave us parameters in the wrong order
                    float max_limit = (arm.joints[i].angularConstraint.min_angle > arm.joints[i].angularConstraint.max_angle) ? arm.joints[i].angularConstraint.min_angle : arm.joints[i].angularConstraint.max_angle;
                    // Check if escaping violation
                    if ((curr_angle < min_limit && test_angle > curr_angle) ||
                        (curr_angle > max_limit && test_angle < curr_angle))
                    {
                        escaping = true;
                    }

                    // Check if movement is safe
                    if (new_limit_state.at_limit[i])
                    {
                        float delta = test_angle - curr_angle;
                        if ((new_limit_state.limit_direction[i] < 0 && delta < -0.001) ||
                            (new_limit_state.limit_direction[i] > 0 && delta > 0.001)) {
                            valid = false;
                            break;
                        }
                    }
                }

                if (valid || escaping) {
                    safe_target.x = test_arm.joints[3].x;
                    safe_target.y = test_arm.joints[3].y;
                    t += t_step;
                }
                else {
                    t -= t_step;
                }
            }
            else {
                t -= t_step;
            }
            t_step *= 0.5;
            t = fmax(0.0, fmin(1.0, t));
        }
    }

    *target = safe_target;
    *limit_state = new_limit_state;
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
