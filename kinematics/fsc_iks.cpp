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

// returns whether point is inside a "simple" enclosed polygon (concave or convex, no holes or segments crossing)
bool Fsciks::pointInPolygon(point_ts point, polygon_ts polygon, int num_points)
{

    bool oddNodes = false, current = polygon.pnt[num_points - 1].y > point.y, previous;
    for (int i = 0; i < num_points; i++)
    {
        previous = current; current = polygon.pnt[i].y > point.y; if (current != previous) oddNodes ^= point.y * multiple[i] + constant[i] < point.x;
    }
    return oddNodes;
}

// Squared distance between two points
double Fsciks::dist2(point_ts a, point_ts b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Minimum distance from point to segment [a, b]
double Fsciks::distPointToSegment(point_ts point, point_ts a, point_ts b)
{
    double l2 = dist2(a, b);
    if (l2 == 0.0) return sqrt(dist2(point, a)); // a == b

    // Project point onto segment, then clamp t to [0,1]
    double t = ((point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y)) / l2;
    if (t < 0.0)
        return sqrt(dist2(point, a));
    else if (t > 1.0)
        return sqrt(dist2(point, b));

    point_ts projection = {
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y)
    };
    return sqrt(dist2(point, projection));
}

// Distance from point to arbitrary "simple" polygon (concave or convex, no holes or segments crossing)
// Yes, this can be quite cpu-intensive if the polygon has many points.... no I don't feel like making it more
// efficient....so just be careful. 
double Fsciks::distPointToPolygon(point_ts point, polygon_ts polygon, int num_points)
{
    double min_dist = DBL_MAX;
    for (int i = 0; i < num_points; ++i) {
        point_ts a = polygon.pnt[i];
        point_ts b = polygon.pnt[(i + 1) % num_points];
        double d = distPointToSegment(point, a, b);
        if (d < min_dist) min_dist = d;
    }
    if (pointInPolygon(point, polygon, num_points))
    {
        return -min_dist; // return a negative number if inside the polygon
    }
    return min_dist;
}
