#include "controller/inverse_kinematics.h"

using namespace std;

#define DEBUG
#define CONTROL_FREQ 100    // Hz


/**
 * ! Leg Workspace:
 * Gamma must be within [0.087, 2.61] radians
 * Theta must be within [-2.47, +2.47] radians
 */


/**
 * @brief Converts Abstract leg position into Theta, Gamma values
 *
 * @param L : Leg length
 * @param Theta : Abstract leg angle
 * @return : Pair (Theta, Gamma) representing the motor separation
 *             for the given input parameters. Also called the Diff Angle
 */
void GetGamma(float L, float &gamma)
{
    gamma = acosf((pow(L1, 2) + pow(L, 2) - pow(L2, 2)) / (2 * L1 * L));
}

/**
 * @brief Finds the Leg length, angle, and motor separation for a given X, Y toe point
 *
 * @param X : Toe X position
 * @param Y : Toe Y position
 *
 * @return : Returns <Length, Angle, Gamma>
 */
void PhysicalToAbstract(float X, float Y, float &L, float &theta, float &gamma)
{
    L = sqrt(pow(X, 2) + pow(Y, 2));
    theta = atan2(X, Y);

    float theta_temp = atan2(X, Y);
    // compensates for -x, -y wrapping
    // if (X < 0.0f && Y < 0.0f) {
    //     theta = 2 * M_PI + theta_temp;
    // } else {
    //     theta = theta_temp;
    // }
    if (theta_temp < 0.0f) {
        theta = 2 * M_PI + theta_temp;
    } else {
        theta = theta_temp;
    }
    gamma = (float)acosf((pow(L1, 2) + pow(L, 2) - pow(L2, 2)) / (2 * L1 * L));
}

/**
 * @brief Finds the Leg angle and motor separation for a given X, Y toe point
 *
 * @param X : Toe X position
 * @param Y : Toe Y position
 *
 * @return : Returns <Theta, Gamma>
 */
void PhysicalToAbstract(float X, float Y, float &theta, float &gamma)
{
    float L = sqrt(pow(X, 2) + pow(Y, 2));

    float theta_temp = atan2(X, Y);
    // compensates for -x, -y wrapping
    // if (X < 0.0f && Y < 0.0f) {
    //     theta = 2 * M_PI + theta_temp;
    // } else {
    //     theta = theta_temp;
    // }
    if (theta_temp < 0.0f) {
        theta = 2 * M_PI + theta_temp;
    } else {
        theta = theta_temp;
    }
    gamma = static_cast<float>(acosf((pow(L1, 2) + pow(L, 2) - pow(L2, 2)) / (2 * L1 * L)));
    // printf("Theta: %f, Gamma: %f\n", theta, gamma);
}

/**
 * @brief Finds the Leg angle and motor separation for a given X, Y toe point with consideration of L extension
 *
 * @param X : Toe X position
 * @param Y : Toe Y position
 *
 * @return : Returns <Theta, Gamma>
 */
void PhysicalToAbstractWithExtension(float X, float Y, float &theta, float &gamma)
{

    float L4 = sqrt(pow(L2,2) + pow(L3, 3) + 2*L2*L3*cosf(leg_extension_angle));
    float sigma_2 = acosf((pow(L2,2)-pow(L3,2)-pow(L4,2))/(2*L3*L4));
    float L6 = sqrt(pow(X,2) + pow(Y, 3));
    float sigma_3 = acosf((pow(L1,2)-pow(L4,2)-pow(L6,2))/(2*L6*L4));
    float L = sqrt(pow(L3,2) + pow(L6, 3) + 2*L3*L6*cosf(sigma_3-sigma_2));
    float sigma_4 = acosf((pow(L3,2)-pow(L,2)-pow(L6,2))/(2*L6*L));

    // float L = sqrt(pow(X, 2) + pow(Y, 2));

    float theta_temp = atan2(X, Y) - sigma_4;
    // compensates for -x, -y wrapping
    // if (X < 0.0f && Y < 0.0f) {
    //     theta = 2 * M_PI + theta_temp;
    // } else {
    //     theta = theta_temp;
    // }
    if (theta_temp < 0.0f) {
        theta = 2 * M_PI + theta_temp;
    } else {
        theta = theta_temp;
    }
    gamma = static_cast<float>(acosf((pow(L1, 2) + pow(L, 2) - pow(L2, 2)) / (2 * L1 * L)));
    // printf("Theta: %f, Gamma: %f\n", theta, gamma);
}

/**
 * @brief Finds the physical (X, Y) position of the toe for a given L and Theta
 *
 * @param L : Leg length
 * @param Theta : Abstract leg angle
 * @return : Pair (X, Y) representing the position of the toe in relation to
 *              the Origin (the hip joint)
 */
void AbstractToPhysical(float L, float Theta, float &x, float &y)
{
    // check consistency for this
    x = - L * sinf(Theta);
    y = L * cosf(Theta);
}

void AbstractToPhysical(float L, float Theta, XY_pair &point) {
    point.x = - L * sinf(Theta);
    point.y = L * cosf(Theta);
}


void LinearTraj(float t, float t_start, float vel, XY_pair A, XY_pair B, float &X, float &Y) {
    float t_rel = t - t_start;
    float d1 = B.x - A.x;
    float d2 = B.y - A.y;

    float raw_vel = sqrtf(d1*d1 + d2*d2);
    float scalar = vel/raw_vel;
    
    X = scalar * d1 * t_rel + A.x;
    Y = scalar * d2 * t_rel + B.x;
}

bool LinearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float &X, float &Y) {
    float d1 = B.x - A.x;
    float d2 = B.y - A.y;

    float raw_vel = sqrtf(d1*d1 + d2*d2);
    float scalar = vel/raw_vel;

    X = (scalar * d1 * t_rel) + A.x;
    Y = (scalar * d2 * t_rel) + A.y;

    // Checking for end of trajectory
    // distance from point A to point B
    float ideal_dist = sqrtf((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
    // actual distance is point along the path
    float actual_dist = sqrtf((X - A.x) * (X - A.x)+ (Y - A.y) * (Y - A.y));

    if (actual_dist <= ideal_dist) {
        return false;
    } else {
        return true;    // returns true when complete
    }
}


bool validPath(XY_pair A, XY_pair B) {
    float a = A.y - B.y;
    float b = B.x - A.x;
    float c = (A.x - B.x) *  A.y + A.x * (B.y - A.y);
    float dist = abs(c) / sqrt(a * a + b * b);
    bool valid = (dist >= 0.105);
    // CASE: infinite line does not intersect circle
    if (valid) {
        printf("Path does not intersect center of leg\n");
        return true;
    // CASE: infinite line intersects circle, but segment may not
    } else {
        Point_Pair circle_points;
        circle_points = findSwingPoints(A, B);
        float seg_dist = distance(A, B);
        float d1 = distance(circle_points.A, A);  // distance from circle intersection 1 and point A
        float d2 = distance(circle_points.B, A);  // distance from circle intersection 2 and point A
        float d3 = distance(circle_points.A, B);  // distance btwn circle intersection 1 and point B
        float d4 = distance(circle_points.B, B);  // distance btwn circle intersection 2 and point B
        /**
         * @logic_description:
         *  d1 + d3 is the line segment that goes from point A -> circle point 1 -> point B
         *  if this distance is equal to the segment distance, then intersection point 1 is between points A and B, thus
         *  the segment intersects the circle
         */
        if (abs(seg_dist - (d1 + d3)) < 0.0001 || abs(seg_dist - (d2 + d4)) < 0.0001 ) {
            valid = false;  // path intersects center
            printf("Path intersects center of leg\n");
        } else {
            valid = true;
            printf("Path does not intersect center of leg\n");
        }
    }
    return (valid);
}

Point_Pair findSwingPoints(XY_pair A, XY_pair B) {
    float m = (B.y - A.y) / (B.x - A.x);
    float b = ((B.x - A.x) * A.y - A.x * (B.y - A.y)) / (B.x - A.x);

    float A_ = (m * m + 1);
    float B_ = 2 * m * b;
    float C_ = (b * b - 0.105 * 0.105);

    XY_pair xvals;
    xvals = findRoots(A_, B_, C_);
    // Point_Pair out;
    Point_Pair out;
    out = findCircleIntercepts(xvals, m, b);
    XY_pair swingA = out.A;
    XY_pair swingB = out.B;

    float distanceA = distance(A, swingA);
    float distanceB = distance(A, swingB);
    // swap points case
    if (distanceB < distanceA) {
        out.A = swingB;
        out.B = swingA;
    }
    return out;
}

/**
 * ! Leg Workspace Must be validated empirically
 * Gamma must be within [0.087, 2.61] radians
 * Theta must be within [-2.47, +2.47] radians
 */

/**
 * @brief Checks whether a current abstract leg position is valid
 */
bool inBounds(float Gamma, float Theta, float L)
{
    if (Gamma <= 0.087 || Gamma > 2.61 ||
        Theta < -2.47 || Theta > 2.47 ||
        L <= 0.03 || L >= 0.3)
    {   // ! CALCULATE PRECISE L RANGE FROM GAMMA
        return false;
    } else {
        return true;
    }
}

/**
 * @brief Checks whether a toe position is in bounds
 */
bool inBounds(float x, float y)
{
    float L = sqrtf(pow(x, 2) + pow(y, 2));

    if (L <= 0.105|| L >= 0.29)
    {
        return false;
    } else {
        return true;
    }
}


// Finds roots of quadratic equation ax*2 + bx + x
XY_pair findRoots(float a, float b, float c)
{
    XY_pair out;

    // If a is 0, then equation is not quadratic, but
    // linear
    if (a == 0) {
        out.x = 0;
        out.y = 0;
        return out;
    }

    float d = b * b - 4 * a * c;
    double sqrt_val = sqrt(abs(d));

    if (d > 0) {    // two different real roots
        out.x = static_cast<double>(-b + sqrt_val) / (2 * a);
        out.y = static_cast<double>(-b - sqrt_val) / (2 * a);
    } else if (d == 0) {    // one real root
        out.x = -static_cast<double>(b / (2 * a));
        out.y = -static_cast<double>(b / (2 * a));
    } else {    // d < 0 -> complex roots
        out.x = 0;
        out.y = 0;
    }
    return out;
}

// Finds circle intercepts given x values for traveler
Point_Pair findCircleIntercepts(XY_pair xvals, float m, float b) {
    Point_Pair out;
    out.A.x = xvals.x;
    out.A.y = xvals.x * m + b;
    out.B.x = xvals.y;
    out.B.y = xvals.y * m + b;

    return out;
}

float distance(XY_pair A, XY_pair B) {
    return sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
}
