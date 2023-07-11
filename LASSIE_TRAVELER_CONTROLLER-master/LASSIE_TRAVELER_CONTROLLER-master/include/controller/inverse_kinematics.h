#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#define _USE_MATH_DEFINES

#include <chrono>

#include <tuple>
#include <utility>
#include <stdio.h>
#include <cmath>
#include <iostream>

#include "../proxy/control_data.h"

void GetGamma(float L, float& gamma);

void PhysicalToAbstract(float X, float Y, float& L, float& theta, float& gamma);
void PhysicalToAbstract(float X, float Y, float &theta, float &gamma);
void PhysicalToAbstractWithExtension(float X, float Y, float &theta, float &gamma);

void AbstractToPhysical(float L, float Theta, float& x, float& y);
void AbstractToPhysical(float L, float Theta, XY_pair &point);



/**
 * @brief Defines positions along line defined by
 *      points A, B with given velocity
 * @param t -> time for curr position
 * @param t_start -> time when trajectory started at point A
 * @param vel -> desired velocity magnitude
 * @param A @param B: two (x,y) points to define motion
 * @param X, @param Y: motor position
 */
void LinearTraj(float t, float t_start, float vel, XY_pair A, XY_pair B, float &X, float &Y);

/**
 * @brief Defines positions along line defined by
 *      points A, B with given velocity
 * @param t_rel -> time of traj (calculated outside of function)
 * @param vel -> desired velocity magnitude
 * @param A @param B: two (x,y) points to define motion
 * @param X, @param Y: motor position
 */
bool LinearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float &X, float &Y);

bool validPath(XY_pair A, XY_pair B);

Point_Pair findSwingPoints(XY_pair A, XY_pair B);

/**
 * @brief Checks if Leg Position defined by Theta, gamma, L is valid.
 * @return true -> leg position is in workspace
 * @return false -> leg position is not in workspace
 */
bool inBounds(float Gamma, float Theta, float L);

/**
 * @brief Checks if Leg Position defined by Theta, gamma, L is valid.
 * @return true -> leg position is in workspace.
 * 
 * @return false -> leg position is not in workspace
 */
bool inBounds(float x, float y);

// Prints roots of quadratic equation ax*2 + bx + x
XY_pair findRoots(float a, float b, float c);

Point_Pair findCircleIntercepts(XY_pair xvals, float m, float b);

float distance(XY_pair A, XY_pair B);

typedef struct WorkspaceTraversalParams{
    // behavior parameters
    float max_ext = 0.22f; // meters
    float min_ext = 0.11f; // meters
    double min_theta = 0.6f; // radians from vertical
    double max_theta = 4.1f; //radians from vertical
    double d_theta = 0.008f;
    float d_L = 0.0002f;
    float L_step = 0.01f;

    // state variables
    float curr_ext = max_ext;
    float curr_theta = min_theta;
    bool cw = true;
    bool run = false;
    int counter = 0;
    bool shorten_leg = false;
    float next_ext;
    const int cycle_len = 1; // = seconds * 100Hz
    const int measure_time = 0; // = seconds * 100Hz -> time of cycle spent static
} WorkspaceTraversalParams;





#endif