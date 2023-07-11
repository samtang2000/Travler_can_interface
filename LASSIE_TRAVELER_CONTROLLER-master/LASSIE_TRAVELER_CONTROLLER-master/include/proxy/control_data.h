/*
 * @Author: Ryoma Liu, John Bush -- ROBOLAND
 * @Date: 2022-02-02 16:17:20
 * @Last Modified by: John Bush
 * @Last Modified time: 2022-10-05
 */

#ifndef DATA_H_
#define DATA_H_

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <vector>
#include <cmath>
#include "utils/traveler_utils.h"

#define M0_OFFSET 1.5519 // radians
#define M1_OFFSET 1.5896 // radians

// old motor offset: 1.3515 radians
const float L1 = 0.1f; // meters
const float L2 = 0.2f; // meters
const float leg_extension_angle = 25.964f; //the angle of leg extension
const float L3 = 0.12791f; //the length of leg extension

struct MotorStatus
{
    float error;
    float effort;
    float temperature;
    float position;
    float velocity;
};

struct LegStatus
{
    MotorStatus axis0;
    MotorStatus axis1;
    XY_pair toe_position;
    XY_pair toe_velocity;
    XY_pair toe_force;

    float diffAng;
    float meanAng;
    float l1proj;
    float length;
    float dummy;
};

struct ChassisStatus
{
    LegStatus Leg_lf; // default
    LegStatus Leg_lb;
    LegStatus Leg_rf;
    LegStatus Leg_rb;
};

struct MotorCommand
{
    uint8_t motor_control_type;
    float motor_control_effort;
    float motor_control_velocity;
    float motor_control_position;
    float motor_control_speed;
};

struct LegCommand
{
    MotorCommand axis0;
    MotorCommand axis1;
    float theta_command;
    float length_command;
    int state_flag;
};

struct LowControlCommand
{
    LegCommand Leg_lf;
    LegCommand Leg_lb;
    LegCommand Leg_rf;
    LegCommand Leg_rb;
};

struct HumanInterface
{
    float drag_traj = 0;

    // Trajectory Start Flag (run state = true or false)
    int start_flag = 0;
    bool status_update_flag = false;
};

// struct that defines the behavior of trajectories
struct TrajectoryData
{
    // Extrustion Trajectory Parameters
    float extrude_speed;                    // given as cm/s
    float back_speed;
    float extrude_angle;                    // given as deg
    float extrude_depth;                   // defined as the leg extension at the end of the extrusion,
                                            // not the total length of extrusion.

    // Penetration and Shear Parameters
    float shear_penetration_depth;          // given as cm
    float shear_penetration_speed;          // given as cm/s
    float shear_penetration_delay;          // given as sec
    float shear_length;                     // given as cm
    float shear_speed;                      // given as cm/s
    float shear_delay;                      // given as seconds
    float shear_return_speed;               // given as cm/s

    // Workspace Traversal Parameters
    float workspace_angular_speed;          // !given as cm/s, should be rad/s or deg/s
    float workspace_moving_angle;           // given as deg
    float workspace_time_delay;             // given as seconds

    // variables for static movement
    float static_length; // cm -> m
    // given angle: 0deg is downward. + is forward, - is toward frame
    float static_angle; // deg -> rad, * -1 + pi

    //varaible for searching ground height
    float search_start;
    float search_end;
    float ground_height;
    // *Note: not updated by GUI
    float current_t; // time counter
};

struct Traveler
{
    ChassisStatus traveler_chassis;
    LowControlCommand traveler_control;
    HumanInterface traveler_gui;
    TrajectoryData traj_data;
};


#endif