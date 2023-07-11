#ifndef TRAJECTRIES_PARSER_
#define TRAJECTRIES_PARSER_

#include <iostream>
#include "proxy/control_data.h"
#include "controller/inverse_kinematics.h"

namespace traveler_namespace{
namespace control{

    class TrajectoriesParser
    {
    public:
        static TrajectoriesParser &GetTrajParser()
        {
            static TrajectoriesParser singleton;
            return singleton;
        }
        void Init();
        void generate_temp_traj(Traveler &);
        void generate_temp_speed(Traveler &);

        /**
         * @brief Shorter function call that uses private theta_ and gamma_ values
         */
        void SetCoupledPosition(Traveler &traveler_);

        /**
         * @brief Sets the motor positions based on a given theta and gamma value
         *
         * @param theta Angle of the toe from the upward vertical axis
         * @param gamma Separation angle from the midline of the leg to the linkages
         */
        void SetCoupledPosition(Traveler &, float theta, float gamma);

        /**
         * @brief Commands the motors to send the toe to a given (target x, target y)
         *
         * @param target_x target X coordinate of toe (cartesian)
         * @param target_y target Y coordinate of toe (cartesian)
         */
        void CartesianMotorCommand(Traveler &, float target_x, float target_y);

        /**
         * @brief Commands the leg to a given Theta, Length
         *
         * @param target_length Target length of leg extension (meters)
         * @param target_theta  Target angle of leg measured from positive vertical axis
         */
        void AbstractMotorCommand(Traveler &, float target_length, float target_theta);

        // TRAJECTORIES
        void TraverseWorkspace(Traveler &, WorkspaceTraversalParams &params);

        /**
         * @brief Commands a leg penetration at desired angle, depth, and velocity
         *
         * @returns True-> complete, false-> not complete
         */
        bool Penetrate(Traveler &);

        /**
         * @brief Commands a leg to find the ground height
         *
         * @returns True-> complete, false-> not complete
         */
        bool DetectGround(Traveler &);

        /**
         * @brief Commands a penetration and shear movement
         *
         * @returns True-> complete, false-> not complete
         */
        bool PenetrateAndShearRoutine(Traveler &);

        /**
         * @brief Moves leg to Goal point at 5cm/s.
         *
         * @param Goal: X,Y Pair Struct representing desired leg destination
         * @param first_iteration: State variable
         * @return 1: Returns 1 if movement completes successfully
         * @return 0: If movement is in progress
         * @return -1: if Goal point is invalid
         */
        int GoToPoint(Traveler &, XY_pair Goal);

    private:
        float t_period = 0;
        float target_x = 0;
        float target_y = -0.16f;
        float theta_ = M_PI;
        float gamma_;
        float length_ = 0.173;
        float axis_0 = 0;
        float axis_1 = 0;
        bool outside_traj = false;

        /**
         * General State variables shared by all trajectories
         */
        bool first_iteration = false;
        int state_flag = 0;
        std::chrono::steady_clock::time_point start; // chrono literal
        std::chrono::steady_clock::time_point GTP_start;
        std::chrono::steady_clock::time_point DG_start;
        /**
         * Variables for Penetration trajectory
         */
        XY_pair penetration_start;
        XY_pair penetration_end;
        float delay = 2.0f;
        std::chrono::steady_clock::time_point delay_start;
        int Pene_state = 0;
        int DG_state = 0;

        /**
         * State Params for GoToPoint()
         */
        XY_pair start_point;
        bool swing;
        Point_Pair swing_points;
        int dir;
        float theta1;
        float theta2;
        int GTP_state = 0;
        bool GTP_first_iteration = true;
        bool DG_first_iteration = true;

        /**
         * State Params for Workspace Traversal
         */
        WorkspaceTraversalParams traverseParams;
        // float threshold_ = 0.03125 * (traverseParams.max_theta - traverseParams.min_theta);
        float threshold_ = 0.0;
        bool run_ = true;
        int WT_state = 0;

        /**
         * defined points for penetrate + shear
         */
        XY_pair A = {0.0f, -0.12f};
        
        struct PS_Params {
            int state = 0;
            bool first_iteration = true;
            XY_pair A{0.0f, -0.12f};
            XY_pair B;
            XY_pair C;
            float vel1;
            float vel2;
            float vel3;
            float delay1;
            float delay2;
            std::chrono::steady_clock::time_point start;
            
            float delay_elapsed;
            XY_pair curr;
            XY_pair dest;
            float curr_vel;
            float curr_delay;
        };

        PS_Params ps;

        struct DG_Params {
            int state = 0;
            bool first_iteration = true;
            XY_pair search_start;
            XY_pair search_end;

            float vel1;
            float vel2;
            float vel3;


            std::chrono::steady_clock::time_point delay_start;
            std::chrono::steady_clock::time_point start;
            
            float delay_elapsed;

        };

        DG_Params dg;

        /**
         * @brief parameter for static movement
         */
        XY_pair destination;


        void printTrajData(Traveler &);
    };
}
}
#endif