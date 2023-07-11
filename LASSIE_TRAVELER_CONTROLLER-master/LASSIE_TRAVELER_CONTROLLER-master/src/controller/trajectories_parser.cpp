#include "controller/trajectories_parser.h"

//#define DEBUG_WORKSPACE

using namespace std;

namespace traveler_namespace
{
    namespace control
    {

        void TrajectoriesParser::Init() {}

        void TrajectoriesParser::SetCoupledPosition(Traveler &traveler)
        {
            float axis_0 = -((2 * M_PI - M0_OFFSET) - (theta_ + gamma_));
            float axis_1 = -(theta_ - gamma_ - M1_OFFSET);
            traveler.traveler_control.Leg_lf.axis0.motor_control_position = axis_0;
            traveler.traveler_control.Leg_lf.axis1.motor_control_position = axis_1;
        }

        void TrajectoriesParser::SetCoupledPosition(Traveler &traveler, float theta, float gamma)
        {
            float axis_0 = -((2 * M_PI - M0_OFFSET) - (theta + gamma));
            float axis_1 = -(theta - gamma - M1_OFFSET);
            traveler.traveler_control.Leg_lf.axis0.motor_control_position = axis_0;
            traveler.traveler_control.Leg_lf.axis1.motor_control_position = axis_1;
        }

        void TrajectoriesParser::CartesianMotorCommand(Traveler &traveler, float target_x_, float target_y_)
        {
            PhysicalToAbstractWithExtension(target_x_, target_y_, theta_, gamma_);
            printf("with extension target x: %f, target y: %f, theta: %f, gamma: %f\n", target_x_, target_y_, theta_, gamma_);
            PhysicalToAbstract(target_x_, target_y_, theta_, gamma_);
            printf("target x: %f, target y: %f, theta: %f, gamma: %f\n", target_x_, target_y_, theta_, gamma_);
            SetCoupledPosition(traveler);
        }

        void TrajectoriesParser::AbstractMotorCommand(Traveler &traveler, float target_length, float target_theta)
        {
            GetGamma(target_length, gamma_);
            SetCoupledPosition(traveler, target_theta, gamma_);
        }

        void TrajectoriesParser::TraverseWorkspace(Traveler &traveler, WorkspaceTraversalParams &params)
        {
            float theta_dist_min = params.curr_theta - params.min_theta;
            float theta_dist_max = params.max_theta - params.curr_theta;
            if ((params.counter >= params.measure_time) && params.run)
            {
                WT_state = 0;
                // check for completion
                if ((theta_dist_min == 0.0f && !params.cw && params.curr_ext <= params.min_ext) || (params.curr_ext < params.min_ext))
                {
                    WT_state = 1;
                    params.run = false; // sets run to false
#ifdef DEBUG_WORKSPACE
                    printf("======= COMPLETED TRAVERSAL =======\n");
#endif
                }
                else
                {
                    if (params.shorten_leg)
                    {
                        WT_state = 1;
                        if (params.curr_ext >= params.next_ext)
                        {
                            params.curr_ext -= params.d_L;
                        }
                        else
                        {
                            params.shorten_leg = false;
                        }
                    }
                    // changing direction & changing extension
                    else if ((theta_dist_min <= 0.0f && !params.cw) || (theta_dist_max <= 0.0f && params.cw))
                    {
                        // change direction
                        params.cw = !params.cw;
                        // shorten leg
                        params.next_ext = params.curr_ext - params.L_step;
                        printf("Next Ext: %f", params.next_ext);
                        // send to shortening state
                        params.shorten_leg = true;
#ifdef DEBUG_WORKSPACE
                        printf("CHANGE DIR  :: Theta %4.4f | L %4.4f \n", params.curr_theta, params.curr_ext);
#endif
                    }
                    else
                    { // constant speed
                        WT_state = 0;
                        float delta = traveler.traj_data.workspace_angular_speed / 100.0f;
                        delta = params.cw ? delta : -delta;
                        params.curr_theta += delta;

#ifdef DEBUG_WORKSPACE
                        printf("CONSTANT    :: Theta %4.4f | L %4.4f \n", params.curr_theta, params.curr_ext);
#endif
                    }
                }
            }
            else
            {
                WT_state = 1;
#ifdef DEBUG_WORKSPACE
                printf("STATIC      :: Theta %4.4f | L %4.4f \n", params.curr_theta, params.curr_ext);
#endif
            }
            params.counter++;
            params.counter %= params.cycle_len;
            // leg command
            AbstractMotorCommand(traveler, traverseParams.curr_ext, traverseParams.curr_theta);
            // update command values
            traveler.traveler_control.Leg_lf.theta_command = traverseParams.curr_theta;
            traveler.traveler_control.Leg_lf.length_command = traverseParams.curr_ext;
            traveler.traveler_control.Leg_lf.state_flag = WT_state;
        }

        bool TrajectoriesParser::Penetrate(Traveler &traveler)
        {
            
            if (first_iteration)
            {
                float theta = traveler.traj_data.extrude_angle;
                float L = traveler.traj_data.extrude_depth + traveler.traj_data.ground_height;
                // define start and end points
                AbstractToPhysical(traveler.traj_data.ground_height - 0.03, theta, penetration_start);
                AbstractToPhysical(L, theta, penetration_end);

                start = chrono::steady_clock::now();
                printf(".............Penetrating................\n");
                
            }

            auto now = chrono::steady_clock::now();
            float t = chrono::duration<float>(now - start).count();
            float delay_elapsed;
            switch (Pene_state)
            {
            // move to start of penetration
            case 0:
                printf(".............Go to start point................\n");
                printf("penetration_startx: %f, penetration_starty: %f\n", penetration_start.x, penetration_start.y);
                printf("penetration_endx: %f, penetration_endy: %f\n", penetration_end.x, penetration_end.y);
                
                if (GoToPoint(traveler, penetration_start) == 1)
                {
                    Pene_state++;
                    dg.delay_start = chrono::steady_clock::now();
                }
                
                break;
            case 1:
                printf(".............pause................\n");
                delay_elapsed = chrono::duration<float>(now - dg.delay_start).count();
                if (delay_elapsed > delay)
                {
                    Pene_state++;
                    start = chrono::steady_clock::now();
                }
                
                break;
                
            // penetrate downward
            case 2:
                printf(".............penetrate downward................\n");
                printf("penetration_startx: %f, penetration_starty: %f\n", penetration_start.x, penetration_start.y);
                printf("penetration_endx: %f, penetration_endy: %f\n", penetration_end.x, penetration_end.y);
                if (LinearTraj(t, traveler.traj_data.extrude_speed, penetration_start, penetration_end, target_x, target_y))
                {
                    Pene_state++;
                    dg.delay_start = chrono::steady_clock::now();
                }
                printf("target_x: %f, target_y: %f\n", target_x, target_y);
                CartesianMotorCommand(traveler, target_x, target_y);
                
                break;
            // delay in extended position
            case 3:
                printf(".............delay in extended position................\n");
                delay_elapsed = chrono::duration<float>(now - dg.delay_start).count();
                if (delay_elapsed > delay)
                {
                    Pene_state++;
                    start = chrono::steady_clock::now();
                }
                
                break;
            // return to start position
            case 4:
                
                if (LinearTraj(t, traveler.traj_data.back_speed, penetration_end, penetration_start, target_x, target_y))
                {
                    Pene_state++;
                }
                CartesianMotorCommand(traveler, target_x, target_y);
                printf(".............return to start position................\n");
                break;
            case 5:
                return true;
            }
            
            return false;
        }

        bool TrajectoriesParser::DetectGround(Traveler &traveler)
        {
            
            if (dg.first_iteration)
            {
                dg.state = 0;
                float L1 = traveler.traj_data.search_start;
                float L2 = traveler.traj_data.search_end;
                // define start and end points
                AbstractToPhysical(L1, M_PI, dg.search_start);
                AbstractToPhysical(L2, M_PI, dg.search_end);
                dg.start = chrono::steady_clock::now();
                dg.first_iteration = false;
                printf(".............Penetrating................\n");
            }

            auto now = chrono::steady_clock::now();
            float t = chrono::duration<float>(now - dg.start).count();
            float delay_elapsed;
            switch (dg.state)
            {
            // move to start of penetration
            case 0:
                printf(".............Go to start point................\n");
                printf("dg.search_startx: %f, dg.search_starty: %f\n", dg.search_start.x, dg.search_start.y);
                printf("penetration_endx: %f, penetration_endy: %f\n", dg.search_end.x, dg.search_end.y);
                
                if (GoToPoint(traveler, dg.search_start) == 1)
                {
                    dg.state++;
                    dg.delay_start = chrono::steady_clock::now();
                }
                
                break;
            case 1:
                printf(".............pause................\n");
                delay_elapsed = chrono::duration<float>(now - dg.delay_start).count();
                if (delay_elapsed > delay)
                {
                    dg.state++;
                    dg.start = chrono::steady_clock::now();
                }
                
                break;
                
            // penetrate downward
            case 2:
                printf(".............penetrate downward................\n");
                printf("dg.search_startx: %f, dg.search_starty: %f\n", dg.search_start.x, dg.search_start.y);
                printf("dg.search_endx: %f, dg.search_endy: %f\n", dg.search_end.x, dg.search_end.y);
                printf("time: %f\n", t);
                if (LinearTraj(t, 0.005, dg.search_start, dg.search_end, target_x, target_y))
                {
                    dg.state++;
                    dg.delay_start = chrono::steady_clock::now();
                }
                printf("target_x: %f, target_y: %f\n", target_x, target_y);
                CartesianMotorCommand(traveler, target_x, target_y);
                
                break;
            // delay in extended position
            case 3:
                printf(".............delay in extended position................\n");
                delay_elapsed = chrono::duration<float>(now - dg.delay_start).count();
                if (delay_elapsed > delay)
                {
                    dg.state++;
                    dg.start = chrono::steady_clock::now();
                }
                
                break;
            // return to start position
            case 4:
                
                if (LinearTraj(t, 0.1f, dg.search_end, dg.search_start, target_x, target_y))
                {
                    dg.state++;
                }
                CartesianMotorCommand(traveler, target_x, target_y);
                printf(".............return to start position................\n");
                break;
            case 5:
                return true;
            }
            
            return false;
        }

        bool TrajectoriesParser::PenetrateAndShearRoutine(Traveler &traveler) {
            if (ps.first_iteration) {
                ps.first_iteration = false;
                ps.state = 0;

                float depth = -1.0f * (traveler.traj_data.shear_penetration_depth + traveler.traj_data.ground_height);
                float start_depth = -1.0f * (traveler.traj_data.ground_height - 0.03);
                ps.A = XY_pair(0.0f, start_depth);
                ps.B = XY_pair(0.0f, depth);
                ps.C = XY_pair(traveler.traj_data.shear_length, depth);
                
                ps.vel1 = traveler.traj_data.shear_penetration_speed;
                ps.vel2 = traveler.traj_data.shear_speed;
                ps.vel3 = traveler.traj_data.shear_return_speed;

                ps.delay1 = traveler.traj_data.shear_penetration_delay;
                ps.delay2 = traveler.traj_data.shear_delay;

                ps.curr = ps.A;
                ps.dest = ps.B;
                ps.curr_vel = ps.vel1;
            }
            float X, Y, t;
            switch(ps.state) 
            {
                case 0: // smooth start
                {
                    printf(".............go to start position................\n");
                    if (GoToPoint(traveler, ps.A) == 1) {
                        ps.state++;
                        ps.start = chrono::steady_clock::now();
                    }
                    break;
                }
                case 1: // }
                case 3: // movement states
                case 5: // }
                {
                    printf("..........move...........: state: %d\n", ps.state);
                    printf("move_startx: %f, move_starty: %f\n", ps.curr.x, ps.curr.y);
                    printf("move_endx: %f, move_endy: %f\n", ps.dest.x, ps.dest.y);
                    auto now = chrono::steady_clock::now();
                    // printTrajData(traveler);
                    t = chrono::duration<float>(now - ps.start).count();
                    if (LinearTraj(t, ps.curr_vel, ps.curr, ps.dest, X, Y)) {
                        // update state
                        ps.state++;
                        // update start time for delay state
                        ps.start = chrono::steady_clock::now();
                        ps.curr_delay = (ps.state == 2) ? ps.delay1 : ps.delay2;
                    }
                    // printf("curr_vel: %f, curr_vel: %f\n",ps.curr_vel, ps.curr_vel);
                    CartesianMotorCommand(traveler, X, Y);
                    break;
                }
                case 2: // delay states
                case 4: //
                {
                    printf("..........delay...........: state: %d\n", ps.state);
                    auto now = chrono::steady_clock::now();
                    ps.delay_elapsed = chrono::duration<float>(now - ps.start).count();
                    // float delay = (ps.state == 2) ? ps.delay1 : ps.delay2;
                    if (ps.delay_elapsed > ps.curr_delay)
                    {
                        ps.state++;
                        // update start time for linear traj state
                        ps.start = chrono::steady_clock::now();
                        ps.curr = (ps.state == 3) ? ps.B : ps.C;
                        ps.dest = (ps.state == 3) ? ps.C : ps.A;
                        ps.curr_vel = (ps.state == 3) ? ps.vel2 : ps.vel3;
                        
                    }
                    break;
                }
                default:
                    return true;  // state 6 is completion
            }
            traveler.traveler_control.Leg_lf.state_flag = ps.state;
            return false;
        }

        int TrajectoriesParser::GoToPoint(Traveler &traveler, XY_pair Goal)
        {
            if (GTP_first_iteration)
            {
                GTP_state = 0;
                GTP_first_iteration = false;
                // printf("First Iteration::\nGoal Point: %f, %f\n", Goal.x, Goal.y);
                if (!inBounds(Goal.x, Goal.y))
                {
                    return -1; // goal point is invalid
                }
                start_point = traveler.traveler_chassis.Leg_lf.toe_position;
                // printf("Starting from: %f, %f\n", start_point.x, start_point.y);
                if (validPath(start_point, Goal))
                {
                    // printf("Straight line path available\n");
                    swing_points.B = start_point;
                    GTP_state = 1;
                }
                else
                {
                    // printf("Swing points available\n");
                    swing_points = findSwingPoints(start_point, Goal);
                    GTP_state = 3;
                    swing = true;
                    float gamma, L;
                    PhysicalToAbstract(swing_points.A.x, swing_points.A.y, L, theta1, gamma);
                    PhysicalToAbstract(swing_points.B.x, swing_points.B.y, L, theta2, gamma);
                    theta_ = theta1;
                    float comp = (theta1 > theta2) ? -M_PI : M_PI;
                    dir = (theta2 - theta2 > comp) ? -1 : 1;
                    // printf("Theta 1: %f, Theta 2: %f, Dir = %d\n", theta1, theta2, dir);
                    // printf("Swing Pt A: (%f, %f) \n", swing_points.A.x, swing_points.A.y);
                    // printf("Swing Pt B: (%f, %f) \n", swing_points.B.x, swing_points.B.y);
                }

                GTP_start = chrono::steady_clock::now();
            }
            auto now = chrono::steady_clock::now();
            auto t = chrono::duration<float>(now - GTP_start).count();

            float X, Y;
            switch (GTP_state)
            {
            case 3:
                {
                if (LinearTraj(t, 0.1f, start_point, swing_points.A, X, Y))
                {
                    GTP_state--;
                }
                // printf("Case 3: Cartesian command to %f, %f\n", X, Y);
                CartesianMotorCommand(traveler, X, Y);
                // printf("Go to Point in progress..\n");
                }
                return 0;
            case 2:
                {
                if (abs(theta_ - theta2) > 0.05)
                {
                    theta_ += dir * 0.01;
                    AbstractMotorCommand(traveler, 0.105f, theta_);
                    // printf("Abstract command to theta: %f\n", theta_);
                }
                else
                {
                    GTP_state--;
                    GTP_start = chrono::steady_clock::now();
                }
                // printf("Case 2: Go to Point in progress..\n");
                // printf("State flag %d\n", GTP_state);
                }
                return 0;
            case 1: // last segment of trajectory
                {
                printf("going from SwingPoint B: (%f, %f) to goal: (%f, %f)\n", swing_points.B.x, swing_points.B.y, Goal.x, Goal.y);
                printf("time: %f\n", t);
                if (LinearTraj(t, 0.1f, swing_points.B, Goal, X, Y))
                {
                    // printf("Go to Point Complete.\n");
                    GTP_state--;
                }
                else {
                    CartesianMotorCommand(traveler, X, Y);
                }
                // printf("Cartesian command to %f, %f\n", X, Y);
                // printf("State flag %d\n", GTP_state);
                }
                return 0;

            default:
                GTP_first_iteration = true;
                // printf("completing go to point...\n");
                return 1;
            }
        }

        void TrajectoriesParser::generate_temp_speed(Traveler &traveler)
        {
            traveler.traveler_control.Leg_lf.axis0.motor_control_speed = 2;
            // traveler.traveler_chassis.Leg_lb.axis0.position
        }

        void TrajectoriesParser::generate_temp_traj(Traveler &traveler)
        {
            /**
            **NOTE:
            ** commands in form: traveler.traveler_gui._____
            ** are value set by GUI
            */

            /**
             * for debugging
             */
            // traveler.traveler_gui.start_flag = 1;
            // traveler.traveler_gui.drag_traj = 3;
            // traveler.traj_data.shear_penetration_depth = 15 / 100.0f;
            // traveler.traj_data.shear_penetration_speed = 10 / 100.0f;
            // traveler.traj_data.shear_penetration_delay = 3;
            // traveler.traj_data.shear_length = 17/ 100.0f;
            // traveler.traj_data.shear_speed = 10/ 100.0f;
            // traveler.traj_data.shear_delay = 2;
            // traveler.traj_data.shear_return_speed = 10 / 100.0f;

            int trajectory = traveler.traveler_gui.drag_traj;
            int run = traveler.traveler_gui.start_flag;

            

            /**
             * if run is false, reset parameters
             * and kill loop
             */
            if (!run)
            {
                // printTrajData(traveler);
                first_iteration = true;
                GTP_first_iteration = true;
                traverseParams.run = true;
                run_ = true;
                state_flag = 0;
                Pene_state = 0;
                WT_state = 0;
                ps.first_iteration = true;
                dg.first_iteration = true;
                ps.state = 0;
                return;
            }
            switch (trajectory)
            {
            // *Extrusion Trajectory
            case 1:
                if (first_iteration)
                {
                    printf("Extrusion Traj\n");
                }
                Penetrate(traveler);
                break;

            // *Workspace Traversal
            case 2:
                if (traverseParams.run)
                {
                    TraverseWorkspace(traveler, traverseParams);
                }
                break;

            // *Penetrate and Shear
            case 3:
                PenetrateAndShearRoutine(traveler);
                break;

            // *Static Leg movement
            case 4:
                if (first_iteration)
                {
                    theta_ = (float)traveler.traj_data.static_angle;
                    length_ = (float)traveler.traj_data.static_length;
                    AbstractToPhysical(length_, theta_, destination);
                    // printf("Static Move First Iteration to angle %f, length %f \n", theta_, length_);
                    // printf("Translated as cartesian point (%f, %f)\n", destination.x, destination.y);
                }
                // printf("Run condition = %d\n", run_);
                if (run_)
                {
                    int status = GoToPoint(traveler, destination);
                    run_ = (status == 0) ? true : false;
                }
                break;

            case 7:
            if (first_iteration)
                {
                    printf("Detecting Ground\n");
                }
                DetectGround(traveler);
                break;

            }
            // DEFAULT
            traveler.traj_data.current_t += 0.01000;

            if (run && first_iteration)
            {
                first_iteration = false;
            }
        }

        void TrajectoriesParser::printTrajData(Traveler &traveler)
        {
            printf("Extrude_Speed: %f\n", traveler.traj_data.extrude_speed);
            printf("Extrude Angle: %f\n", traveler.traj_data.extrude_angle);
            printf("extrude_depth: %f\n", traveler.traj_data.extrude_depth);
            printf("shear_penetration_depth: %f\n", traveler.traj_data.shear_penetration_depth);
            printf("shear_penetration_speed: %f\n", traveler.traj_data.shear_penetration_speed);
            printf("shear_penetration_delay: %f\n", traveler.traj_data.shear_penetration_delay);
            printf("shear_length: %f\n", traveler.traj_data.shear_length);
            printf("shear_speed: %f\n", traveler.traj_data.shear_speed);
            printf("shear_delay: %f\n", traveler.traj_data.shear_delay);
            printf("shear_return_speed: %f\n", traveler.traj_data.shear_return_speed);
            printf("workspace_angular_speed: %f\n", traveler.traj_data.workspace_angular_speed);
            printf("workspace_moving_angle: %f\n", traveler.traj_data.workspace_moving_angle);
            printf("workspace_time_delay: %f\n", traveler.traj_data.workspace_time_delay);
            printf("static_length: %f\n", traveler.traj_data.static_length);
            printf("static_angle: %f\n", traveler.traj_data.static_angle);
        }

    } // namespace control
} // namespace traveler_namespace
