/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-21 21:58:00
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:31:48
 */

#include "proxy/lowerproxy.h"

#define _USE_MATH_DEFINES
//#define DEBUG

/**
 * lowerproxy - class to publish control command to webots traveler_namespace or real
 * agile taur.
 */

const double TWO_PI = M_PI * 2;
// old motor offset: 1.3515 radians

using namespace std;

namespace traveler_namespace
{
namespace control
{

    lowerproxy::lowerproxy(std::string name) : Node(name)
    {
        std::cout << "Traveler Lower Proxy established"
                    << std::endl;
        // create a list of low level velocity command publisher
        joint0_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_position_controller/commands", 10);
        joint1_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_position_controller/commands", 10);
        joint0_speed_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);
        joint1_speed_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);
        controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/travelerstate", 10);
        // Robot_state_publisher = this->create_publisher<travelermsgs::msg::RobotState>
        //     ("/robot_state", 10);
        Leg1_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>
                                ("/dynamic_joint_states", 10, std::bind(&lowerproxy::handle_joint_state, this, _1));

        _count = 0;

        // RCLCPP_INFO(this->get_logger(), "Publisher created!!");
    }

    float lowerproxy::fmodf_0_pi(float f)
    {
        float result = fmodf(f, M_PI);
        if (result < 0)
            result += M_PI;
        return result;
    }

    float lowerproxy::fmodf_0_2pi(float f)
    {
        float r = fmodf(f, TWO_PI);
        if (r < 0)
            r += TWO_PI;
        return r;
    }

    float lowerproxy::fmodf_mpi_pi(float f)
    {
        if (f>0)
            return (fmodf(f+M_PI, TWO_PI) - M_PI);
        else
            return (fmodf(f-M_PI, TWO_PI) + M_PI);
    }

    void lowerproxy::getMeanDiffAngles(float &meanAng, float &diffAng)
    {
        float mot_pos0 = fmodf_0_2pi(-M0_OFFSET + traveler_leg_.traveler_chassis.Leg_lf.axis0.position);
        float mot_pos1 = fmodf_0_2pi(M1_OFFSET - traveler_leg_.traveler_chassis.Leg_lf.axis1.position);
        float r = mot_pos0 - mot_pos1;
        diffAng = fmodf_0_pi(0.5 * abs(r));
        meanAng = fmodf_0_2pi(mot_pos1 + diffAng);
        
        #ifdef DEBUG
            //printf("DEBUG:: Motor Pos: [%4.2f, %4.2f] | Diff Angle: %4.2f | Mean Angle: %4.2f | ", 
                    //mot_pos0, mot_pos1, diffAng_, meanAng_);
        #endif
    }

    void lowerproxy::calculateLength(float diffAng, float &proj, float &length)
    {
        proj = L1 * sinf(diffAng);
        length = sqrtf(L2 * L2 - proj * proj) + L1 * cosf(diffAng);
        
        #ifdef DEBUG
            //printf("Length DiffAng: %4.2f | L1Proj: %4.2f | Leg Length: %4.2f\n", diffAng, proj, length);
        #endif
    }

    void lowerproxy::calculationHelper()
    {
        // determine motor positions
        mot_pos0_ = fmodf_0_2pi(-M0_OFFSET + traveler_leg_.traveler_chassis.Leg_lf.axis0.position);
        mot_pos1_ = fmodf_0_2pi(M1_OFFSET - traveler_leg_.traveler_chassis.Leg_lf.axis1.position);
        float r = mot_pos0_ - mot_pos1_;
        
        // update private variables
        float diffAng = fmodf_0_pi(0.5 * abs(r));
        float meanAng = fmodf_0_2pi((mot_pos1_ + mot_pos0_)/2);
        if (mot_pos0_ < mot_pos1_) {
            meanAng -= M_PI;
            diffAng = M_PI - diffAng;
        }
        float l1proj = L1 * sinf(diffAng);
        float length = sqrtf(L2 * L2 - l1proj * l1proj) + L1 * cosf(diffAng);
        // float cosDiff = cosf(diffAng);
        // float sinDiff = sinf(diffAng);

        diffAng_ = diffAng;
        meanAng_ = meanAng;
        l1proj_ = l1proj;
        leg_length_ = length;
        
        // Debug output log
        #ifdef DEBUG
            //printf("DEBUG:: Motor Pos[0,1]: [%4.2f, %4.2f] | \n", mot_pos0_, mot_pos1_);
            //printf("Diff Angle: %4.2f | Mean Angle: %4.2f |Leg Length: %4.2f\n", diffAng, meanAng, l1proj, length);
        #endif
    }

    void lowerproxy::getToeVelocity()
    {
        float vel_0 = -(1.0 * traveler_leg_.traveler_chassis.Leg_lf.axis0.velocity) * TWO_PI;
        float vel_1 = 1.0 * traveler_leg_.traveler_chassis.Leg_lf.axis1.velocity * TWO_PI;
        
        // float filtered_vel_0 = lpf.update(vel_0, 0.01, 5);
        // float filtered_vel_1 = lpf.update(vel_1, 0.01, 5);
        // Polar Velocity: <theta direction, radial direction>
        float dummy = L1 * sin(diffAng_) / 2 - L1 * cos(diffAng_) / (4 * sqrtf(L2 * L2 - l1proj_ * l1proj_));
        float L_dot = dummy * (vel_1 - vel_0);
        float theta_dot = (vel_0 + vel_1) / 2;

        float vel_x = L_dot * sinf(meanAng_) - leg_length_ * theta_dot * cosf(meanAng_);
        float vel_y = -L_dot * cosf(meanAng_) - leg_length_ * theta_dot * sinf(meanAng_);

        traveler_leg_.traveler_chassis.Leg_lf.toe_velocity.x = vel_x;
        traveler_leg_.traveler_chassis.Leg_lf.toe_velocity.y = vel_y;
        
    }

    void lowerproxy::getToeForce()
    {
        float u0 = -1.0 * traveler_leg_.traveler_chassis.Leg_lf.axis0.effort;
        float u1 = traveler_leg_.traveler_chassis.Leg_lf.axis1.effort;

        // adjust the raw torque through our calibration
        float Iq_setpoint0 = u0/0.055;
        float Iq_setpoint1 = u1/0.055;

        //float u0_adjust = Iq_setpoint0 * (0.06966 - 0.0003501 * Iq_setpoint0);
        //float u1_adjust = Iq_setpoint1 * (0.06565 - 0.0001687 * Iq_setpoint1);
        float u0_adjust = Iq_setpoint0 * 0.055;
        float u1_adjust = Iq_setpoint1 * 0.055;

        //-0.0392, 0.5195 -> 146 deg
        //[-0.5290, -0.0358] -> 214 deg



        //float dummy = -2 / (-2 * l1proj_ + L1 * L1 * sinf(2 * diffAng_) / sqrtf(L2 * L2 - l1proj_ * l1proj_));
        float dummy = -2 / (2 * l1proj_ + L1 * L1 * sinf(diffAng_ * 2) / sqrtf(L2 * L2 - l1proj_ * l1proj_));
        
        // float arg1 = L1 * L1 * sinf(diffAng_ * 2); 
        // float arg2 = sqrtf(L2 * L2 - l1proj_ * l1proj_);
        // float denom = (2 * l1proj_ + L1 * L1 * sinf(diffAng_ * 2) / sqrtf(L2 * L2 - l1proj_ * l1proj_));

        // VALIDATED
        float ur = dummy * (u0_adjust - u1_adjust);
        float uth = u0_adjust + u1_adjust;

        
        float cosMean = cosf(meanAng_);
        float sinMean = sinf(meanAng_);

        float forceX = (ur * sinMean + uth * cosMean / leg_length_);
        float forceY = (-1.0f * ur * cosMean + uth * sinMean / leg_length_);

        traveler_leg_.traveler_chassis.Leg_lf.toe_force.y = forceY;
        traveler_leg_.traveler_chassis.Leg_lf.toe_force.x = forceX;

        traveler_leg_.traveler_chassis.Leg_lf.diffAng = diffAng_;
        traveler_leg_.traveler_chassis.Leg_lf.meanAng = meanAng_;
        traveler_leg_.traveler_chassis.Leg_lf.l1proj = l1proj_;
        traveler_leg_.traveler_chassis.Leg_lf.length = leg_length_;
        traveler_leg_.traveler_chassis.Leg_lf.dummy = dummy;



        #ifdef DEBUG
            printf("DEBUG:: Intermediate values: ur = %4.2f, uth = %4.2f \n", ur, uth);
            printf("Mean Angle: %4.2f | Diff Angle: %4.2f | Leg Length: %4.2f | L1 Proj: %4.2f\n", meanAng_, diffAng_, leg_length_, l1proj_);
            printf("Dummy: %4.2f | Arg 1: %4.2f | Arg 2: %4.2f | Denom: %4.2f\n", dummy, arg1, arg2, denom);
            printf("cosine Mean: %4.2f | sine mean: %4.2f\n", cosMean, sinMean);
            printf("Motor Torques [0,1]: [%4.4f, %4.4f] | Toe Force [X, Y]: [%4.2f, %4.2f]\n\n", u0, u1, forceX, forceY);
        #endif
    }

    void lowerproxy::getToePosition()
    {
        traveler_leg_.traveler_chassis.Leg_lf.toe_position.x = leg_length_ * sin(meanAng_);
        traveler_leg_.traveler_chassis.Leg_lf.toe_position.y = leg_length_ * cos(meanAng_);
    }

    void lowerproxy::handle_joint_state(const control_msgs::msg::DynamicJointState::SharedPtr msg)
    {
        traveler_leg_.traveler_chassis.Leg_lf.axis0.effort =
            msg->interface_values[0].values[0];
        traveler_leg_.traveler_chassis.Leg_lf.axis0.position =
            msg->interface_values[0].values[1];
        
        traveler_leg_.traveler_chassis.Leg_lf.axis1.effort =
            msg->interface_values[1].values[0];
        traveler_leg_.traveler_chassis.Leg_lf.axis1.position =
            msg->interface_values[1].values[1];
        
        
        // getMeanDiffAngles(meanAng_, diffAng_);
        // calculateLength(meanAng_, l1proj_, leg_length_);
        calculationHelper();
        getToeForce();
        getToeVelocity();
        getToePosition();

    }

    

     void lowerproxy::PublishControlCommand(Traveler &traveler_)
    {
        _count = _count + 0.01;
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(traveler_.traveler_control.Leg_lf.axis0.motor_control_position);
        auto message2 = std_msgs::msg::Float64MultiArray();
        message2.data.push_back(traveler_.traveler_control.Leg_lf.axis1.motor_control_position);
        auto message3 = std_msgs::msg::Float64MultiArray();
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_force.x);
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_force.y);
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_position.x);
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_position.y);
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_velocity.x);
        message3.data.push_back(traveler_.traveler_chassis.Leg_lf.toe_velocity.y);
        message3.data.push_back(traveler_.traveler_control.Leg_lf.theta_command);
        message3.data.push_back(traveler_.traveler_control.Leg_lf.length_command);
        message3.data.push_back(traveler_.traveler_control.Leg_lf.state_flag);

        joint0_publisher->publish(message);
        joint1_publisher->publish(message2);
        controller_state_publisher->publish(message3);

    } 

    void lowerproxy::PublishSpeedCommand(Traveler &traveler_)
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(traveler_.traveler_control.Leg_lf.axis0.motor_control_speed);
        joint0_speed_publisher->publish(message);

    }

    void lowerproxy::Estop()
    {
    }

    void lowerproxy::UpdateJoystickStatus(Traveler &traveler_)
    {
        traveler_.traveler_chassis = traveler_leg_.traveler_chassis;
    }

    void lowerproxy::send_request(Traveler &traveler_)
    {
        // first channel 
        auto request1 = std::make_shared<odrive_pro_srvs_msgs::srv::SetInputPos::Request>();
        request1->can_channel = 0;
        request1->axis = 0;
        request1->input_position = traveler_.traveler_control.Leg_lf.axis0.motor_control_position;  // Update this as needed
        request1->vel_ff = 0;          // Update this as needed
        request1->torque_ff = 0;       // Update this as needed

        auto result_future1 = client_->async_send_request(request1);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future1) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
        }
        else
        {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /odrive/set_input_pos");
        }

// Second channel 
        auto request2 = std::make_shared<odrive_pro_srvs_msgs::srv::SetInputPos::Request>();
        request2->can_channel = 1;
        request2->axis = 0;
        request2->input_position = traveler_.traveler_control.Leg_lf.axis1.motor_control_position;  // Update this as needed
        request2->vel_ff = 0;          // Update this as needed
        request2->torque_ff = 0;       // Update this as needed

        auto result_future2 = client_->async_send_request(request2);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future2) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
        }
        else
        {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service /odrive/set_input_pos");
        }  
    }

} // namespace control
} // namespace traveler_namespace
