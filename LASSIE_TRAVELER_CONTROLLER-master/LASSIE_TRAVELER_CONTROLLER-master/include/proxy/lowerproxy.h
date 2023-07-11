/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 22:01:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:14:21
 */

#ifndef LOWERPROXY_H_
#define LOWERPROXY_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "utils/lowpassfilter.hpp"
// #include "travelermsgs/msg/robot_state.hpp"
#include "odrive_pro_srvs_msgs/srv/set_input_pos.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace traveler_namespace{
namespace control{

	class lowerproxy : public rclcpp::Node {
	public:
	 ClientNode()
		: rclcpp::Node("client_node"), timer_(nullptr)
		{
		   client_ = this->create_client<odrive_pro_srvs_msgs::srv::SetInputPos>("odrive/set_input_pos");
		   //timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ClientNode::send_request, this));
		}
		lowerproxy(std::string name = "lower_proxy");
		void PublishSpeedCommand(Traveler &);
		void PublishControlCommand(Traveler &);
		void getMeanDiffAngles(float &meanAng, float &diffAng);
		void calculateLength(float diffAng, float &proj, float &length);
		void calculationHelper();
		void getToePosition();
		void getToeVelocity();
		void getToeForce();
		void Estop();
		void UpdateJoystickStatus(Traveler &);
		void send_request(Traveler &);

		/**
		 * @brief mods angle f to between -PI and PI
		 * @param f: input angle
		 * @return result in [-PI, PI]
		 */
		float fmodf_mpi_pi(float f);

		/**
		 * @brief mods angle f to between 0 and PI
		 * @param f: input angle
		 * @return result in [0, PI]
		 */
		float fmodf_0_pi(float f);

		/**
		 * @brief mods angle f to between 0 and 2PI
		 *
		 * @param f: input angle
		 * @return result in [0, 2*PI]
		 */
		float fmodf_0_2pi(float f);

		LowPassFilter lpf;

	private:
		float meanAng_;
		float diffAng_;
		float l1proj_;
		float leg_length_;
		float mot_pos0_;
		float mot_pos1_;

		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			joint0_publisher;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			joint1_publisher;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			joint0_speed_publisher;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			joint1_speed_publisher;
		rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
			Leg1_subscriber;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			controller_state_publisher;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			controller_state_publisher2;
		rclcpp::Client<odrive_pro_srvs_msgs::srv::SetInputPos>::SharedPtr client_;
		rclcpp::TimerBase::SharedPtr _timer;
		float _count;
		// float M0_OFFSET = 1.5519;
		// float M1_OFFSET = 1.5896;
		Traveler traveler_leg_;
		void handle_joint_state(const control_msgs::msg::DynamicJointState::SharedPtr msg);
	};

} //namespace control
} //namespace traveler_namespace



#endif
