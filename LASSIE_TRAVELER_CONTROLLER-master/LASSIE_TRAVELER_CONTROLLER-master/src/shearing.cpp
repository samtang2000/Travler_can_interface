/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:20:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:12:01
 */


#include "shearing.h"
#include "rclcpp/rclcpp.hpp"

/**
 * main - entrance of controller.
 * @param argc
 * @param argv
 * @return 0
 */


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv); //initial ros
	shearing_motor.traj_data.current_t = 0;
	ControllerMonitor& monitor = ControllerMonitor::GetStateMonitor(); //change controller
	TrajectoriesParser& traj_parser = TrajectoriesParser::GetTrajParser();
	std::shared_ptr<lowerproxy> Lower_proxy_ = std::make_shared<lowerproxy>(); //detect motor status, and publish motion command
	std::shared_ptr<upperproxy> Upper_proxy_ = std::make_shared<upperproxy>(); //detect motor status, and publish motion command
	monitor.Init(); //initial monitor
	traj_parser.Init();
	rclcpp::Rate loop_rate(100); //renew frequence 100HZ
	while(rclcpp::ok()){
		
		Lower_proxy_->UpdateJoystickStatus(shearing_motor); //update leg feedback status
		Upper_proxy_->UpdateGuiCommand(shearing_motor); //update gui command
		traj_parser.generate_temp_speed(shearing_motor); 
		Lower_proxy_->PublishSpeedCommand(shearing_motor); //publish control command
		Upper_proxy_->PublishStatusFeedback(shearing_motor); //publish current times
		rclcpp::spin_some(Upper_proxy_);
		rclcpp::spin_some(Lower_proxy_);
		loop_rate.sleep();
	}





	return 0;
}
