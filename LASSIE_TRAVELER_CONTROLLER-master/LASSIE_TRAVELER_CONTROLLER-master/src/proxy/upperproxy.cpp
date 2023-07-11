/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 14:38:09
 */

#include "proxy/upperproxy.h"


/**
 * upperproxy - class to collect robot's information and trajectories from path
 * planning and decision making part. 
 * agile taur.
 */

namespace traveler_namespace{
namespace control{

upperproxy::upperproxy(std::string name) : Node(name){
    std::cout<<"Traveler Upper Proxy established"
                <<std::endl;
    GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/Gui_information", 10, std::bind(&upperproxy::handle_gui, this, _1));
}

void upperproxy::handle_gui
    (const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        int len = msg->data.size();
        traveler_leg.traveler_gui.start_flag = msg->data[0];
        traveler_leg.traveler_gui.drag_traj = msg->data[1];
        traveler_leg.traj_data.extrude_speed = msg->data[2] / 100.0f;
        traveler_leg.traj_data.extrude_angle = (msg->data[3] / -180 * M_PI) + M_PI;
        traveler_leg.traj_data.extrude_depth = msg->data[4] / 100.0f;
        traveler_leg.traj_data.shear_penetration_depth = msg->data[5] / 100.0f;
        traveler_leg.traj_data.shear_penetration_speed = msg->data[6] / 100.0f;
        traveler_leg.traj_data.shear_penetration_delay = msg->data[7];
        traveler_leg.traj_data.shear_length = msg->data[8]/ 100.0f;
        traveler_leg.traj_data.shear_speed = msg->data[9]/ 100.0f;
        traveler_leg.traj_data.shear_delay = msg->data[10];
        traveler_leg.traj_data.shear_return_speed = msg->data[11] / 100.0f;
        traveler_leg.traj_data.workspace_angular_speed = msg->data[12] / 100.0f;
        traveler_leg.traj_data.workspace_moving_angle = msg->data[13] / 180 * M_PI;
        traveler_leg.traj_data.workspace_time_delay = msg->data[14];
        traveler_leg.traj_data.static_length = msg->data[15] / 100.0f;
        traveler_leg.traj_data.static_angle = (msg->data[16] / -180 * M_PI) + M_PI;
        traveler_leg.traj_data.search_start = msg->data[17] / 100.0f;
        traveler_leg.traj_data.search_end = msg->data[18] / 100.0f;
        traveler_leg.traj_data.ground_height = msg->data[19] / 100.0f;
        traveler_leg.traj_data.back_speed = msg->data[20] / 100.0f;
    }

void upperproxy::UpdateGuiCommand(Traveler& traveler_){
    traveler_.traveler_gui = traveler_leg.traveler_gui;
    traveler_.traj_data = traveler_leg.traj_data;
}
void upperproxy::PublishStatusFeedback(Traveler& traveler_){
    if(traveler_.traveler_gui.status_update_flag == true){
        auto message = std_msgs::msg::Float64MultiArray();
        // std::cout <<  message.data[message.data.size() - 1] << std::endl;
        GUI_publisher->publish(message);
        traveler_.traveler_gui.status_update_flag = false;
    }
    
}

} //namespace control
} //namespace traveler_namespace