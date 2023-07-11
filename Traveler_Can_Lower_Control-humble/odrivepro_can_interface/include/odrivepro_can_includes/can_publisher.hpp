#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "odrive_pro_srvs_msgs/msg/odrive_status.hpp"
#include "odrive_pro_srvs_msgs/msg/set_input_pos.hpp"
#include "odrive_pro_srvs_msgs/msg/set_input_vel.hpp"
#include "odrive_pro_srvs_msgs/msg/set_input_torque.hpp"


class CanPublisher : public rclcpp::Node
{
public:
    CanPublisher(/* args */);
    ~CanPublisher();

private:
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    SocketcanInterface socket_axis0_read_;
    SocketcanInterface socket_axis1_read_;
    SocketcanInterface socket_get_iq_;
    SocketcanInterface socket_get_encoder_estimates_;
    SocketcanInterface socket_set_position_;
    SocketcanInterface socket_set_velocity_;
    SocketcanInterface socket_set_torque_;
    odrive_pro_srvs_msgs::msg::OdriveStatus odrive_status_msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<odrive_pro_srvs_msgs::msg::OdriveStatus>::SharedPtr publisher_;
    
    rclcpp::Subscription<odrive_pro_srvs_msgs::msg::SetInputPos>::SharedPtr input_pos_subscription_;
    rclcpp::Subscription<odrive_pro_srvs_msgs::msg::SetInputVel>::SharedPtr input_vel_subscription_;
    rclcpp::Subscription<odrive_pro_srvs_msgs::msg::SetInputTorque>::SharedPtr input_torque_subscription_;
    rclcpp::Clock ros_clock_;

    void timerCallback();
    void updateStatusCallback();
    void set_input_pos_callback(const odrive_pro_srvs_msgs::msg::SetInputPos::SharedPtr  msg);
    void set_input_vel_callback(const odrive_pro_srvs_msgs::msg::SetInputVel::SharedPtr  msg);
    void set_input_torque_callback(const odrive_pro_srvs_msgs::msg::SetInputTorque::SharedPtr  msg);
};