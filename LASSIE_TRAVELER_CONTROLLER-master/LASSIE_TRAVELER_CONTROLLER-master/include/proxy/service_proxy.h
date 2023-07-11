#include "rclcpp/rclcpp.hpp"
#include "odrive_pro_srvs_msgs/srv/set_input_pos.hpp"



class ClientNode : public rclcpp::Node
{
public:
  ClientNode()
  : rclcpp::Node("client_node"), timer_(nullptr)
  {
    client_ = this->create_client<odrive_pro_srvs_msgs::srv::SetInputPos>("odrive/set_input_pos");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ClientNode::send_request, this));
  }

  void send_request(Traveler &);
 
 
private:
  rclcpp::Client<odrive_pro_srvs_msgs::srv::SetInputPos>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
