#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "odrivepro_can_includes/can_service.hpp"
#include "odrivepro_can_includes/can_publisher.hpp"


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto server = std::make_shared<CanService>();
  exec.add_node(server);

  auto publisher = std::make_shared<CanPublisher>();
  exec.add_node(publisher);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
