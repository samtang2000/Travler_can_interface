cmake_minimum_required(VERSION 3.5)
project(traveler_high_controller)

set(CMAKE_BUILD_TYPE Debug)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
#find_package(travelermsgs REQUIRED)
#find_package(glog REQUIRED)
#find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/eigen3
    /usr/local/include/qpOASES
    /usr/local/include/
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
add_executable( traveler_high_controller
    src/main.cpp

    src/controller/controller_monitor.cpp
    src/controller/pid_speed_controller.cpp
    src/controller/Inverse_kinematics.cpp
    src/controller/trajectories_parser.cpp

    src/proxy/lowerproxy.cpp
    src/proxy/upperproxy.cpp

    src/utils/lowpassfilter.cpp
)
add_executable( shearing_speed
    src/shearing.cpp

    src/controller/controller_monitor.cpp
    src/controller/pid_speed_controller.cpp
    src/controller/Inverse_kinematics.cpp
    src/controller/trajectories_parser.cpp

    src/proxy/lowerproxy.cpp
    src/proxy/upperproxy.cpp

    src/utils/lowpassfilter.cpp
)

ament_target_dependencies(traveler_high_controller rclcpp std_msgs control_msgs)
ament_target_dependencies(shearing_speed rclcpp std_msgs control_msgs)
install(TARGETS
  traveler_high_controller
  shearing_speed
  DESTINATION lib/${PROJECT_NAME})

ament_package()
