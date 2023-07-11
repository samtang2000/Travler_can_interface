cmake_minimum_required(VERSION 3.5)
project(odrivepro_can_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(odrive_pro_srvs_msgs REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(odrive_can_interface 
              src/odrive_can.cpp
              src/can_service.cpp
              src/can_publisher.cpp
              src/socketcan_interface.cpp)

ament_target_dependencies(odrive_can_interface
  rclcpp
  odrive_pro_srvs_msgs
)


install(TARGETS odrive_can_interface
        DESTINATION lib/${PROJECT_NAME})

ament_package() 
