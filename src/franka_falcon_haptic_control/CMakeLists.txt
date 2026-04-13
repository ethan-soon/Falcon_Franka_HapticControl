cmake_minimum_required(VERSION 3.5)
project(franka_falcon_haptic_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(falcon_franka_bridge src/falcon_franka_bridge.cpp)

ament_target_dependencies(falcon_franka_bridge
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
)

install(TARGETS falcon_franka_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
