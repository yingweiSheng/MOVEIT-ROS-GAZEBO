cmake_minimum_required(VERSION 3.8)
project(realtime_servo_er7robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(dual_arm_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(laser_servo_demo 
  src/laser_servo_demo.cpp
)
ament_target_dependencies(laser_servo_demo 
  rclcpp
  control_msgs
  geometry_msgs
  tf2_ros
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_move_group
)
add_executable(camera_servo_demo 
  src/camera_servo_demo.cpp
)
ament_target_dependencies(camera_servo_demo 
  rclcpp
  control_msgs
  geometry_msgs
  tf2_ros
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_move_group
)

add_executable(dual_arm_servo_server 
  src/dual_arm_servo_server.cpp
)
ament_target_dependencies(dual_arm_servo_server 
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  tf2_ros
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  tf2_eigen
  Eigen3
  sensor_msgs
  dual_arm_interfaces
)

add_executable(dual_arm_servo_client src/dual_arm_servo_client.cpp)
ament_target_dependencies(dual_arm_servo_client
  rclcpp
  rclcpp_action
  dual_arm_interfaces
  nlohmann_json
  std_msgs
)

install(
  TARGETS
    camera_servo_demo laser_servo_demo dual_arm_servo_server dual_arm_servo_client
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
