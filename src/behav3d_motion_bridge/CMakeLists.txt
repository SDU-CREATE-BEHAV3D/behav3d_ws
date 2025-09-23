cmake_minimum_required(VERSION 3.16)
project(behav3d_motion_bridge)

find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behav3d_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(motion_bridge_node src/motion_bridge_node.cpp)
ament_target_dependencies(motion_bridge_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  behav3d_interfaces
  rclcpp rclcpp_action
  control_msgs visualization_msgs
)

install(TARGETS motion_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
