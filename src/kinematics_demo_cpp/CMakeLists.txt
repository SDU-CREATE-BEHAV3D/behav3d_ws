cmake_minimum_required(VERSION 3.22)
project(kinematics_demo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behav3d_cpp REQUIRED)

add_executable(demo kinematics_demo_cpp/demo.cpp)

target_link_libraries(demo
  rclcpp::rclcpp
)

ament_target_dependencies(demo
  rclcpp
  std_msgs
  geometry_msgs
  behav3d_cpp
)


install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
