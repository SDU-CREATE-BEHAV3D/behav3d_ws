cmake_minimum_required(VERSION 3.22)
project(kinematics_demo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behav3d_cpp REQUIRED)

add_executable(${PROJECT_NAME}_node
  kinematics_demo_cpp/demo.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  behav3d_cpp
)


install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
