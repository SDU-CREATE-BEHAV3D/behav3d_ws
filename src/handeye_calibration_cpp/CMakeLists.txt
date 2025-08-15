cmake_minimum_required(VERSION 3.10)
project(handeye_calibration_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behav3d_cpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(standalone
  handeye_calibration_cpp/main.cpp)

ament_target_dependencies(standalone
  rclcpp
  behav3d_cpp)

install(TARGETS standalone
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()