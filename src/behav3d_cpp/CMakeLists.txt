cmake_minimum_required(VERSION 3.22)
project(behav3d_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# ─── Library target ──────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/motion_controller.cpp
  src/target_builder.cpp
  src/trajectory_builder.cpp
  src/util.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# ─── Install ─────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}
  EXPORT  export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ─── ament export ────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  eigen3_cmake_module
)

ament_package()