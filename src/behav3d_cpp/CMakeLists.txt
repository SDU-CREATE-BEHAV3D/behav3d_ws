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
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

# ─── Library target ──────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/motion_controller.cpp
  src/target_builder.cpp
  src/trajectory_builder.cpp
  src/util.cpp
  src/motion_visualizer.cpp
  src/camera_manager.cpp
  src/session_manager.cpp
  src/handeye_calibration.cpp
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
  moveit_visual_tools
  std_srvs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  nlohmann_json::nlohmann_json
  yaml-cpp
)

# ─── Hand-eye executable target ─────────────────────────────────────────────
add_executable(handeye_calibration_cpp
  src/handeye_main.cpp
)

ament_target_dependencies(handeye_calibration_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  cv_bridge
)

# Link against our shared library so we reuse implementations
 target_link_libraries(handeye_calibration_cpp
  ${PROJECT_NAME}
)

# Install the binary into the standard ROS 2 location for this package
install(TARGETS handeye_calibration_cpp DESTINATION lib/${PROJECT_NAME})

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
  moveit_visual_tools
  std_srvs
  cv_bridge
  nlohmann_json
  yaml-cpp
)

ament_package()
