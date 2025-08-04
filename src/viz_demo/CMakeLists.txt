cmake_minimum_required(VERSION 3.10)
project(viz_demo)

# ======================
# Find dependencies
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)

# ======================
# Build executable
# ======================
add_executable(demo src/demo.cpp)

target_include_directories(demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(demo
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
)

# ======================
# Install targets and launch files
# ======================
install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ======================
# Package export
# ======================
ament_package()
