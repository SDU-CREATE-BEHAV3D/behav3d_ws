cmake_minimum_required(VERSION 3.22)
project(behav3d_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behav3d_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- executables ---
add_executable(demo   behav3d_demo/demo.cpp)
add_executable(mancap behav3d_demo/mancap.cpp)

# Usa ament_target_dependencies para propagar includes/libs de ROS2
ament_target_dependencies(demo
  rclcpp
  std_msgs
  geometry_msgs
  behav3d_cpp
  tf2 tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(mancap
  rclcpp
  std_msgs
  geometry_msgs
  behav3d_cpp
  tf2 tf2_ros tf2_geometry_msgs
  nlohmann_json
)

# (no necesitas target_link_libraries(rclcpp::rclcpp) si usas ament_target_dependencies)

# --- install ---
install(TARGETS
  demo
  mancap
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
