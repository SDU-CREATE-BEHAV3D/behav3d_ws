cmake_minimum_required(VERSION 3.8)
project(behav3d_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
# List all your .action and .srv files here
set(action_files
  action/PlanAndExecute.action
  action/PrintTime.action
)

set(srv_files
  srv/PlanWithMoveIt.srv
  srv/ComputeHandEye.srv
  srv/Pause.srv
  srv/Resume.srv
  srv/Abort.srv
  srv/CaptureSnapshot.srv
  srv/UpdatePrintConfig.srv
  srv/GetPrintStatus.srv
  srv/PlanCartesianPath.srv
  srv/PlanPilzLin.srv
  srv/PlanPilzPtp.srv
  srv/PlanPilzSequence.srv
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    moveit_msgs
)

# Ensure runtime deps are exported
ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_msgs
)

# Install package.xml so the ament index can see this package clearly
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
