# ROS Jazzy and MoveIt2 Setup Cheatsheet

This guide provides a comprehensive set of instructions for installing ROS 2 Jazzy, MoveIt2, and configuring a Universal Robot (UR) for use with MoveIt.

## ROS Jazzy Installation

These instructions are for a fresh installation on Ubuntu 24.04, based on the official documentation.

### Prerequisites: VSCode and Nano

Ensure you have a text editor like `nano` and an IDE like Visual Studio Code.

**Install nano:**
```bash
sudo apt update
sudo apt install nano
```

**Install Visual Studio Code:**
```bash
# Install dependencies
sudo apt install wget gpg apt-transport-https

# Import the Microsoft GPG key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null

# Add the VS Code repository
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | \
sudo tee /etc/apt/sources.list.d/vscode.list

# Update and install
sudo apt update
sudo apt install code
```

### Set Locale

Check and set the system locale to one that supports UTF-8.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Enable Required Repositories

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add the ROS 2 GPG Key

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add the ROS 2 Repository to Sources List

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Development Tools

```bash
sudo apt update && sudo apt install ros-dev-tools
```

### Install ROS 2 Jazzy Desktop

Update and upgrade packages before installing ROS 2.

```bash
sudo apt update
sudo apt upgrade
```

This guide uses the desktop installation:

```bash
sudo apt install ros-jazzy-desktop
```

### Setup Environment

Source the ROS 2 setup file. You can add this to your `.bashrc` to run it automatically in new terminals.

```bash
source /opt/ros/jazzy/setup.bash
```

To add it to your `.bashrc`:
```bash
nano ~/.bashrc
# Go to the end of the file and paste: source /opt/ros/jazzy/setup.bash
# Then save and exit.
```

Source the `.bashrc` file to apply the changes:
```bash
source ~/.bashrc
```

### Create and Build a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

Add the workspace's setup file to your `.bashrc` as well:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Source the `.bashrc` again:
```bash
source ~/.bashrc
```

### Test the Installation

Run a demo talker and listener node in separate terminals. If they coordinate, the installation is successful.

**Terminal 1:**
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
ros2 run demo_nodes_py listener
```

---

## MoveIt2 Installation from Source

This section details installing MoveIt2 from source. Note that this version will be missing tutorial files and examples. The version with tutorials is covered in the next section. These instructions are based on the official MoveIt2 documentation.

### Prerequisites

```bash
sudo apt install -y \
build-essential \
cmake \
git \
python3-colcon-common-extensions \
python3-flake8 \
python3-rosdep \
python3-setuptools \
python3-vcstool \
wget
```

Initialize `rosdep`:
```bash
sudo rosdep init
```

Update and upgrade packages:
```bash
sudo apt update
sudo apt dist-upgrade
rosdep update
```

Source the ROS 2 environment:
```bash
source /opt/ros/jazzy/setup.bash
```

### Uninstall Pre-existing Binaries

```bash
sudo apt remove ros-jazzy-moveit*
```

### Clone MoveIt2

Navigate to your workspace's `src` directory and clone the repository.

```bash
cd ~/ros2_ws/src
git clone https://github.com/moveit/moveit2.git -b jazzy
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_jazzy.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
```

### Configure Middleware (CycloneDDS)

The MoveIt developers recommend using CycloneDDS.

```bash
sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && source ~/.bashrc
```
*(Note: This only works by doing the firewall trick)*

Verify the RMW implementation:
```bash
echo $RMW_IMPLEMENTATION
```
You should see `rmw_cyclonedds_cpp`.

*(Note: This step was necessary in some devices and not in others)*

### Firewall Configuration for CycloneDDS

```bash
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
sudo ufw reload  # To refresh the firewall with new rules
```

### Build the Workspace

Navigate back to the workspace root and build. If your computer has limited RAM (e.g., 16GB), use the `--executor sequential` argument.

```bash
cd ~/ros2_ws/
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

---

## MoveIt2 Installation from Source (with Tutorials)

These instructions are based on the official MoveIt2 tutorials.

### Prerequisites

Ensure `rosdep` and `colcon-mixin` are installed and up-to-date.

```bash
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

**Install colcon mixin:**
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

### Clone Tutorials and Dependencies

In your workspace's `src` folder:
```bash
cd ~/ros2_ws/src
git clone -b main https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

### Remove Binary Installations

```bash
sudo apt remove ros-jazzy-moveit*
```

### Update Packages and Install Dependencies

```bash
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
```

### Build the Workspace

In your workspace folder:
```bash
cd ~/ros2_ws
colcon build --mixin release
```

### Test with Panda Robot Examples

Run the following commands in separate terminals.

**Terminal 1:**
```bash
ros2 launch moveit2_tutorials move_group.launch.py
```

**Terminal 2:**
```bash
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

---

## General Source Commands

Useful commands for sourcing setup files.

```bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc
```

For a `jazzy_ws` inside a Docker container:
```bash
# Before opening the image use:
xhost +local:docker
# Inside the container:
source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash
```

---

## UR and MoveIt Setup

This section explains how to set up a custom robot, specifically a Universal Robot (UR10e), within a MoveIt launch file by replicating the original sample files.

**System Architecture:**
```
 ┌──────────────────────────────────┐
 │  ur_robot_driver  (ROS 2 node)  │←— Connects over TCP/IP to the robot
 └──────────────────────────────────┘
              ▲ publishes joint_states
              │ offers /follow_joint_trajectory etc.
              ▼
 ┌──────────────────────────────────┐
 │      ros2_control controllers    │  (pos / vel / effort or FakeSystem)
 └──────────────────────────────────┘
              ▲
              │ robot_description, controller list
              ▼
 ┌──────────────────────────────────┐
 │ move_group node  (MoveIt 2 core)│
 └──────────────────────────────────┘
              ▲  planning scene, trajectories
              │
              ▼
 ┌──────────────────────────────────┐
 │   Your C++/Python “tutorial” app │
 └──────────────────────────────────┘
```

Choose one of the following two methods to build the UR files and configurations.

### Option A: Build UR Drivers from Source

This method might not be strictly necessary, but it is proven to work.

**1. Clone UR drivers and files:**
```bash
sudo apt install python3-colcon-common-extensions python3-vcstool
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ros2_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source install/setup.bash
```

**2. Build:**
```bash
colcon build --mixin release
```

**3. Run simulated robot:**
```bash
# 1) Bring up the UR10e “fake” hardware + controllers
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true

# 2) Bring up MoveIt2 (planning, RViz, etc.) against the fake hardware
# (Sometimes you have to manually move the joint/nullspace sliders in the motionplanning panel to be able to use the gimbal)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

### Option B: Binary Files Alternative (Untested)

**1. Download UR Drivers and Models:**
Install `ur_descriptions` (xacro and meshes):
```bash
sudo apt install ros-jazzy-ur-description
```
Install UR drivers:
```bash
sudo apt install ros-jazzy-ur-robot-driver
```
Build and source:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
Check if the packages are there:
```bash
ros2 pkg prefix ur_description     # prints /opt/ros/jazzy
ros2 pkg prefix ur_robot_driver
```

**2. Generate MoveIt Config:**
```bash
sudo apt install ros-jazzy-ur-moveit-config
```
Source and build:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**3. Run simulated robot:**
```bash
# 1) Bring up the UR10e “fake” hardware + controllers
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true

# 2) Bring up MoveIt2 (planning, RViz, etc.) against the fake hardware
# (Sometimes you have to manually move the joint/nullspace sliders in the motionplanning panel to be able to use the gimbal)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

---

## UR MoveIt Launch File (Work in Progress)

This section details creating a custom launch file for the UR robot with MoveIt.

### Create the Package

Go to your `src` folder:
```bash
cd ~/ros2_ws/src
```

Create a new package named `ur_moveit_demos`:
```bash
ros2 pkg create ur_moveit_demos --build-type ament_python --dependencies rclpy moveit_ros_planning_interface launch launch_ros
```

Enter the folder and create `launch` and `config` directories for organization:
```bash
cd ur_moveit_demos
mkdir launch
mkdir config
```

### Modify setup.py

Modify the `setup.py` file inside the package to look like this:
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur_moveit_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create_ros',
    maintainer_email='create_ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### Compile and Verify

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
source ~/.bashrc
```

Check that the package is recognized:
```bash
ros2 pkg list | grep ur_moveit_demos
```

### Create the Launch File

Now, create the launch file:
```bash
cd ~/ros2_ws/src/ur_moveit_demos/launch
nano ur10e_moveit_system.launch.py
```

Inside the file, paste the following code to launch the robot with mock hardware for simulations (For changing IP and Mock hardware you can do "ros2 launch ur_moveit_demos ur10e_moveit_system_live.launch.py \
        robot_ip:=192.168.1.130 use_mock_hardware:=false"):
```python
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch MoveIt + UR ROS 2 driver for a *real* UR10e by default.

    * Keeps the original defaults (127.0.0.1, mock‑hardware) so nothing breaks
      for simulation users.
    * Exposes two new launch arguments so they can be overridden from CLI:
        • ``robot_ip`` – IP of the physical controller
        • ``use_mock_hardware`` – ``true`` for simulation, ``false`` for real

    Examples
    --------
    ```bash
    # Simulation (old behaviour, no override needed)
    ros2 launch ur_moveit_demos ur10e_moveit_system_live.launch.py

    # Real robot at 192.168.1.130
    ros2 launch ur_moveit_demos ur10e_moveit_system_live.launch.py \
        robot_ip:=192.168.1.130 use_mock_hardware:=false
    ```
    """

    # --------------- Declare overridable arguments -----------------
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="IP address of the UR controller (real robot).",
    )

    mock_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="true = simulation/mock, false = real hardware",
    )

    # ---------------- Paths -----------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("ur_robot_driver"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("ur_moveit_config"), "launch"
    )

    joint_limits_yaml = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "joint_limits.yaml",
    )

    # ---------------- 1) UR driver -----------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # ---------------- 2) Small pause so driver is fully up -----------------
    pause_1s = TimerAction(
        period=1.0,
        actions=[LogInfo(msg="⌛  Driver ready; launching MoveIt …")],
    )

    # ---------------- 3) MoveIt stack -----------------
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "use_sim_time": "false",  # driver already runs in real-time clock
            "joint_limits": joint_limits_yaml,
        }.items(),
    )

    # ---------------- 4) RViz -----------------
    rviz_cfg = os.path.join(
        get_package_share_directory("ur_description"), "rviz", "view_robot.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # ---------------- LaunchDescription -----------------
    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            ur_driver,
            pause_1s,
            moveit_stack,
            rviz,
        ]
    )


```

### Build and Source

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
source ~/.bashrc
```

You can now run the launch file with:
```bash
ros2 launch ur_moveit_demos ur10e_moveit_system.launch.py
```

---

## C++ Movement Example

### Create the Package

Navigate to your workspace's `src` folder and create a new C++ package.
```bash
cd ~/ros2_ws/src
ros2 pkg create ur10e_moveit_tutorials --build-type ament_cmake \
  --dependencies rclcpp moveit_ros_planning_interface moveit_msgs
```

### Create the C++ Node

Create the source file for your node:
```bash
cd ur10e_moveit_tutorials/src
nano move_group_interface_ur10e.cpp
```

Paste one of the following examples into the file.

**Simple Goal Pose Example:**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp> // .hpp aquí
#include <moveit/planning_scene_interface/planning_scene_interface.hpp> // .hpp aquí
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur10e_move_group_interface");

  static const std::string PLANNING_GROUP = "ur_manipulator";
  static const std::string EEF_LINK = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EEF_LINK);

  geometry_msgs::msg::Pose target;
  target.position.x = 0.4;
  target.position.y = -0.2;
  target.position.z = 0.4;
  tf2::Quaternion q; q.setRPY(0, M_PI, 0);
  target.orientation = tf2::toMsg(q);
  move_group.setPoseTarget(target);

  // NUEVO: Creas un objeto Plan, y lo pasas por referencia
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) move_group.execute(plan);

  RCLCPP_INFO(node->get_logger(), "Demo terminado");
  rclcpp::shutdown();
  return 0;
}
```

**Relative Movement Example:**
This more complex node gets the current pose and adds a relative movement on each axis. The use of cores is crucial.
```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  /* --------------------------------------------------------------------
   * 1.  Initialize ROS and create a node that inherits parameters
   * declared elsewhere (e.g. robot_description, planning pipelines).
   * ------------------------------------------------------------------ */
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_opts;
  node_opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur10e_move_group_interface", node_opts);

  /* --------------------------------------------------------------------
   * 2.  Spin the node in a *background* thread so that MoveIt’s
   * CurrentStateMonitor can continuously receive /joint_states.
   * ------------------------------------------------------------------ */
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  /* --------------------------------------------------------------------
   * 3.  Construct MoveGroupInterface for the UR10e manipulator
   * and tell it which link is the end-effector.
   * ------------------------------------------------------------------ */
  const std::string PLANNING_GROUP = "ur_manipulator";
  const std::string EEF_LINK       = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EEF_LINK);

  /* --------------------------------------------------------------------
   * 4.  Wait (max 2 s) for a valid RobotState; abort if none arrives.
   * ------------------------------------------------------------------ */
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
  if (!current_state)
  {
      RCLCPP_ERROR(LOGGER, "Timed out waiting for /joint_states – aborting.");
      rclcpp::shutdown();
      return 1;
  }

  /* --------------------------------------------------------------------
   * 5.  Define a new pose goal = current pose + 0.05 m in X, Y, Z.
   * Orientation is kept unchanged.
   * ------------------------------------------------------------------ */
  geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
  target.position.x += 0.05;
  target.position.y += 0.05;
  target.position.z += 0.05;
  move_group.setPoseTarget(target);

  /* --------------------------------------------------------------------
   * 6.  Plan and execute the motion.
   * ------------------------------------------------------------------ */
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
      move_group.execute(plan);
      RCLCPP_INFO(LOGGER, "Motion executed successfully.");
  }
  else
      RCLCPP_WARN(LOGGER, "Planning failed.");

  rclcpp::shutdown();
  return 0;
}
```

### Create a Launch File for the Node

```bash
cd ~/ros2_ws/src/ur10e_moveit_tutorials
mkdir launch
cd launch
nano move_group_interface_ur10e.launch.py
```

Inside the file, add:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur10e_moveit_tutorials",
            executable="ur10e_move_group_interface",
            output="screen",
        )
    ])
```

### Edit CMakeLists.txt

Modify the `CMakeLists.txt` inside the package folder to include the necessary dependencies and executable.
```cmake
cmake_minimum_required(VERSION 3.8)
project(ur10e_moveit_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ur10e_moveit_interface src/move_group_interface_ur10e.cpp)
ament_target_dependencies(ur10e_moveit_interface
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS ur10e_moveit_interface
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

### Build and Source

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
source ~/.bashrc
```

### Run the System

First, run the launch file with RViz, MoveIt, and the UR robot drivers:
```bash
ros2 launch ur_moveit_demos ur10e_moveit_system.launch.py
```

In another terminal, run your C++ node:
```bash
ros2 launch ur10e_moveit_tutorials move_group_interface_ur10e.launch.py
```

---

## Unsorted Useful Commands

**Get current joint states:**
```bash
ros2 topic echo /joint_states --once
```

**Move each joint individually:**
*(Note: `joint_names` must match the order in `positions`. Be careful to avoid collisions.)*
```bash
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
      points: [
        {
          positions: [0.12895223915576935, -1.5521489896676322, 1.5388830343829554, 0.07252506792034907, -0.2845042387591761, -0.012816254292623341],
          velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

**Activate movement on the physical robot:**
```bash
ros2 service call /dashboard_client/play std_srvs/srv/Trigger
```

**Clean the Docker log file:**
```bash
sudo truncate -s 0 /var/log/syslog
```

**Echo joint states:**
```bash
ros2 topic echo /joint_states --once
```

**Source setup files:**
```bash
source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash
```

---

## To Be Done

- Figure out time parametrization and if it works with Cartesian movements.
- Calibrate the UR for ROS (check `ur_calibration`).
