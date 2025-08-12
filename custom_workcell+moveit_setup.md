Of course\! Here are your notes on setting up the custom workcell, formatted into a comprehensive `README.md` file. All the original notes and instructions have been preserved.

-----

# URDFs and MoveIt Configuration for I4O Custom Workcell

This guide details the setup of robot description files for a custom workcell involving a Universal Robot (UR), a camera, and custom tools. It covers the entire process from URDF file configuration to MoveIt setup and launch file creation.

> **Assumption:** This guide assumes that the UR drivers and MoveIt2 packages are already installed as explained in the repository's main setup document.

-----

## 1\. Clone Robot Description Packages from UR

First, navigate to your ROS 2 workspace's `src` directory and clone the official UR ROS 2 description package for the `jazzy` distribution.

```bash
cd ros2_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
```

-----

## 2\. Create and Configure a Custom Workcell Package

We will create a dedicated ROS 2 package to hold our custom workcell configuration.

### Create the Package

Inside the `src` folder of your workspace, create the package.

```bash
# Still in ros2_ws/src
ros2 pkg create i40_workcell --build-type ament_cmake --dependencies xacro rclcpp std_msgs trajectory_msgs
```

### Create Project Folders

Navigate into the newly created package and create the necessary directories for our configuration files.

```bash
cd i40_workcell
mkdir launch config worlds urdf meshes rviz
```

### Update `CMakeLists.txt`

To ensure our configuration files are installed correctly, add the following lines to your `CMakeLists.txt` file, right before the final `ament_package()` line.

```cmake
install(DIRECTORY launch urdf config meshes worlds rviz
  DESTINATION share/${PROJECT_NAME}
)
```

-----

## 3\. Prepare the URDF Files

We'll use `xacro` to create a modular and reusable robot description.

### Create the Main URDF Xacro

Navigate to the `urdf` directory and create the main xacro file for our workcell.

```bash
cd urdf
touch ur10_bolt.urdf.xacro
```

Edit `ur10_bolt.urdf.xacro` and add the following content. This file adapts the standard UR description to use our custom workcell macro.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ur">

  <xacro:arg name="name" default="ur"/>
  <xacro:include filename="$(find i40_workcell)/urdf/workcell_macro.xacro"/>

  <xacro:arg name="ur_type" default="ur10e"/>

  <xacro:arg name="joint_limits_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"/>
  <xacro:arg name="kinematics_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/default_kinematics.yaml"/>
  <xacro:arg name="physical_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml"/>
  <xacro:arg name="visual_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml"/>
  <xacro:arg name="force_abs_paths" default="false" />

  <link name="world" />

  <xacro:i40_workcell
      parent="world"
      ur_type="$(arg ur_type)"
      joint_limits_parameters_file="$(arg joint_limits_parameters_file)"
      kinematics_parameters_file="$(arg kinematics_parameters_file)"
      physical_parameters_file="$(arg physical_parameters_file)"
      visual_parameters_file="$(arg visual_parameters_file)"
      >
      <origin xyz="0 0 0" rpy="0 0 0" />          </xacro:i40_workcell>
</robot>
```

### Create the Workcell Macro

Now, create the macro file that defines the structure of our workcell, including the table, walls, and the robot itself.

```bash
touch workcell_macro.xacro
```

Paste the following content into `workcell_macro.xacro`. This macro references the UR robot macro and adds our custom objects. Make sure you have previously imported any necessary mesh files into the `meshes` folder.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>

  <xacro:macro name="i40_workcell" params="
      parent
      *origin
      ur_type
      joint_limits_parameters_file
      kinematics_parameters_file
      physical_parameters_file
      visual_parameters_file
      ">

    <joint name="table_base_joint" type="fixed">
      <xacro:insert_block name="origin" />
      <parent link="${parent}" />
      <child link="table" />
    </joint>

    <link name="table">
      <visual>
          <geometry>
          <mesh filename="package://i40_workcell/meshes/table.dae"/>
          </geometry>
          <origin rpy="0 0 0" xyz="-0.125 0.475 -0.74"/>
      </visual>
      <collision>
          <geometry>
          <mesh filename="package://i40_workcell/meshes/table.dae"/>
          </geometry>
          <origin rpy="0 0 0" xyz="-0.125 0.475 -0.74"/>
      </collision>
    </link>

    <link name="wall">
      <visual>
          <geometry>
          <mesh filename="package://i40_workcell/meshes/wall.dae"/>
          </geometry>
      </visual>
      <collision>
          <geometry>
          <mesh filename="package://i40_workcell/meshes/wall.dae"/>
          </geometry>
      </collision>
    </link>
    <joint name="base_to_wall" type="fixed">
      <parent link="table"/>
      <child link="wall"/>
      <origin xyz="0 -0.67 -0.74"/>
    </joint>

    <link name="robot_mount"/>
    <joint name="base_to_robot_mount" type="fixed">
      <parent link="table"/>
      <child link="robot_mount"/>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
    </joint>

    <xacro:ur_robot
      name="${ur_type}"
      tf_prefix="${ur_type}_"
      parent="robot_mount"
      joint_limits_parameters_file="${joint_limits_parameters_file}"
      kinematics_parameters_file="${kinematics_parameters_file}"
      physical_parameters_file="${physical_parameters_file}"
      visual_parameters_file="${visual_parameters_file}"
      >
      <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:ur_robot>
  </xacro:macro>
</robot>
```

-----

## 4\. Visualize the Robot Model

To check our progress, we'll create a simple launch file to visualize the robot in RViz.

### Create the Visualization Launch File

In the `launch` folder, create a file named `view_robot.launch.py`.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    description_package = FindPackageShare("i40_workcell")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "ur10_bolt.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "view_ur10_bolt.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", ur_type]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10e",
        )
    ]

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
```

### Build, Source, and Test

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
source ~/.bashrc
```

Launch the visualization:

```bash
ros2 launch i40_workcell view_robot.launch.py
```

> **YOU WON'T SEE ANYTHING\! FIX:**
>
> a) In RViz, change the "Fixed Frame" to `world`.
>
> b) Add a `RobotModel` display.
>
> c) Inside the `RobotModel` display options, set the "Description Topic" to `/robot_description`.
>
> You can save this RViz configuration. In our case, it's saved as `view_ur10_bolt.rviz` in the `rviz` folder and referenced by the launch file.

-----

## 5\. Add `ros2_control` for Robot Control

To make the robot description usable with `ros2_control` and the `ur_robot_driver`, we need to add control information.

### Create the Controlled URDF

Go to the `urdf` folder and create a new xacro file for the controlled robot.

```bash
cd ~/ros2_ws/src/i40_workcell/urdf
touch ur10_bolt_controlled.urdf.xacro
```

### Update `package.xml`

Add the following dependencies to your `package.xml` file. Also, remove the `<depend>xacro</depend>` line to avoid conflicts.

```xml
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>position_controllers</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>ur_controllers</exec_depend>
<exec_depend>ur_robot_driver</exec_depend>
<exec_depend>ur_client_library</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>controller_manager</exec_depend>
```

### Write the Controlled URDF File

Paste this content into `ur10_bolt_controlled.urdf.xacro`. This file includes our custom workcell macro and the `ros2_control` macro from the UR driver.

```xml
<?xml version="1.0"?>
<robot name="ur" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find i40_workcell)/urdf/workcell_macro.xacro"/>
  <xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro"/>

  <xacro:arg name="ur_type" default="ur10e"/>
  <xacro:arg name="joint_limits_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"/>
  <xacro:arg name="kinematics_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/default_kinematics.yaml"/>
  <xacro:arg name="physical_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml"/>
  <xacro:arg name="visual_parameters_file" default="$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml"/>

  <xacro:arg name="robot_ip" default="0.0.0.0"/>
  <xacro:arg name="headless_mode" default="false" />
  <xacro:arg name="ur_script_filename" default="$(find ur_client_library)/resources/external_control.urscript"/>
  <xacro:arg name="ur_output_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_output_recipe.txt"/>
  <xacro:arg name="ur_input_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_input_recipe.txt"/>
  <xacro:arg name="use_mock_hardware" default="false" />
  <xacro:arg name="mock_sensor_commands" default="false" />

  <link name="world" />

  <xacro:i40_workcell
      parent="world"
      ur_type="$(arg ur_type)"
      joint_limits_parameters_file="$(arg joint_limits_parameters_file)"
      kinematics_parameters_file="$(arg kinematics_parameters_file)"
      physical_parameters_file="$(arg physical_parameters_file)"
      visual_parameters_file="$(arg visual_parameters_file)"
      >
      <origin xyz="0 0 0" rpy="0 0 0" />          </xacro:i40_workcell>

  <xacro:ur_ros2_control
      name="$(arg ur_type)"
      tf_prefix="$(arg ur_type)_"
      kinematics_parameters_file="$(arg kinematics_parameters_file)"
      robot_ip="$(arg robot_ip)"
      script_filename="$(arg ur_script_filename)"
      output_recipe_filename="$(arg ur_output_recipe_filename)"
      input_recipe_filename="$(arg ur_input_recipe_filename)"
      use_mock_hardware="$(arg use_mock_hardware)"
      mock_sensor_commands="$(arg mock_sensor_commands)"
      headless_mode="$(arg headless_mode)"
      use_tool_communication="true"
  />

  </robot>
```

### Add Calibration File

Add your robot's calibration YAML file (e.g., `my_robot_calibration.yaml`, obtained from the `ur_calibration` utility) to the `config` directory.

-----

## 6\. Prepare Launch Files for the Driver

We'll create a set of launch files to start the robot driver with our custom configuration.

### Create `rsp.launch.py`

In your `launch` folder, create `rsp.launch.py` (Robot State Publisher launch). This file will be responsible for loading the controlled URDF and starting the `robot_state_publisher`.

```python
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    headless_mode = LaunchConfiguration("headless_mode")

    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")

    # Load description with necessary parameters
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("i40_workcell"),
                    "urdf",
                    "ur10_bolt_controlled.urdf.xacro",
                ]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "kinematics_parameters_file:=",
            kinematics_parameters_file,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("i40_workcell"),
                    "config",
                    "my_robot_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
```

### Test the `ur_control` Launch

Build your workspace and then test the setup by launching the main `ur_control.launch.py` from the driver, pointing it to your custom RSP launch file.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
source ~/.bashrc

# Then run:
ros2 launch ur_robot_driver ur_control.launch.py \
  description_launchfile:=$(ros2 pkg prefix i40_workcell)/share/i40_workcell/launch/rsp.launch.py \
  use_mock_hardware:=true \
  robot_ip:=123 \
  ur_type:=ur10e \
  tf_prefix:=ur10e_
```

> **Note:** You might need to change the fixed frame to `world` in RViz.

### Create a Wrapper Launch File (`start_robot.launch.py`)

To simplify the launch command, create a wrapper launch file in your `launch` directory called `start_robot.launch.py`.

```python
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",  # put your robot's IP address here
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("i4a40_workcell"),
                            "rviz",
                            "view_ur10_bolt.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("i40_workcell"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                }.items(),
            ),
        ]
    )
```

### Final Test of the Driver Launch

Build and source your workspace again.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
source ~/.bashrc
```

Now you can start the driver with a much simpler command:

```bash
# Start the driver with mocked hardware
ros2 launch i40_workcell start_robot.launch.py use_mock_hardware:=true ur_type:=ur10e

# Start the driver with real hardware (make sure the robot IP is correct)
ros2 launch i40_workcell start_robot.launch.py
```

> **Note:** When changing the `ur_type`, be sure you have the correct calibration `.yaml` file, otherwise the robot links will be broken.

-----

## 7\. Build the MoveIt Configuration

It's time to generate the MoveIt configuration package, which contains all the specific parameters (a lot of YAMLs) that MoveIt needs to plan for our robot.

### Use the MoveIt Setup Assistant

Launch the setup assistant:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Follow the instructions provided in the official UR ROS 2 documentation to generate the configuration package:
**[UR Tutorial: Build MoveIt Config](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_tutorials/my_robot_cell/doc/build_moveit_config.html)**

Save the generated package to the following path in your workspace:
`/home/create_ros/ros2_ws/src/i40_workcell_moveit_config`

### Modify MoveIt Config Files

You can make some adjustments to the generated files for better performance.

**`joint_limits.yaml`:**
Downscale the default velocity and acceleration for safer initial testing.

```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  ur10e_elbow_joint:
    has_acceleration_limits: true
    max_acceleration: 5.0
  ur10e_shoulder_lift_joint:
    has_acceleration_limits: true
    max_acceleration: 5.0
  ur10e_shoulder_pan_joint:
    has_acceleration_limits: true
    max_acceleration: 5.0
  ur10e_wrist_1_joint:
    has_acceleration_limits: true
    max_acceleration: 5.0
  ur10e_wrist_2_joint:
    has_acceleration_limits: true
    max_acceleration: 5.00
  ur10e_wrist_3_joint:
    has_acceleration_limits: true
    max_acceleration: 5.00


```

**`moveit_controllers.yaml`:**
Ensure this file is configured to use the `scaled_joint_trajectory_controller`.

```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - scaled_joint_trajectory_controller

  scaled_joint_trajectory_controller:
    type: FollowJointTrajectory
    joints:
      - ur10e_shoulder_pan_joint
      - ur10e_shoulder_lift_joint
      - ur10e_elbow_joint
      - ur10e_wrist_1_joint
      - ur10e_wrist_2_joint
      - ur10e_wrist_3_joint
    action_ns: follow_joint_trajectory
    default: true
```

### Build the MoveIt Package

Now, the old reliable:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
source ~/.bashrc
```

### Run Everything Together

1.  **Start the UR driver:**

    ```bash
    ros2 launch i40_workcell start_robot.launch.py use_mock_hardware:=true ur_type:=ur10e
    ```

2.  **Start the MoveIt `move_group` node:**

    ```bash
    ros2 launch i40_workcell_moveit_config move_group.launch.py
    ```

3.  **Start the MoveIt RViz plugin with interactive markers:**

    ```bash
    ros2 launch i40_workcell_moveit_config moveit_rviz.launch.py
    ```

> **Note:** Remember to update the IK solver if needed and import the specific solver YAMLs into the `i40_workcell_moveit_config/config` folder.

-----

## 8\. Appendix: Adding Attachments

### Adding a Camera (Orbbec Femto Bolt)

To add the camera to the robot model, edit your `workcell_macro.xacro` file.

1.  **Include the camera's xacro** next to the UR import:

    ```xml
    <xacro:include filename="$(find orbbec_description)/urdf/femto_bolt.urdf.xacro"/>
    ```

2.  **Instantiate the camera macro**, attaching it to the robot's tool frame (`tool0`). The origin offset is pre-configured to match the frames.

    ```xml
    <xacro:femto_bolt
      prefix="femto_"
      parent="${ur_type}_tool0"
      use_nominal_extrinsics="true"
    >
      <origin xyz="0 0.0555 0.0103" rpy="0 0 1.5708" />
    </xacro:femto_bolt>
    ```

### Adding a Custom Mesh (e.g., Camera Mount)

Here is an example of adding a custom mesh for a Femto camera mount in `workcell_macro.xacro`. Pay attention to the origin and orientation in your 3D modeling software (e.g., Rhino).

```xml
<link name="femto_mount">
  <visual>
    <geometry>
      <mesh filename="package://i40_workcell/meshes/femto_mount.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://i40_workcell/meshes/femto_mount.dae"/>
    </geometry>
  </collision>
</link>
<joint name="base_to_tool0" type="fixed">
  <parent link="${ur_type}_tool0"/>
  <child link="femto_mount"/>
  <origin xyz="0 0 0"/>
</joint>
```

### Settings for Importing Meshes from Rhino

For meshes to be imported correctly for MoveIt, use these specific export settings in Rhino:
a) Use **meters** as the unit system.
b) Export to the **Collada (`.dae`)** format.
c) In the export options, **uncheck** "Use plugin config" and **check** "Geometry only".