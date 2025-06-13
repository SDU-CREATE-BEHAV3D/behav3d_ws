# INSTALL ROS JAZZY
As if the os was new on 24.04:
According to https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Install vscode and nano if you dont have them:

```bash
    sudo apt update
    sudo apt install nano
```

And VScode:
```bash
    # Install dependencies
    sudo apt install wget gpg apt-transport-https
```

```bash
    # Import the Microsoft GPG key
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null
```

```bash
    # Add the VS Code repository
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | \
    sudo tee /etc/apt/sources.list.d/vscode.list
```

```bash
    # Update and install
    sudo apt update
    sudo apt install code
```

set locale:

```bash
    locale  # check for UTF-8
```

```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
```

```bash
    locale  # verify settings
```

Enable reqs:

```bash
    sudo apt install software-properties-common
```

```bash
    sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```bash
    sudo apt update && sudo apt install curl -y
```

```bash
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install tools:
```bash
    sudo apt update && sudo apt install ros-dev-tools
```

Install el ROS:
```bash
    sudo apt update
```

```bash
    sudo apt upgrade
```

We go for desktop install:

```bash
    sudo apt install ros-jazzy-desktop
```

setup environment:
```bash
    source /opt/ros/jazzy/setup.bash
    (you may add this to the bashrc to avoid doing it always with:)
```

```bash
    nano ~/.bashrc
    (go to the end and paste: source /opt/ros/jazzy/setup.bash    then save and exit)
    
```
source the bashrc 
```bash
    source ~/.bashrc
```

CREATE THE WORKSPACE and build!:

```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
```

add the source of the workspace to the bashrc too either with nano or with this single line:

```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

We source again:

```bash
    source ~/.bashrc
```
You can try the install by running a demo node talker and listener in different terminals:
```bash
    on one terminal run:
     ros2 run demo_nodes_cpp talker
    on the other run:
     ros2 run demo_nodes_py listener
```
if everything works they should coordinate!

## EL FIN

<<<<<<<<<<<<<<INSTALL Moveit2 from Source>>>>>>>>>>>>>>>
WE WILL BE MISSING THE TUTORIAL FILES AND EXAMPLES NOTE THAT (the version with tutorials is in the nextr chunk)
According to: https://moveit.ai/install-moveit2/source/

Install Prerequisites:

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

now send:

   sudo rosdep init

then:
```bash
    sudo apt update
    sudo apt dist-upgrade
    rosdep update
```
Source jik:
   source /opt/ros/jazzy/setup.bash

Uninstall preexistances:
```bash
    sudo apt remove ros-jazzy-moveit*
```

go to src in workspace:

```bash
    cd ~/ros2_ws/src
```

CLONE IT: 
```bash
    git clone https://github.com/moveit/moveit2.git -b jazzy
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_jazzy.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
```

(THIS ONLY WORK BY DOING THIS FIREWALL TRICK) Then do the middleware thing for using cyclone as they recommend:
```bash
    sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && source ~/.bashrc
```

(FUCKYOU CYCLONE AND MOVEIT DEVS)If youy check with this command you should see rmw_cyclonedds_cpp:

```bash
    echo $RMW_IMPLEMENTATION
```

DO THIS FIREWALL THINGY TO MAKE THIS CYCLONE MIDDLEWARE WORK:

sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
sudo ufw reload  # To refresh the firewall with new rules

Now back to the workspace and build (if the computer is limited to 16gb of ram you should use the executor sequential command)

```bash
    cd ~/ros2_ws/
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

## EL FIN

<<<<<<<<<<<<<<INSTALL Moveit2 from Source and tutorials>>>>>>>>>>>>>>>
According to https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html

Be sure to have rosdeps and colcon mixin:
```bash
    sudo rosdep init
    rosdep update
    sudo apt update
    sudo apt dist-upgrade
```
mixin:
```bash
    sudo apt install python3-colcon-common-extensions
    sudo apt install python3-colcon-mixin
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default
```

Clone tutorials and dependencies in src folder with:
```bash
    cd ~/ros2_ws/src
    git clone -b main https://github.com/moveit/moveit2_tutorials
```

```bash
    vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

Remove binary id installed:
```bash
    sudo apt remove ros-jazzy-moveit*
```
Update pkgs:
```bash
    sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
```
Build in workspace folder:
```bash
    cd ~/ros2_ws
    colcon build --mixin release
```

You now can try the sample files for moving a panda robot:
```bash
    Run this in one terminal:
        ros2 launch moveit2_tutorials move_group.launch.py
    In another with that terminal running do:
        ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

SOURCE COMMANDS

source ~/ros2_ws/install/setup.bash
source ~/.bashrc

For the jazzy_ws in docker
```bash
    Before opening image use: 
    xhost +local:docker
```
source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash


## UR MOVEIT SETUP

Here we will se how to setup a custom robot in this case the UR into the a moveit launchfile. The idea is to take the original sample file and replicate it with the UR10e.

 ┌──────────────────────────────────┐
 │  ur_robot_driver  (ROS 2 node)  │←— Connects over TCP/IP to the robot
 └──────────────────────────────────┘
```bash
              ▲ publishes joint_states
              │ offers /follow_joint_trajectory etc.
              ▼
```
 ┌──────────────────────────────────┐
 │      ros2_control controllers    │  (pos / vel / effort or FakeSystem)
 └──────────────────────────────────┘
```bash
              ▲
              │ robot_description, controller list
              ▼
```
 ┌──────────────────────────────────┐
 │ move_group node  (MoveIt 2 core)│
 └──────────────────────────────────┘
```bash
              ▲  planning scene, trajectories
              │
              ▼
```
 ┌──────────────────────────────────┐
 │   Your C++/Python “tutorial” app │
 └──────────────────────────────────┘

There are two ways for building the UR files and configurations choose only one:

```bash
    A) (Build Ur Drivers from src (might not be really necessary but it works))
        Clone Ur drivers and files:
```

```bash
            sudo apt install python3-colcon-common-extensions python3-vcstool
            cd ~/ros2_ws/src
            git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
            cd ~/ros2_ws/
            rosdep update
            rosdep install --from-paths src --ignore-src -r -y
            source install/setup.bash
```

```bash
        Build:
```

```bash
            colcon build --mixin release
```


```bash
        Run simulated robot?
```

```bash
        # 1) Bring up the UR10e “fake” hardware + controllers
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true
        # 2) Bring up MoveIt2 (planning, RViz, etc.) against the fake hardware (Some times you have to manually move the joint/nullspace sliders in the motionplanning panel to be able to use the gimball)
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```




```bash
    B) &&&UNTESTED Binary files alternative&&&&&
```

```bash
        1) DOWNLOAD UR DRIVERS AND MODELS
        Install ur_descriptions (xacro and meshes)
```

```bash
            sudo apt install ros-jazzy-ur-description
```

```bash
        Install UR drivers: 
```

```bash
            sudo apt install ros-jazzy-ur-robot-driver
```

```bash
        Build and source
```

```bash
            cd ~/ros2_ws
            rosdep install --from-paths src --ignore-src -r -y
            colcon build
            source install/setup.bash
```

```bash
        check if its there:
```

```bash
            ros2 pkg prefix ur_description     # prints /opt/ros/jazzy
            ros2 pkg prefix ur_robot_driver
```


```bash
        2) GENERATE MOVEIT CONFIG
```

```bash
            sudo apt install ros-jazzy-ur-moveit-config
```

```bash
        source and build
            cd ~/ros2__ws
            colcon build
            source install/setup.bash
```

```bash
        Run simulated robot?
```

```bash
        # 1) Bring up the UR10e “fake” hardware + controllers
        ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true
        # 2) Bring up MoveIt2 (planning, RViz, etc.) against the fake hardware (Some times you have to manually move the joint/nullspace sliders in the motionplanning panel to be able to use the gimball)
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```



## 
>>>>>>>>>>>>>>>>UR move it and Launch File WIP:<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

Go to src folder:
```bash
    cd ~/ros2_ws/src
```

Create pkg with dependencies it will be called ur_moveit_demos:

```bash
    ros2 pkg create ur_moveit_demos --build-type ament_python --dependencies rclpy moveit_ros_planning_interface launch launch_ros
```

Enter the folder and create this folders (Just to have them in order)

```bash
    cd ur_moveit_demos
```

```bash
    mkdir launch
```
 //   mkdir config

Modify setup.py inside the pck to leave it like:

```bash
    from setuptools import find_packages, setup
    import os
    from glob import glob
```

```bash
    package_name = 'ur_moveit_demos'
```

```bash
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
Compile and verify:

```bash
    cd ~/ros2_ws
    colcon build 
    source install/setup.bash
    source ~/.bashrc
```

Check that its there:
```bash
    ros2 pkg list | grep ur_moveit_demos
```

Now we will create the launch files:
```bash
    cd ~/ros2_ws/src/ur_moveit_demos/launch
    nano ur10e_moveit_system.launch.py
```

```bash
    Inside the file paste and save this launch the robot as mock hardware for simulations: 
```

```bash
        from launch import LaunchDescription
        from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch_ros.actions import Node
        from ament_index_python.packages import get_package_share_directory
        import os
```


```bash
        def generate_launch_description():
```

```bash
            ur_launch_dir = os.path.join(
                get_package_share_directory("ur_robot_driver"), "launch"
            )
            moveit_launch_dir = os.path.join(
                get_package_share_directory("ur_moveit_config"), "launch"
            )
```

```bash
            # ────────── 1) Driver UR (mock) ──────────
            ur_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ur_launch_dir, "ur_control.launch.py")
                ),
                launch_arguments={
                    "ur_type": "ur10e",
                    "robot_ip": "127.0.0.1",
                    "use_mock_hardware": "true",
                    "launch_rviz": "false",
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                
                }.items(),
            )
```

```bash
            # ────────── 2) Pausa breve ──────────
            pause_5s = TimerAction(
                period=1.0,
                actions=[LogInfo(msg="⌛ Driver listo; arrancamos MoveIt…")],
            )
```

```bash
            # ────────── 3) MoveIt ──────────
            moveit_stack = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_launch_dir, "ur_moveit.launch.py")
                ),
                launch_arguments={
                    "ur_type": "ur10e",
                    "use_sim_time": "false",        
                }.items(),
            )
```

```bash
            # ────────── 4) RViz ──────────
            rviz_cfg = os.path.join(
                get_package_share_directory("ur_description"),
                "rviz",
                "view_robot.rviz",
            )
            rviz = Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                output="screen",
                parameters=[{"use_sim_time": False}],  
            )
```

```bash
            return LaunchDescription([
                ur_driver,
                pause_5s,
                moveit_stack,
                rviz,
            ])
```


Build and source:

```bash
    cd ~/ros2_ws
    colcon build 
    source install/setup.bash
    source ~/.bashrc
```

You can run the lunch with:
```bash
    ros2 launch ur_moveit_demos ur10e_moveit_system.launch.py
```

We will create a Ros pkg to store our launch nodes


## C MOVEMENT EXAMPLE:

cd ~/ros2_ws/src
ros2 pkg create ur10e_moveit_tutorials --build-type ament_cmake \
  --dependencies rclcpp moveit_ros_planning_interface moveit_msgs

c++ commands node:

```bash
    cd ur10e_moveit_tutorials/src
    nano move_group_interface_ur10e.cpp
```

Paste inside (This is a simple goal pose example):

```bash
        #include <rclcpp/rclcpp.hpp>
        #include <moveit/move_group_interface/move_group_interface.hpp> // .hpp aquí
        #include <moveit/planning_scene_interface/planning_scene_interface.hpp> // .hpp aquí
        #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

```bash
        int main(int argc, char** argv)
        {
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("ur10e_move_group_interface");
```

```bash
        static const std::string PLANNING_GROUP = "ur_manipulator";
        static const std::string EEF_LINK = "tool0";
```

```bash
        moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
        move_group.setEndEffectorLink(EEF_LINK);
```

```bash
        geometry_msgs::msg::Pose target;
        target.position.x = 0.4;
        target.position.y = -0.2;
        target.position.z = 0.4;
        tf2::Quaternion q; q.setRPY(0, M_PI, 0);
        target.orientation = tf2::toMsg(q);
        move_group.setPoseTarget(target);
```

```bash
        // NUEVO: Creas un objeto Plan, y lo pasas por referencia
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
```

```bash
        if (success) move_group.execute(plan);
```

```bash
        RCLCPP_INFO(node->get_logger(), "Demo terminado");
        rclcpp::shutdown();
        return 0;
        }
```

```bash
    As an alternative here is a more complex node that get the current pose and add an relative movement in each axis. Its crucial the use of cores:
```

```bash
        #include <rclcpp/rclcpp.hpp>
        #include <moveit/move_group_interface/move_group_interface.hpp>
        #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
```

```bash
        #include <moveit_msgs/msg/display_robot_state.hpp>
        #include <moveit_msgs/msg/display_trajectory.hpp>
        #include <moveit_msgs/msg/attached_collision_object.hpp>
        #include <moveit_msgs/msg/collision_object.hpp>
        #include <moveit_visual_tools/moveit_visual_tools.h>
        #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

```bash
        static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
```

```bash
        int main(int argc, char** argv)
        {
```
## /*
```bash
        * 1.  Initialize ROS and create a node that inherits parameters
        *     declared elsewhere (e.g. robot_description, planning pipelines).
```
## * */
```bash
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions node_opts;
        node_opts.automatically_declare_parameters_from_overrides(true);
        auto node = rclcpp::Node::make_shared("ur10e_move_group_interface", node_opts);
```

## /*
```bash
        * 2.  Spin the node in a *background* thread so that MoveIt’s
        *     CurrentStateMonitor can continuously receive /joint_states.
```
## * */
```bash
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        std::thread([&executor]() { executor.spin(); }).detach();
```

## /*
```bash
        * 3.  Construct MoveGroupInterface for the UR10e manipulator
        *     and tell it which link is the end-effector.
```
## * */
```bash
        const std::string PLANNING_GROUP = "ur_manipulator";
        const std::string EEF_LINK       = "tool0";
```

```bash
        moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
        move_group.setEndEffectorLink(EEF_LINK);
```

## /*
```bash
        * 4.  Wait (max 2 s) for a valid RobotState; abort if none arrives.
```
## * */
```bash
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
        if (!current_state)
        {
            RCLCPP_ERROR(LOGGER, "Timed out waiting for /joint_states – aborting.");
            rclcpp::shutdown();
            return 1;
        }
```

## /*
```bash
        * 5.  Define a new pose goal = current pose + 0.05 m in X, Y, Z.
        *     Orientation is kept unchanged.
```
## * */
```bash
        geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
        target.position.x += 0.05;
        target.position.y += 0.05;
        target.position.z += 0.05;
        move_group.setPoseTarget(target);
```

## /*
```bash
        * 6.  Plan and execute the motion.
```
## * */
```bash
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
```

```bash
        if (success)
        {
            move_group.execute(plan);
            RCLCPP_INFO(LOGGER, "Motion executed successfully.");
        }
        else
            RCLCPP_WARN(LOGGER, "Planning failed.");
```

```bash
        rclcpp::shutdown();
        return 0;
        }
```

Create Launchfile for the node:

```bash
    cd ~/ros2_ws/src/ur10e_moveit_tutorials
    mkdir launch
    cd launch
    nano move_group_interface_ur10e.launch.py
```

```bash
    Inside you add:
```

```bash
        from launch import LaunchDescription
        from launch_ros.actions import Node
```

```bash
        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package="ur10e_moveit_tutorials",
                    executable="ur10e_move_group_interface",
                    output="screen",
                )
            ])
```

Edit cmakelists inside the pkg folder like this (there are some extra packages that for now are not in use):

```bash
    cmake_minimum_required(VERSION 3.8)
    project(ur10e_moveit_tutorials)
```

```bash
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
```

```bash
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(moveit_ros_planning_interface REQUIRED)
    find_package(moveit_msgs REQUIRED)
    find_package(moveit_visual_tools REQUIRED)          #  << NUEVA línea
    find_package(tf2_geometry_msgs REQUIRED)      
```

```bash
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
```

```bash
    add_executable(ur10e_move_group_interface src/move_group_interface_ur10e.cpp)
    ament_target_dependencies(ur10e_move_group_interface
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    moveit_visual_tools
    tf2_geometry_msgs 
    geometry_msgs
    )
```

```bash
    install(TARGETS ur10e_move_group_interface
            DESTINATION lib/${PROJECT_NAME})
```

```bash
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

```bash
    ament_package()
```

Build and source:

```bash
    cd ~/ros2_ws
    colcon build 
    source install/setup.bash
    source ~/.bashrc
```

Run the launch file with Rviz, Moveit and the UR robot drivers:

```bash
    ros2 launch ur_moveit_demos ur10e_moveit_system.launch.py
```

Run in another terminal:

```bash
    ros2 launch ur10e_moveit_tutorials move_group_interface_ur10e.launch.py
```


## Unsorted useful commands


To get current State (ros2 topic echo /joint_states --once)

FOR MOVING EACH JOINT INDIVIDUALLY joint_names match positions order BEWARE OF CRUSHING:

ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
```bash
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
```
  }"

For activating the movement in the physical robot:

ros2 service call /dashboard_client/play std_srvs/srv/Trigger

For cleaning the docker log file:
sudo truncate -s 0 /var/log/syslog

Joint states:
ros2 topic echo /joint_states --once

source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash

TO BE DONE
## 

FIGURE OUT TIME PARAMETRIZATION AND IF IT WORKS WITH CARTESIAN MOVEMENTS

CALIBRATE THE UR FOR ROS (CHECK UR_CALIBRATION)