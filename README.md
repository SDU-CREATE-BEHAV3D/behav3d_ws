
-----

# Behav3D ROS 2 Workspace Installation
This guide provides instructions for setting up the Behav3D ROS 2 workspace on Ubuntu 24.04 with ROS 2 Jazzy.
For installing Ros and more details read the file [manual_install.md](manual_install.md)

-----

## 1\. Prerequisites

First, ensure you have **ROS 2 Jazzy** installed on **Ubuntu 24.04**.

Next, open a terminal and run the following command to install the essential tools and packages required for building the workspace:

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-setuptools \
  wget
```

-----

## 2\. Getting Started

### Clone the Repository

Create a workspace directory and clone the repository, including all its submodules.

```bash
mkdir -p ~/behav3d_ws
cd ~/behav3d_ws
git clone https://github.com/SDU-CREATE-BEHAV3D/behav3d_ws.git .
```

### Initialize `rosdep`

If you haven't used `rosdep` before, you'll need to initialize it.

```bash
sudo rosdep init
```

-----

## 3\. Middleware Configuration (CycloneDDS)

MoveIt developers recommend using CycloneDDS for best performance.

### Install CycloneDDS

Run the following command to install the CycloneDDS middleware and set it as the default RMW implementation in your `.bashrc` file.

```bash
sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && source ~/.bashrc
```

> **Note:** This only works by doing the firewall trick.

### Verify RMW Implementation

Check that CycloneDDS is correctly set as the RMW implementation.

```bash
echo $RMW_IMPLEMENTATION
```

You should see the output: `rmw_cyclonedds_cpp`.

> **Note:** This step was necessary in some devices and not in others.

### Firewall Configuration for CycloneDDS

For CycloneDDS to work correctly, you need to allow its traffic through the firewall.

```bash
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
sudo ufw reload
```

-----

## 4\. Install External Repositories

Now, you'll import the external package dependencies into your workspace.

### Navigate and Import

Go to the `src/external` directory within your workspace and use `vcs` to import the repositories listed in `ros2.repos`.

```bash
cd ~/behav3d_ws/src
mkdir external
cd external
vcs import < ../../ros2.repos
```

> ***note:Orbecc sdk  install to be reviewed it might fail in new devices.**

-----

## 5\. Build the Workspace

### Install Dependencies

Update your package lists and use `rosdep` to install all the necessary dependencies for the packages in your workspace.

```bash
sudo apt update
sudo apt dist-upgrade
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

### Build with `colcon`

Compile the entire workspace using `colcon`.

```bash
cd ~/behav3d_ws
colcon build --executor sequential
```

> **Note:** Using `--executor sequential` is recommended for systems with less than 16GB of RAM.

-----

## 6\. Source the Workspace

To use your newly built workspace, you need to source its setup files.

### Add to `.bashrc`

Open your `.bashrc` file to add the sourcing commands.

```bash
nano ~/.bashrc
```

Add the following lines to the end of the file. This will automatically source your ROS 2 installation and your local workspace every time you open a new terminal.

```bash
source /opt/ros/jazzy/setup.bash
source ~/behav3d_ws/install/setup.bash
```

Save the file and exit the editor. Then, source the `.bashrc` file to apply the changes to your current terminal session.

```bash
source ~/.bashrc
```

-----

## 8\. Test the Installation

You can test your setup by launching the kinematics demo.

### Launch the Demo

Run the following launch command:

```bash
ros2 launch kinematics_demo_cpp kinematics_demo_launch.launch.py
```

### Publish a Test Topic

In a new terminal, publish a message to the `/user_input` topic to interact with the demo:

```bash
ros2 topic pub --once /user_input std_msgs/msg/String "{data: 'draw_line'}"
```

If the demo runs and responds to the command, your installation is complete and working correctly. 🎉
-----
## 7\. How to connect your Orbbec Femto camera for the first time on a Linux system. 

### Step 1: Check USB Detection

First, let's confirm that your computer recognizes the camera. Plug the device in and run the following command in your terminal:

```bash
lsusb
```

You should see a line in the output that identifies the camera. Look for the **vendor ID `2bc5`**:

`Bus 002 Device 006: ID 2bc5:066b Orbbec 3D Technology International, Inc Orbbec Femto Bolt 3D Camera`

If you don't see this entry, try using a different USB port (preferably a USB 3.0 port) or a different data cable.

-----

### Step 2: Fix Device Permissions

To avoid needing `sudo` every time you access the camera, you'll create a **`udev` rule**. This automatically grants the correct permissions to the device when it's plugged in.

1.  Create and open a new rules file using a text editor like `nano`:

    ```bash
    sudo nano /etc/udev/rules.d/99-orbbec.rules
    ```

2.  Paste the following line into the file. This rule uses the **vendor ID `2bc5`** that you confirmed in the previous step.

    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666"
    ```

3.  Save the file and exit the editor. (In `nano`, press `Ctrl+X`, then `Y`, then `Enter`).

4.  Finally, tell the system to reload its rules so your new rule takes effect:

    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

After completing these steps, unplug and replug your camera. It should now be fully accessible and ready to use\! ✅
