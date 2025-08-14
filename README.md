
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

## 7\. Test the Installation

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
Here’s your updated `README.md` with the new **Step 8** added in the same style as the rest of the document:

---

````markdown
## 8. Orbbec Femto Bolt Setup (Optional but Recommended)

If you plan to use the **Orbbec Femto Bolt** camera, follow these steps to ensure it is detected and accessible.

### 8.1 Check USB Detection

First, confirm that Linux detects the device:

```bash
lsusb
````

Look for a line similar to:

```
ID 2bc5:xxxx Orbbec 3D Technology International, Inc Orbbec Femto Bolt 3D Camera
```

If it does not appear:

* Try a different USB port (preferably USB 3.x directly on the motherboard).
* Use a high-quality USB-C data cable (not just a charging cable).
* Avoid using USB hubs or adapters.

### 8.2 Fix Permissions

By default, you might need `sudo` to access the device. To avoid this, create a **udev rule**:

```bash
sudo nano /etc/udev/rules.d/99-orbbec.rules
```

Paste the following line:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666"
```

Then reload the rules and replug the camera:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the Femto Bolt.

### 8.3 Install the Orbbec SDK

If not already installed:

```bash
sudo apt update
sudo apt install libusb-1.0-0-dev libudev-dev cmake build-essential
git clone https://github.com/orbbec/OrbbecSDK.git
cd OrbbecSDK
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 8.4 Test with Orbbec Viewer

After building the SDK, run the viewer:

```bash
./bin/ObViewer
```

If you can see depth and RGB streams, the camera is working correctly.

```

---

Do you want me to also add a **quick Python test snippet** here so users can check the Femto Bolt output without going into ROS? That would make Step 8 immediately verifiable for them.
```
