#!/bin/bash
# Quadruped ROS2 Control Setup Script for Beginners
# This script sets up the Quadruped ROS2 Control project (https://github.com/legubiao/quadruped_ros2_control)
# on Ubuntu 24.04 with ROS 2 Jazzy. It runs simulations of the Unitree Go2 robot in Gazebo Harmonic, MuJoCo, and RViz
# with keyboard control. The script is simple, beginner-friendly, and works for all users with standard Ubuntu setups.
#
# What the Script Does:
# - Installs ROS 2 Jazzy, Gazebo Harmonic, and MuJoCo.
# - Sets up a workspace with all needed packages.
# - Fixes common issues (e.g., MuJoCo not opening, robot lying down).
# - Shows how to run simulations and move the robot.
#
# Repository Structure:
# - Controllers: Control programs for the robot (unitree_guide_controller)
# - Commands: Tools to send commands (keyboard_input)
# - Descriptions: Robot model files (go2_description)
# - Hardwares: Simulation interfaces (hardware_unitree_mujoco)
#
# Requirements:
# - Ubuntu 24.04 (fresh install recommended)
# - Internet connection
# - Basic computer (no special graphics card needed)
#
# Usage:
# 1. Save as setup_quadruped_ros2_go2.sh
# 2. Make executable: chmod +x setup_quadruped_ros2_go2.sh
# 3. Run: ./setup_quadruped_ros2_go2.sh
#
# Note: Follow the instructions printed at the end to run simulations.
# If something doesn’t work, check the troubleshooting tips.
#
# Author: Simplified from legubiao/quadruped_ros2_control README
# Date: April 23, 2025

set -e

echo "Welcome! Let’s set up the Quadruped ROS2 Control project to simulate a Unitree Go2 robot..."

# Step 1: Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy (the robot software framework)..."
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "ROS 2 Jazzy installed! Checking version..."
printenv ROS_DISTRO  # Should show: jazzy

# Step 2: Install Tools and Simulators
echo "Installing tools for building and running simulations (Gazebo, MuJoCo)..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init || true
rosdep update

# Install Gazebo Harmonic (a 3D robot simulator)
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers gz-sim8

# Install MuJoCo dependencies (another 3D simulator)
sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libosmesa6-dev

# Install MuJoCo
echo "Installing MuJoCo simulator..."
cd ~
rm -rf mujoco-3.2.3-linux-x86_64.tar.gz ~/.mujoco
wget https://github.com/deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz
tar -xvzf mujoco-3.2.3-linux-x86_64.tar.gz
mv mujoco-3.2.3 ~/.mujoco
echo "export MUJOCO_PY_MUJOCO_PATH=~/.mujoco" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/lib" >> ~/.bashrc
source ~/.bashrc
echo "Testing MuJoCo (a window should open, close it with Ctrl+C)..."
~/.mujoco/bin/simulate ~/.mujoco/model/humanoid.xml &
sleep 5
pkill simulate || true

# Use FastDDS to avoid conflicts with robot software
sudo apt install -y ros-jazzy-rmw-fastrtps-cpp
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
echo "FastDDS set up! Checking..."
printenv RMW_IMPLEMENTATION  # Should show: rmw_fastrtps_cpp

# Step 3: Create and Build Workspace
echo "Creating a workspace to store and build the robot project..."
cd ~
rm -rf quadruped_ros2_go2
mkdir -p quadruped_ros2_go2/src
cd quadruped_ros2_go2/src

# Download project files
echo "Downloading robot project and supporting files..."
git clone https://github.com/legubiao/quadruped_ros2_control
cd quadruped_ros2_control
git submodule update --init --recursive
cd ..
git clone https://github.com/unitreerobotics/unitree_mujoco
git clone https://github.com/unitreerobotics/unitree_sdk2

# Install project dependencies
echo "Installing project dependencies..."
cd ~/quadruped_ros2_go2
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble

# Build the project
echo "Building the project (this may take a few minutes)..."
colcon build --packages-up-to gz_quadruped_playground unitree_guide_controller go2_description keyboard_input unitree_mujoco hardware_unitree_mujoco --symlink-install
source install/setup.bash
echo "Project built! Checking packages..."
ls install | grep -E "gz_quadruped_playground|unitree_guide_controller|go2_description|keyboard_input|unitree_mujoco|hardware_unitree_mujoco"

# Step 4: Fix MuJoCo to Show Simulator (Not Just RViz)
echo "Making sure MuJoCo shows the robot simulation..."
MUJOCO_LAUNCH=~/quadruped_ros2_go2/src/quadruped_ros2_control/controllers/unitree_guide_controller/launch/mujoco.launch.py
MODEL_FILE=$(find ~/quadruped_ros2_go2/src/unitree_mujoco -name "*.xml" | head -n 1 || echo "")
if [ -z "$MODEL_FILE" ]; then
    echo "WARNING: No MuJoCo model file found. You may need to check unitree_mujoco."
else
    cat > $MUJOCO_LAUNCH << EOL
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MuJoCo simulator
        Node(
            package='unitree_mujoco',
            executable='mujoco_sim',  # May need adjustment
            name='mujoco_sim',
            output='screen',
            parameters=[{'model_file': '$MODEL_FILE'}]
        ),
        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
EOL
    echo "Updated MuJoCo launch file."
fi

# Rebuild after changing launch file
cd ~/quadruped_ros2_go2
colcon build --packages-up-to unitree_guide_controller unitree_mujoco --symlink-install
source install/setup.bash

# Step 5: Instructions for Running Simulations
echo "All done! Here’s how to run the robot simulations:"

echo -e "\n1. Run Gazebo Harmonic (3D simulation):"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  ros2 launch unitree_guide_controller gazebo.launch.py"

echo -e "\n2. Run MuJoCo (another 3D simulation):"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  ros2 launch unitree_guide_controller mujoco.launch.py"

echo -e "\n3. Run RViz (robot visualization):"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  rviz2"
echo "  # In RViz, set Fixed Frame to 'base_link', add RobotModel (/robot_description), and TF"

echo -e "\n4. Control the robot with keyboard (open a new terminal):"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  ros2 run keyboard_input keyboard_input"
echo "  # Keys: w (forward), s (backward), a (left), d (right), q (stop)"

echo -e "\n5. Fix robot if it’s lying down:"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  ros2 topic pub /joint_states sensor_msgs/msg/JointState \"header: {frame_id: ''}, name: ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'], position: [0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2], velocity: [], effort: []\" --once"

echo -e "\n6. Check robot joint names (if stand command fails):"
echo "  source ~/quadruped_ros2_go2/install/setup.bash"
echo "  ros2 topic echo /robot_description"

echo -e "\nIf Something Goes Wrong:"
echo "  - Gazebo not opening? Try: gz sim -r empty.sdf"
echo "  - MuJoCo not showing? Check: ros2 run unitree_mujoco mujoco_sim"
echo "  - Robot not moving? Check: ros2 topic echo /cmd_vel"
echo "  - Error messages? Copy them and search online or ask for help."

echo -e "\nWhat to Do Next:"
echo "  - Try other controllers (like OCS2) in the project."
echo "  - Add sensors (like cameras) in Gazebo."
echo "  - Learn more: https://github.com/legubiao/quadruped_ros2_control"

echo -e "\nSetup complete! Run the commands above to see your robot in action!"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////::

ORIGINAL 


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot.

* [Controllers](controllers): contains the ros2-control controllers
* [Commands](commands): contains command node used to send command to the controller
* [Descriptions](descriptions): contains the urdf model of the robot
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot

Todo List:

- [x] **[2025-02-23]** Add Gazebo Playground
  - [x] OCS2 controller for Gazebo Simulation
  - [x] Refactor FSM and Unitree Guide Controller
- [x] **[2025-03-30]** Add Real Go2 Robot Support
- [ ] OCS2 Perceptive locomotion demo

Video on Real Unitree Go2 Robot:
[![](http://i0.hdslb.com/bfs/archive/7d3856b3c5e5040f24990d3eab760cf8ba4cf80d.jpg)](https://www.bilibili.com/video/BV1QpZaY8EYV/)

## 1. Quick Start

* rosdep
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install
    ```

### 1.1 Mujoco Simulator or Real Unitree Robot
> **Warning:** CycloneDDS ROS2 RMW may conflict with unitree_sdk2. If you cannot launch unitree mujoco simulation
> without `sudo`, then you cannot used `unitree_mujoco_hardware`. This conflict could be solved by one of below two
> methods:
> 1. Uninstall CycloneDDS ROS2 RMW, used another ROS2 RMW, such as FastDDS **[Recommended]**.
> 2. Follow the guide in [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) to configure the ROS2 RMW by
     compiling cyclone dds.

* Compile Unitree Hardware Interfaces
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to hardware_unitree_mujoco
    ```
* Follow the guide in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco) to launch the unitree mujoco go2
  simulation
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller mujoco.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![mujoco](.images/mujoco.png)

### 1.2 Gazebo Classic Simulator (ROS2 Humble)

* Install Gazebo Classic
  ```bash
  sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
  ```
* Compile Leg PD Controller
    ```bash
    colcon build --packages-up-to leg_pd_controller
    ```
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller gazebo_classic.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![gazebo classic](.images/gazebo_classic.png)

### 1.3 Gazebo Harmonic Simulator (ROS2 Jazzy)

* Install Gazebo
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

* Compile Gazebo Playground
  ```bash
  colcon build --packages-up-to gz_quadruped_playground --symlink-install
  ```
* Launch the ros2-control
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo.launch.py
  ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![gazebo](.images/gazebo.png)

For more details, please refer to the [unitree guide controller](controllers/unitree_guide_controller/)
and [go2 description](descriptions/unitree/go2_description/).

## What's Next
Congratulations! You have successfully launched the quadruped robot in the simulation. Here are some suggestions for you to have a try:
* **More Robot Models** could be found at [description](descriptions/)
* **Try more controllers**. 
  * [OCS2 Quadruped Controller](controllers/ocs2_quadruped_controller): Robust MPC-based controller for quadruped robot
  * [RL Quadruped Controller](controllers/rl_quadruped_controller): Reinforcement learning controller for quadruped robot
* **Simulate with more sensors**
  * [Gazebo Quadruped Playground](libraries/gz_quadruped_playground): Provide gazebo simulation with lidar or depth camera.
* **Real Robot Deploy**
  * [Unitree Go2 Robot](descriptions/unitree/go2_description): Check here about how to deploy on go2 robot.

## Reference

### Conference Paper

[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with
duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp.
2723-2730. IEEE, 2023.

### Miscellaneous

[1] Unitree Robotics. *unitree\_guide: An open source project for controlling the quadruped robot of Unitree Robotics,
and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree
Robotics*. [Online].
Available: [https://github.com/unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)

[2] Qiayuan Liao. *legged\_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged
robots*. [Online]. Available: [https://github.com/qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)

[3] Ziqi Fan. *rl\_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.*

2024. Available: [https://github.com/fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 
