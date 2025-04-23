# Quadruped ROS2 Control

Welcome to the **Quadruped ROS2 Control** project! This open-source repository lets you simulate a **Unitree Go2 robot** using **ROS 2 Jazzy** on **Ubuntu 24.04**. You can control the robot in **Gazebo Harmonic**, **MuJoCo**, or **RViz** with a keyboard, making it perfect for beginners and ROS enthusiasts. Whether you’re new to robotics or an experienced developer, this guide will help you clone, set up, and test the project.

## Features
- **Simulations**: Run the Go2 robot in Gazebo Harmonic (3D physics), MuJoCo (3D dynamics), or RViz (visualization).
- **Keyboard Control**: Move the robot with simple key presses (forward, backward, turn, stop).
- **Components**:
  - **Controllers**: Robot control programs (`unitree_guide_controller`).
  - **Commands**: Tools for sending commands (`keyboard_input`).
  - **Descriptions**: Robot model files (`go2_description`).
  - **Hardwares**: Simulation interfaces (`hardware_unitree_mujoco`).
- **Community-Friendly**: Easy setup script and clear instructions for all users.

## Requirements
- **Operating System**: Ubuntu 24.04 (fresh install recommended).
- **Internet**: To download tools and project files.
- **Computer**: Standard Ubuntu-compatible computer (no special hardware needed).
- **Time**: About 10-20 minutes for setup, depending on your system.

## Getting Started
Follow these steps to clone the repository, set up the project, and run simulations. This guide is designed for beginners, with all commands explained.

### Step 1: Clone the Repository
1. **Open a Terminal**:
   Press `Ctrl+Alt+T` to open a terminal in Ubuntu.

2. **Clone the Project**:
   ```bash
   cd ~
   git clone https://github.com/legubiao/quadruped_ros2_control



   Step 2: Set Up the Project

We provide a simple script to install ROS 2 Jazzy, Gazebo Harmonic, MuJoCo, and all project files. The script also fixes common issues, like ensuring MuJoCo shows the robot and the robot stands upright.

    Save the Setup Script:
    Create a file for the script:
    bash

nano ~/setup_quadruped_ros2_go2.sh

Copy-paste the script below, save (Ctrl+O, Enter, Ctrl+X to exit):
bash
#!/bin/bash
# Quadruped ROS2 Control Setup Script
# Sets up the Quadruped ROS2 Control project on Ubuntu 24.04 with ROS 2 Jazzy.
# Installs ROS 2, Gazebo Harmonic, MuJoCo, and project files for Go2 robot simulations.
# Beginner-friendly and works for all users.
#
# Usage:
# 1. Save as setup_quadruped_ros2_go2.sh
# 2. Make executable: chmod +x setup_quadruped_ros2_go2.sh
# 3. Run: ./setup_quadruped_ros2_go2.sh

set -e

echo "Setting up Quadruped ROS2 Control for Unitree Go2 simulations..."

# Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install tools and simulators
echo "Installing Gazebo, MuJoCo, and ROS tools..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init || true
rosdep update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers gz-sim8
sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libosmesa6-dev

# Install MuJoCo
echo "Installing MuJoCo..."
cd ~
rm -rf mujoco-3.2.3-linux-x86_64.tar.gz ~/.mujoco
wget https://github.com/deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz
tar -xvzf mujoco-3.2.3-linux-x86_64.tar.gz
mv mujoco-3.2.3 ~/.mujoco
echo "export MUJOCO_PY_MUJOCO_PATH=~/.mujoco" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/lib" >> ~/.bashrc
source ~/.bashrc

# Use FastDDS to avoid conflicts
sudo apt install -y ros-jazzy-rmw-fastrtps-cpp
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc

# Create workspace
echo "Creating workspace..."
cd ~
rm -rf quadruped_ros2_go2
mkdir -p quadruped_ros2_go2/src
cd quadruped_ros2_go2/src

# Clone repositories
git clone https://github.com/legubiao/quadruped_ros2_control
cd quadruped_ros2_control
git submodule update --init --recursive
cd ..
git clone https://github.com/unitreerobotics/unitree_mujoco
git clone https://github.com/unitreerobotics/unitree_sdk2

# Install dependencies
echo "Installing project dependencies..."
cd ~/quadruped_ros2_go2
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:noble

# Build project
echo "Building project..."
colcon build --packages-up-to gz_quadruped_playground unitree_guide_controller go2_description keyboard_input unitree_mujoco hardware_unitree_mujoco --symlink-install
source install/setup.bash

# Fix MuJoCo launch
echo "Fixing MuJoCo to show simulator..."
MUJOCO_LAUNCH=~/quadruped_ros2_go2/src/quadruped_ros2_control/controllers/unitree_guide_controller/launch/mujoco.launch.py
MODEL_FILE=$(find ~/quadruped_ros2_go2/src/unitree_mujoco -name "*.xml" | head -n 1 || echo "")
if [ -z "$MODEL_FILE" ]; then
    echo "WARNING: No MuJoCo model file found."
else
    cat > $MUJOCO_LAUNCH << EOL
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_mujoco',
            executable='mujoco_sim',
            name='mujoco_sim',
            output='screen',
            parameters=[{'model_file': '$MODEL_FILE'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
EOL
fi

# Rebuild
cd ~/quadruped_ros2_go2
colcon build --packages-up-to unitree_guide_controller unitree_mujoco --symlink-install
source install/setup.bash

# Instructions
echo "Setup complete! Check README.md for how to run simulations."

Make the Script Executable:
bash
chmod +x ~/setup_quadruped_ros2_go2.sh

Run the Script:
bash

    ./setup_quadruped_ros2_go2.sh

    This will:
        Install ROS 2 Jazzy, Gazebo Harmonic, MuJoCo, and tools.
        Set up the project workspace.
        Fix MuJoCo to show the simulator.
        Prepare the robot to stand upright.

Step 3: Run Simulations

After running the script, use these commands to see the robot in action. Open a new terminal for each command (press Ctrl+Alt+T to open a new terminal).

    Gazebo Harmonic (3D Simulation):
    bash

source ~/quadruped_ros2_go2/install/setup.bash
ros2 launch unitree_guide_controller gazebo.launch.py
This opens a 3D world with the Go2 robot moving in a realistic environment.
MuJoCo (Another 3D Simulation):
bash
source ~/quadruped_ros2_go2/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py
This opens the MuJoCo simulator with the Go2 robot.
RViz (Robot Visualization):
bash
source ~/quadruped_ros2_go2/install/setup.bash
rviz2
In RViz:

    Set Fixed Frame to base_link.
    Add RobotModel (select /robot_description).
    Add TF. This shows a visual model of the robot.

Control the Robot: In a new terminal:
bash
source ~/quadruped_ros2_go2/install/setup.bash
ros2 run keyboard_input keyboard_input
Use these keys to move the robot:

    w: Move forward
    s: Move backward
    a: Turn left
    d: Turn right
    q: Stop

Fix Robot if Lying Down: If the robot appears collapsed (lying on the ground), run:
bash
source ~/quadruped_ros2_go2/install/setup.bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header: {frame_id: ''}, name: ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'], position: [0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2], velocity: [], effort: []" --once
This makes the robot stand upright.
Check Joint Names (if stand command fails):
bash

    source ~/quadruped_ros2_go2/install/setup.bash
    ros2 topic echo /robot_description
    Look for joint names (e.g., FR_hip_joint, FR_thigh_joint) in the output. If they differ, update the stand command with the correct names.

Troubleshooting

If something doesn’t work, try these simple fixes:

    Gazebo Doesn’t Open:
    bash

gz sim -r empty.sdf
If it fails, your computer may need a graphics update. Search online for “Ubuntu 24.04 Gazebo not working” or ask in our Issues page.
MuJoCo Shows Only RViz:
bash
source ~/quadruped_ros2_go2/install/setup.bash
ros2 run unitree_mujoco mujoco_sim
If it doesn’t work, check the unitree_mujoco repository for setup tips.
Robot Doesn’t Stand: Run the stand command again (see Step 3.5). If it fails, check joint names (Step 3.6).
Robot Doesn’t Move:
bash

    source ~/quadruped_ros2_go2/install/setup.bash
    ros2 topic echo /cmd_vel
    If no output appears, ensure the keyboard control command is running (Step 3.4).
    Other Errors: Copy the error message from the terminal and:
        Search it online (e.g., Google).
        Check our Issues page.
        Ask the community by opening a new issue.

Note: You may see warnings about get_value() in unitree_guide_controller. These are safe to ignore.
What’s Next

Congratulations on running the Go2 robot simulation! Here are some fun things to try next:

    Explore Controllers:
        OCS2 Quadruped Controller: Smart control for advanced movements (in gz_quadruped_playground).
        RL Quadruped Controller: Uses machine learning to control the robot.
    Add Sensors:
        Simulate cameras or LIDAR in Gazebo for richer environments (see gz_quadruped_playground).
    Deploy on a Real Robot:
        Use this project with a real Unitree Go2 (check go2_description for setup).
    Contribute:
        Share your improvements or report issues on GitHub.
        Help translate this guide or add new features!

Project Status

    Completed (as of 2025):
        Gazebo Playground with OCS2 controller.
        Refactored FSM and Unitree Guide Controller.
        Real Go2 robot support.
    Planned:
        OCS2 Perceptive locomotion demo (stay tuned!).

Demo

Watch the Unitree Go2 robot in action on a real robot:

Video on Bilibili
References

    Main Repository: github.com/legubiao/quadruped_ros2_control
    MuJoCo Support: github.com/unitreerobotics/unitree_mujoco
    Unitree Guide: github.com/unitreerobotics/unitree_guide
    Paper: Liao et al., “Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots,” IROS 2023.
    Related Projects:
        github.com/qiayuanl/legged_control
        github.com/fan-ziqi/rl_sar

Contributing

We love community contributions! To help out:

    Fork this repository.
    Make changes (e.g., add features, fix bugs, improve this README).
    Submit a pull request.
    Report issues or suggest ideas on our Issues page.

License

This project is open-source. Check the  file for details.

Have fun controlling your robot! If you’re new to ROS, don’t worry—just follow the steps, and you’ll see the Go2 robot moving in no time. Join our community on GitHub to share your success or ask questions!
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
