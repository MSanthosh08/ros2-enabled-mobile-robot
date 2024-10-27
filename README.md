# ROS2-Enabled Mobile Robot Control

This repository contains the code and instructions for setting up and controlling a mobile robot using ROS2 Humble Hawksbill on Ubuntu 22.04. The project is configured to work with a Raspberry Pi 4B and an L298N motor driver module.


## Table of Contents

1.[Requirements]()

2.[Installation Guide]()

3.[Setting up the ROS2 Package]()

4.[Building and Running the Code]()

5.[Controlling the Robot]()



## Requirements
### Hardware:
1.Raspberry Pi 4B

2.L298N Motor Driver

3.DC Motors with chassis for the robot

4.Power source for Raspberry Pi 4B

5.Separate power source for the motor driver

### Software:

1.ROS2 Humble Hawksbill on Ubuntu 22.04

2.Colcon build tool



## Installation Guide
Step 1: Install ROS2 Humble Hawksbill on Ubuntu 22.04

Update your system:

```bash
sudo apt update && sudo apt upgrade
```
Set up the sources:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe

```
Add the ROS2 key and repository:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

```
Install ROS2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-desktop

```
Set up ROS2 environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
Install additional ROS2 dependencies:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```
Initialize rosdep:

```bash
sudo rosdep init
rosdep update

```
Step 2: Install Package Dependencies

If your package relies on specific ROS2 libraries, install them. For instance:


```bash
sudo apt install ros-humble-geometry-msgs ros-humble-rclpy ros-humble-teleop-twist-keyboard
```
Setting up the ROS2 Package

Step 1: Create a ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```
Step 2: Create the motor_control Package
Navigate to your src directory and create the package:


```bash
cd ~/ros2_ws/src
ros2 pkg create motor_control --build-type ament_python --dependencies rclpy geometry_msgs

```
This package will contain your motor_control_node which processes cmd_vel messages to control motor speed and direction.

Step 3: Edit the Package Files

Edit setup.py to define the entry point for the motor_control_node.

Create the motor_control_node:
Inside src/motor_control/motor_control, create motor_control_node.py.

Add Permissions

```bash
chmod +x src/motor_control/motor_control/motor_control_node.py

```
Step 4: Update package.xml

```bash
<depend>rclpy</depend>
<depend>geometry_msgs</depend>

```
Building and Running the Code

Step 1: Build the Workspace


```bash
cd ~/ros2_ws
colcon build

```
Step 2: Source the Workspace

```bash
source install/setup.bash
```
Step 3: Run the motor_control_node

```bash
ros2 run motor_control motor_control_node
```
Controlling the Robot

Launch Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To avoid sourcing your workspace every time manually, you can add the source command to your shell's profile file. Here’s how to do it:

 1.Add Source Command to .bashrc
 ```bash
sudo nano ~/.bashrc
```
2.Add the following line at the end of the file:
 ```bash
source ~/ros2_ws/install/setup.bash
```
(Make sure to replace ~/ros2_ws with the actual path to your ROS 2 workspace if it's different.)

3.Save and close the file by pressing Ctrl + X, then Y, and Enter.

4.Apply the changes immediately without restarting the terminal:
 ```bash
source ~/.bashrc

```