ROS2-Enabled Mobile Robot Control

This repository contains the code and instructions for setting up and controlling a mobile robot using ROS2 Humble Hawksbill on Ubuntu 22.04. The project is configured to work with a Raspberry Pi 4B and an L298N motor driver module.

Table of Contents
1.	Requirements
2.	Installation Guide
3.	Setting up the ROS2 Package
4.	Building and Running the Code
5.	Controlling the Robot
________________________________________

Requirements

•	Hardware:
1.	Raspberry Pi 4B
2.	L298N Motor Driver
3.	DC Motors with chassis for the robot
4.	Power source for Raspberry Pi 4B
5.	Separate power source for the motor driver
   
•	Software:

1.	ROS2 Humble Hawksbill on Ubuntu 22.04
2.	Colcon build tool
   
Installation Guide

Step 1: Install ROS2 Humble Hawksbill on Ubuntu 22.04

Update your system:
sudo apt update && sudo apt upgrade

Set up the sources:
sudo apt install -y software-properties-common
sudo add-apt-repository universe

Add the ROS2 key and repository:
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

Install ROS2 Humble:
sudo apt update
sudo apt install ros-humble-desktop

Set up ROS2 environment:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

Install additional ROS2 dependencies:
sudo apt install -y python3-colcon-common-extensions python3-rosdep

Initialize rosdep:
sudo rosdep init
rosdep update

Step 2: Install Package Dependencies
If your package relies on specific ROS2 libraries, install them. For instance:
sudo apt install ros-humble-geometry-msgs ros-humble-rclpy ros-humble-teleop-twist-keyboard
________________________________________

Setting up the ROS2 Package

Step 1: Create a ROS2 Workspace

Create a workspace to contain your ROS2 packages:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

Step 2: Create the motor_control Package

Navigate to your src directory and create the package:
cd ~/ros2_ws/src
ros2 pkg create motor_control --build-type ament_python --dependencies rclpy geometry_msgs

This package will contain your motor_control_node which processes cmd_vel messages to control motor speed and direction.

Step 3: Edit the Package Files

Edit setup.py to define the entry point for the motor_control_node.

Create the motor_control_node:

Inside src/motor_control/motor_control, create motor_control_node.py.
Add code to process the ROS2 cmd_vel topic to control your robot's motors.

Add Permissions:

chmod +x src/motor_control/motor_control/motor_control_node.py

Step 4: Update package.xml

Add dependencies in package.xml as needed for this project, like:
xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
________________________________________
Building and Running the Code

Step 1: Build the Workspace

From the workspace root, build the package with Colcon:
cd ~/ros2_ws
colcon build

Step 2: Source the Workspace

Once the build completes, source the workspace:
source install/setup.bash

Step 3: Run the motor_control_node

Launch the motor_control_node using ROS2:
ros2 run motor_control motor_control_node
________________________________________
Controlling the Robot

Step 1: Launch Teleoperation

To control your robot manually, use the teleop_twist_keyboard package:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Step 2: Sending Commands

Use the teleop_twist_keyboard terminal to control the robot’s movement. The motor_control_node will convert cmd_vel messages from this input into motor driver commands via GPIO.
________________________________________
