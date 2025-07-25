# Introduction
This repository serves as a testing ground for the functionalities ROSMaster X1 robot. 

High-level overview of the components in the ROS2 system as of July 25, 2025
![Current components/functionalities in the X1_ROS2_ws workspace](https://github.com/MinhNguyen2000/X1_ROS2_ws/blob/main/images/X1_ROS2_ws_system.png)

## Work in Progress
- [ ]  Physical camera integrated into the robot
- [ ]  Control the physical motors using the joysticks
- [ ]  SLAM using slam_toolbox on the physical robot

# Package Explanation

## x1_bringup
The x1_bringup package is used for working with the real robot. One example use case is to launch the robot_test.launch.py on the robot to publish LiDAR laser scan to the /scan topic, then launch the visualize.launch.py file on a computer in the same ROS network to visualize the LiDAR scan

**Launch files**
- robot_test.launch.py - to be launched on the robot to activate the sensors (LiDAR, camera, IMU, encoders,...) on the physical robot and publish their message on the appropriate topics
- visualize.launch.py - to be launched on a workstation in the same ROS network to remotely obtain and visualize the robot's onboard sensor data

## x1_control
This package is responsible for anything related to the robot teleoperation, such as controlling the simulated or real robot using joystick or keyboard. 

**Executables**
- x1_joy.py - defines a node that map from the joystick buttons/axes to velocity command (for physical or simulated robot) and other components on the physical robot (Buzzer and RGBLight)
- x1_keyboard.launch.py - used for controlling the velocity of the robot (physical or simulated) using a keyboard

**Launch files**
- x1_joy.launch.py - runs nodes to handle the communication between a joystick and the velocity command for a robot simulated in Gazebo
- x1_keyboard.launch.py

## x1_description
