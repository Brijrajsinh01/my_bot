# My_Bot

## Overview

My_Bot is a collection of ROS 2 packages designed for [describe the purpose of your project here, e.g., building a mobile robot platform for navigation and perception tasks]. It provides [briefly describe the main functionalities or features of your project].

## Packages

The My_Bot project consists of the following packages:

1. **my_bot_description**: This package contains [describe what this package contains, e.g., URDF and meshes for the robot model].

2. **my_bot_navigation**: This package provides [describe what this package provides, e.g., navigation launch files and configuration for autonomous navigation].

3. **my_bot_control**: This package includes [describe what this package includes, e.g., ROS 2 controllers for controlling the robot's actuators].

## Installation

To use the My_Bot project, you'll need to have ROS 2 installed on your system. Follow the [official ROS 2 installation instructions](https://index.ros.org/doc/ros2/Installation/) to set up ROS 2 on your machine.

Once you have ROS 2 installed, you can clone the My_Bot repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Brijrajsinh01/my_bot.git
```
### Launching Simulation

To launch the simulation environment, run the following command:

```bash
ros2 launch my_bot launch_sim.launch.py
```
### Moving Bot
After launching the simulation, you can control the robot using the ros2 teleop twist keyboard package. Use the following command to control the robot's movement:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
