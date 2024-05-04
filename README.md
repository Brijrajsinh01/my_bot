# My_Bot

## Overview

My_Bot is your ticket to mapping the unknown with style. Powered by ROS 2 and the magic of SLAM (Simultaneous Localization and Mapping), this project propels differential drive robots into uncharted territories while seamlessly crafting detailed maps on the fly.

Go ahead, clone it and have fun. 
Raise some cool good issues(even if it's silly) and we will sove it together.

## Packages

Packages are self explanatory!!!
## Installation

To use the My_Bot project, you'll need to have ROS 2 installed on your system. Follow the [official ROS 2 installation instructions](https://docs.ros.org/en/foxy/Installation.html) to set up ROS 2 on your machine.

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

Yooyoo your bot is moving and mapping now
