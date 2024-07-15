# 2D Occupancy Grid Map

This repository contains the code and documentation for the project "Develop a 2D Occupancy Grid Map of a Room using Overhead Cameras". The project leverages ROS2 and TurtleBot3 to create detailed 2D maps for indoor navigation using image processing techniques.

## Description

The project aims to develop a 2D occupancy grid map of a room using overhead cameras mounted on the TurtleBot3 platform. It includes sensor integration, real-time image processing, and occupancy grid mapping using ROS2 libraries and tools.

## Features

- Sensor integration with aerial cameras for a top-down view of the room.
- Real-time image processing using OpenCV to extract features.
- ROS2-based algorithms to convert processed image data into 2D grid maps.
- Continuous updating and merging of occupancy grid maps.

## Installation

To set up the development environment, follow these steps:

1. Install ROS2 Foxy on your system.
2. Clone this repository and the required TurtleBot3 repositories.
3. Install necessary dependencies and build the workspace.

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
sudo apt install python3-argcomplete python3-colcon-common-extensions python3-vcstool
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations
cd ~/turtlebot3_ws
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
