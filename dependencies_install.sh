#!/bin/bash
sudo apt update;
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control

#   Install ira_laser_tools for lidar fusion
cd ..
git clone -b humble git@github.com:nakai-omer/ira_laser_tools.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash