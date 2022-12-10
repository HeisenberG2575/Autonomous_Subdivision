#!/bin/bash

source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
pip3 install sphinx

mkdir cartographer_ws
cd cartographer_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

sudo rosdep init
rosdep update
sed -i '/libabsl-dev/d' src/cartographer/package.xml
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
src/cartographer/scripts/install_abseil.sh

notify-send "This may take some time; might want to close other processes"
catkin_make_isolated --install --use-ninja
notify-send "Cartographer installation completed!"

