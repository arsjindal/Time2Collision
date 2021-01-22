#!/usr/bin/sh

# source install_ros.sh

source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_siera/src

cp src/CMakeLists.txt ~/catkin_siera/src/CMakeLists.txt

cp -r src/siera ~/catkin_siera/src/

cd ~/catkin_siera/

catkin_make

source devel/setup.bash


