#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

set -x

name_catkinws=$1
name_catkinws=${name_catkinws:="catkin_ws"}

echo "[Setting the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_distro/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkinws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

echo "[Proceeding to install source]"
cp -r ~/f1tenth_gtc_tutorial/data/ ~/$name_catkinws/src/
cd ~/$name_catkinws/
catkin_make install

exec bash

exit 0
