#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

set -x

function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0 [catkin workspace name (default:catkin_ws)] [ROS distro (default: indigo)"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}
# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
if [ $# != 2 ]; then
    usage
fi
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

name_catkinws=$1
name_catkinws=${name_catkinws:="catkin_ws"}
name_ros_distro=$2
name_ros_distro=${name_ros_distro:="indigo"}

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "saucy" | "trusty" | "vivid" | "wily" | "xenial")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Saucy(13.10) / Trusty(14.04) / Vivid / Wily / Xenial. Exit."
    exit 0
esac

echo "[Installing dependencies]"
sudo apt-get install -y ros-$name_ros_distro-gazebo-* ros-$name_ros_distro-control*

echo "[Installing navigation packages]"
sudo apt-get install -y ros-$name_ros_distro-teb-local-planner*

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/$name_ros_distro/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/$name_catkinws/src
cd ~/$name_catkinws/src
catkin_init_workspace
cd ~/$name_catkinws/
catkin_make

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
