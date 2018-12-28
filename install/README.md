# Installing

Based on the current configuration of your computer, you have three options to install the F1/10 package

### Basic options

The 'tools' folder contains scripts that help in installing the F1/10 package and all of its dependencies. You have to provide some parameters necessary to do this on your machine including

1. The name of your workspace, hereby referred to as <your_workspace_name>. This directory will be created in your machines '~/' path and will contain  all the nodes necessary to run the F1/10 package. The default value for this is 'catkin_ws'

2. The name of your preferred ROS distribution must also be supplied to the installer. This depends on the Ubuntu distribution of your machine and more information is available [here](http://wiki.ros.org/ROS/Installation).

### 1. Full Install

This option installs ROS along with the dependent packages necessary to run the F1/10 package locally with full simulator support. To do this, open a terminal and execute
```console
user@computer:$ bash ./tools/install_full.sh <your_workspace_name> <your_prefered_ROS_distribution>
```
