#! /usr/bin/zsh

# Remeber to source this file. 
# I wrote this file to be zsh for my convenience, just change 
# all zsh to bash

here="$(dirname $(realpath $0))"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE="${here}/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml"
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3

source /opt/ros/humble/setup.zsh
source "${here}/install/setup.zsh"


