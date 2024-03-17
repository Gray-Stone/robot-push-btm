#! /usr/bin/zsh 

source /opt/ros/humble/setup.zsh

set -vx
ros2 topic pub -r 0.8 /locobot/commands/joint_group interbotix_xs_msgs/msg/JointGroupCommand "{name: 'camera', cmd: [0.0, 0.3]}"
set +vx
