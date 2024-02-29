
## Launch LoCoBot with slam


Following this page:
https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros2_packages/navigation_stack_configuration.html

I use this command to launch the LoCoBot process on the NUC
```
ros2 launch interbotix_xslocobot_nav xslocobot_rtabmap.launch.py robot_model:=locobot_base use_lidar:=false slam_mode:=mapping rtabmap_args:=-d
```

Then use 
```
ros2 launch interbotix_xslocobot_descriptions remote_view.launch.py
```

To view rviz with everything on my laptop.

### TF problem 

By default, the create3 is configured with namespace `/mobile_base`.
Which all topics are published under this, including TF. 

When we connect to Create3, seems like the robot is publishing tf to 
```
/mobile_base/tf
/mobile_base/tf_static
```

The `xslocobot_rtabmap.launch.py` also launched `xslocobot_control.launch.py`. Which inside launched `interbotix_tf_tools/tf_rebroadcaster.launch.py`. Which seems to re-publish some tfs from one topic to another.

This node is suppose to solve the problem. However, if we look at the log, we can see they are appending the 

```
[component_container-5] [INFO] [1709147541.036418474] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_link', prepending prefix 'locobot/'.
[component_container-5] [INFO] [1709147541.036472602] [tf_rebroadcaster]: Will broadcast TF from frame 'odom' to frame 'base_footprint', prepending prefix 'locobot/'.
[component_container-5] [INFO] [1709147541.039615806] [tf_rebroadcaster]: Will broadcast TFs from topic '/locobot/mobile_base/tf' to the 'tf' topic under namespace '/'.
```

The problem from this log: tf_reboardcaster is looking or `'/<robot_name_launch_arg>/mobile_base/tf'` Which is not what create3 is publishing. So I need to change Create3's 

Looking closely at the TF tree. what's missing the the TF from `locobot/odom` to `locobot/base_footprint`

In interbotix's code, they are using `locobot/base_footprint` as root for many things (like rviz, rtabmap)

Then looking at `interbotix_xslocobot_control/config/tf_rebroadcaster.yaml`, which is passed down to re-broadcaster (there are another config in different package with different prefix setting). The content of it is saying re-broadcast the TF between `odom` -> `base_link` and `odom` -> `base_footprint`

The create3 side, nothing seems to be publishing base_footprint. So I'm gonna change everything in launch file to base_link

### Create3 time problem

The create3 base have problem getting its time synced from ntp servers (might also related to missing setting on NUC)

The tf_rebroadcast basically republish tf with the original timestamp on it. Since the create3 have bad system time, the tf message also have bad timestamp. 

The hack for now is to change the tf_rebroadcaster to put new timestamp on it (there will be delay, but it's the quickest fix for now)

## Controlling LoCoBot

Here are the topics that might be interesting

```
/cmd_vel
/locobot/commands/joint_group
/locobot/commands/joint_single
```

### pan till camera

According to the config file in `interbotix_ros_xslocobots/interbotix_xslocobot_control/config/locobot_base.yaml`

The pan tilt camera is ground into a `camera` group. From the experience with the interbotix arm, We should be able to publish to `/locobot/commands/joint_group` to control the pan tilt of the camera. 

```
/locobot/commands/joint_group
Type: interbotix_xs_msgs/msg/JointGroupCommand
```