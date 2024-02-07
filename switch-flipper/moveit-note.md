

## Tutrials 

To lean srdf
https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html

## Moveit example 

Official example: 
```
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=px100 hardware_type:=actual
```

Which is located :
```
├── interbotix_ros_manipulators
│   ├── interbotix_ros_xsarms
│   │   ├── interbotix_xsarm_moveit
```

This is the path to the wx200 srdf 
`interbotics-ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit/config/srdf/wx200.srdf.xacro`


## Defining custom planning group

This is done by adding a group tag with all the joints 

```
    <group name="lollypop_hammer">
        <joint name="waist" />
        <joint name="shoulder" />
        <joint name="elbow" />
        <joint name="wrist_angle" />
        <joint name="wrist_rotate" />
        <joint name="ee_arm" />
        <joint name="gripper_bar" />
        <joint name="ee_bar" />
        <joint name="lollypop_joint" />
        <joint name="lollypop_ee_offset" />
    </group>
```

This also included all the fixed joints along the chain. (To be tested if I need them). And according to how interboticx does it, all of the fixed joint is also given an axis even not used.

### Add interactive plan marker to custom group 

This is actually nothing to do with the EE. 

This is defined in this file 

`interbotics-ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit/config/kinematics.yaml`

Which by default have:
```
/**:
  ros__parameters:
    robot_description_kinematics:
      interbotix_arm:
        kinematics_solver: lma_kinematics_plugin/LMAKinematicsPlugin
        kinematics_solver_search_resolution: 0.005
        kinematics_solver_timeout: 0.005
        position_only_ik: false
```
So to add one for my custom group, I just add a same section for my `lollypop_hammer` group. 


## interboticx Moveit interface 

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/moveit_interface_and_api.html


```
interbotics-ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_moveit_interface/launch/xsarm_moveit_interface.launch.py
```

This interface thing is also using 

`interbotix_xsarm_moveit/launch/xsarm_moveit.launch.py` inside it.

This launch file also points to `interbotix_common_toolbox/interbotix_moveit_interface` package for a python script.

interbotics-ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/scripts/moveit_python_interface


## NAN joint state problem 

When using `interbotix_xsarm_moveit/launch/xsarm_moveit.launch.py` with sim robot (xs_sdk_sim.py doing the sim) I would get nan for grippers.

```
---
header:
  stamp:
    sec: 1707287552
    nanosec: 909415842
  frame_id: ''
name:
- waist
- shoulder
- elbow
- wrist_angle
- wrist_rotate
- gripper
- left_finger
- right_finger
position:
- 0.0
- -1.8799999952316284
- 1.5
- 0.800000011920929
- 0.0
- .nan
- .nan
- .nan
velocity: []
effort: []

```

This error was printed at the beginning. However it also show up for description only launch file and JS published are fined.
```
[xs_sdk_sim.py-6] Unknown attribute "name" in /robot[@name='wx200']/link[@name='lollypop_hammer']/visual[1]
[xs_sdk_sim.py-6] Unknown attribute "name" in /robot[@name='wx200']/link[@name='camera_stand']/visual[1]
[xs_sdk_sim.py-6] Unknown attribute "name" in /robot[@name='wx200']/link[@name='april_tag_mount']/visual[1]
[xs_sdk_sim.py-6] Unknown tag "ros2_control" in /robot[@name='wx200']
[xs_sdk_sim.py-6] [INFO] [1707287550.698244714] [interbotix_xs_sdk.xs_sdk_sim]: Loaded motor configs from `/home/leogray/OneDrive/Active-file/NW-classes/winter-proj/interbotics-ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml`.
```

This turns out to be a problem with parameters. The use_sim param should not be used on the moveit level. The alternative argument is hardware_type:= fake


