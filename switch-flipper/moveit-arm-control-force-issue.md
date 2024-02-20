
# Motor POWERRRRRRRR!

The motor used in wx200, except grippers, are xm430-w350

## Moveit plan and execute 
After getting moveit motion planning working, and using moveit to let wx200 execute the motion, there is the problem of motion being too weak.

basically the code execution is like the following.
```
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    auto const plan_success =
        static_cast<bool>(move_group_interface.plan(plan_msg));

    if (plan_success) {

      move_group_interface.execute(plan_msg);
    }

```

The robot is moving through a timed trajectory, velocity controlled, which is expected. 

But when the controller think robot has reached the destination, it's really far away from the goal, like 20mm in negative z, dropped due to gravity. 

from this image:

[](medias/moveit-to-click/moveit-exec.png)

We can see the robot end up at a position not only offset in Z, but also x y. 


## Directly talking to interbotix

If I strip out the last waypoint and directly publish that to `wx200/commands/joint_group`, the robot will just zap into the goal right away!. This is not the ideal way of moving as it dis-regard the motion planning. 

Thus we take a reduced way points from the plan, and send it to robot with correct interval. The resulting motion is still a lot sharper then the moveit command. However, it still have offset in Z when it reached destination.

This offset in Z apppear in all ways of commanding. Thus we might conclude that its more of a gain problem from the robot itself.

This image: 
[](medias/moveit-to-click/Direct-JointGroupCommand.png)

Showed a direct command to interbotix topic result in "stronger" movement, at least the XY are lined up, just having the Z offset. 

## Reduce waypoint density.
One assumption for the bad movit behavior is from the dense waypoints, where the motion plan is dense with lots of via points in it.

This means the robot's velocity is more controlled, Which robot carries a lot less momentum when approching target.

So I have reduced the planned points by a lot, but the result still doesn't improve 


## Robot controller.

If I try to dig out the actual controller interbotix used. I first found the `wx200_controllers.yaml` file which is next to the srdf. 

This file indicate its using `FollowJointTrajectory` type of controller. 

next to it, there is a `modes.yaml` config file which list 
```
groups:
  arm:
    operating_mode: position
    profile_type: velocity
    profile_velocity: 131
    profile_acceleration: 25
    torque_enable: true
```

I digged around for which node used this, I found a chain of launch files 
`xsarm_ros_control.launch.py` -> `xsarm_control.launch.py` Which eventually lead to `interbotix_ros_xseries/interbotix_sx_sdk/xs_sdk_obj.cpp`. 

This node subscribes to `commands/joint_trajectory`

Internally, the node save the received trajectory, and give it to a timer. The timer calls the `InterbotixDriverXS::write_commands` which depending on the mode given, convert commands to the right type and send it to the dynamixel


### Setting gain 

The first reaction for not moving hard enough would be to increase gain 

There are gain setting available somewhere.

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/overview/xs_msgs.html#interbotix-xs-msgs-operatingmodes-ros2


And there is a service for it 
https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/overview/xs_sdk.html#set-motor-gains-service

The motor PID gain, according to dynamixel, is P, no ID. 
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/

|Address | 	Size(Byte) | Data Name | Access | Initial| Value | Range | Unit |
| ---- |  ---- |---- |---- |---- |---- |---- |---- |
76 | 2 | Velocity I Gain | RW  | 1,920| 0 ~ 16,383 | - | 
78 | 2 | Velocity P Gain | RW  | 100| 0 ~ 16,383 | - | 
80 | 2 | Position D Gain | RW  | 0| 0 ~ 16,383 | - | 
82 | 2 | Position I Gain | RW  | 0| 0 ~ 16,383 | - | 
84 | 2 | Position P Gain | RW  | 800| 0 ~ 16,383 | - | 
88 | 2 | Feedforward 2nd Gain | RW | 0| 0 ~ 16,383 | - | 
90 | 2 | Feedforward 1st Gain | RW | 0| 0 ~ 16,383 | - | 


There is a motor_config, which is used by driver to load and set register on boot. The file is default reading `interbotix_xsarm_control/config/wx200.yaml`. In this file, we see nothing about setting the PID gains. 

So this is something we should try.

### Problem when setting gain

The `/wx200/set_motor_pid_gains` service seems to cause crashes as soon as I called it with a client. It's a non-catch able crash. Very strange. 

However I was able to use `/wx200/set_motor_registers` to set the P,I and D register separably per motor. 

After upping the gain and giving it some ID gain, The robot is able to reach the target if I commanded it directly using interboticx's `JointGroupCommand`

Using Moveit's execute, things have improved, but still doesn't seems correct.

here is a video where I run the same move twice, using or not using moveit's execute plan.

[](medias/moveit-to-click/moveit_exec_vs_native_Jcmd.mp4)


