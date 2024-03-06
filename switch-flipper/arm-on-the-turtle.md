# Putting arm onto the turlte (LoCoBot)


## Hardware wiring.

The wiring is the wx200 arm's power board use screw terminal to get power from the base's board. But still have the U2 usb2serial adapter and plug in as a seperate usb device.

## Talking to the right serial port 

If I leave both arm and pantilt plug in, launching the base launch file will result the xs driver not able to find the right motor for the pan-tilt kit 

```
[INFO] [camera_tilt-6]: process started with pid [3295]
[xs_sdk-3] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-3] [INFO] Using logging level 'INFO'.
[xs_sdk-3] [INFO] Loaded mode configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/modes_base.yaml'.
[xs_sdk-3] [INFO] Loaded motor configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/locobot_base.yaml'.
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 11, Joint Name: 'tilt':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'
[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 10, Joint Name: 'pan':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 2/3)
[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 11, Joint Name: 'tilt':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'

[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 10, Joint Name: 'pan':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 3/3)


[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 11, Joint Name: 'tilt':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'

[xs_sdk-3] [ERROR]      Can't find DYNAMIXEL ID: 10, Joint Name: 'pan':
[xs_sdk-3]                '[TxRxResult] There is no status packet!'
[xs_sdk-3] [FATAL] Failed to find all motors. Shutting down...
```

The quick solution is to disconnect the arm's usb cable. Launch the LoCoBot nodes, then plugin the WX200 and launch the arm after the base side is running. Then launch the arm nodes. Now they can find the right ports to talk to.

### Logs from when it worked 

From the arm nodes:
```
[INFO] [xs_sdk-6]: process started with pid [4064]
[xs_sdk-6] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-6] [INFO] Using logging level 'INFO'.
[xs_sdk-6] [INFO] Loaded mode configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/modes.yaml'.
[xs_sdk-6] [INFO] Loaded motor configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/wx200.yaml'.
[xs_sdk-6] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  6, Model: 'XL430-W250', Joint Name: 'wrist_rotate'.
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  4, Model: 'XM430-W350', Joint Name: 'elbow'.
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  2, Model: 'XM430-W350', Joint Name: 'shoulder'.
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  7, Model: 'XL430-W250', Joint Name: 'gripper'.
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  3, Model: 'XM430-W350', Joint Name: 'shoulder_shadow'.
[xs_sdk-6] [INFO]       Found DYNAMIXEL ID:  1, Model: 'XM430-W350', Joint Name: 'waist'.
```

From the LoCoBot process

```
[INFO] [xs_sdk-3]: process started with pid [3406]
[xs_sdk-3] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-3] [INFO] Using logging level 'INFO'.
[xs_sdk-3] [INFO] Loaded mode configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/modes_base.yaml'.
[xs_sdk-3] [INFO] Loaded motor configs from '/home/leogray/robot-push-btm/interbotics-ws/install/interbotix_xslocobot_control/share/interbotix_xslocobot_control/config/locobot_base.yaml'.
[xs_sdk-3] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[xs_sdk-3] [INFO]       Found DYNAMIXEL ID: 11, Model: 'XL430-W250-2', Joint Name: 'tilt'.
[xs_sdk-3] [INFO]       Found DYNAMIXEL ID: 10, Model: 'XL430-W250-2', Joint Name: 'pan'.
```

### Existing config file
The motor config file for the base: `interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_control/config/locobot_base.yaml` have the following on top of it:

```
port: /dev/ttyDXL

joint_order: [pan ,tilt]
sleep_positions: [0, 0]

```

The motor config file for the arm `/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/wx200.yaml` (found this file through xsarm_moveit.launch.py -> xsarms_ros_control.launch.py -> xsarms_control.launch.py). Have the following on top 

```
port: /dev/ttyDXL
```

under /dev 

```
lrwxrwxrwx   1 root root             7 Mar  6 15:58 ttyDXL -> ttyUSB1

crw-rw-rw-   1 root dialout 188,     0 Mar  6 16:00 ttyUSB0
crw-rw-rw-   1 root dialout 188,     1 Mar  6 16:00 ttyUSB1

```
