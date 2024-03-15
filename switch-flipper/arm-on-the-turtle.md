# Putting arm onto the turlte (LoCoBot)


## Hardware wiring.

The wiring is the wx200 arm's power board use screw terminal to get power from the base's board. But still have the U2D2 usb2serial adapter and plug in as a seperate usb device.

## Talking to the right serial port

Trosson generally use a U2D2 device to convert usb to serial for the motors. Both the pantilt kilt and the wx200 uses this device for machine directly control it. 

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

### udev rule time 

The actual solution is to do add a special udev-rule to seperate both 

Here is an example U2D2 device's output.

```
▶ udevadm info /dev/ttyUSB0      
P: /devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2:1.0/ttyUSB0/tty/ttyUSB0
N: ttyUSB0
L: 0
S: ttyDXL
S: serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0
S: serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FCKZ-if00-port0
E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2:1.0/ttyUSB0/tty/ttyUSB0
E: DEVNAME=/dev/ttyUSB0
E: MAJOR=188
E: MINOR=0
E: SUBSYSTEM=tty
E: USEC_INITIALIZED=4243601
E: ID_BUS=usb
E: ID_VENDOR_ID=0403
E: ID_MODEL_ID=6014
E: ID_PCI_CLASS_FROM_DATABASE=Serial bus controller
E: ID_PCI_SUBCLASS_FROM_DATABASE=USB controller
E: ID_PCI_INTERFACE_FROM_DATABASE=XHCI
E: ID_VENDOR_FROM_DATABASE=Future Technology Devices International, Ltd
E: ID_AUTOSUSPEND=1
E: ID_MODEL_FROM_DATABASE=FT232H Single HS USB-UART/FIFO IC
E: ID_VENDOR=FTDI
E: ID_VENDOR_ENC=FTDI
E: ID_MODEL=USB__-__Serial_Converter
E: ID_MODEL_ENC=USB\x20\x3c-\x3e\x20Serial\x20Converter
E: ID_REVISION=0900
E: ID_SERIAL=FTDI_USB__-__Serial_Converter_FT89FCKZ
E: ID_SERIAL_SHORT=FT89FCKZ
E: ID_TYPE=generic
E: ID_USB_INTERFACES=:ffffff:
E: ID_USB_INTERFACE_NUM=00
E: ID_USB_DRIVER=ftdi_sio
E: ID_PATH=pci-0000:00:14.0-usb-0:2:1.0
E: ID_PATH_TAG=pci-0000_00_14_0-usb-0_2_1_0
E: ID_MM_CANDIDATE=1
E: ID_MM_DEVICE_IGNORE=1
E: DEVLINKS=/dev/ttyDXL /dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0 /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FCKZ-if00-port0
E: TAGS=:systemd:
E: CURRENT_TAGS=:systemd:
```

This is the line in interbotix's udev rule that reconize u2d2
```
# U2D2 board (also sets latency timer to 1ms for faster communication)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL"
```

We can see these attributes are matching from the printed info. 

The best way to tell the difference between two identical device is usually the serial number. 

Here the serial from the other U2D2 device 
```
E: ID_SERIAL=FTDI_USB__-__Serial_Converter_FT3R4903
E: ID_SERIAL_SHORT=FT3R4903
```


```
udevadm info /dev/ttyUSB0 -a
```

This command's output will show the actual value used in udev-rule matching.

note: 
The actual device configuration could be a lot more complicated. If we look for the same vender ids from above, it's actually in a parent device.

For this specific Locobot's pan-tilt kit, the serial number for the specific U2D2 on the robot is `ATTRS{serial}=="FT89FCKZ"` This is what we will do. 

#### Only lock down base-specific devices

The design here to pin down the locobo's device, as these usually doesn't change. But the arm might change, so leave those at default.

# U2D2 board (also sets latency timer to 1ms for faster communication)
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT89FCKZ", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK="ttyPanTilt", SYMLINK-="ttyDXL"
```

But when you think it should work, well, systemd-udev have a bug! 
https://stackoverflow.com/a/73661279 the `=` and `-=` rules doesn't work. despite we can add another name to it, we can't specify the pan-tilt unit to only be this name. 

So I have to replace their original udev rule to exclude this serial number as well.

### Change xslocobot launch file 

Since the serial port name have changed, we need to also change the motor config file (the first line of it list the port/dev name)

The config file loaded is `/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_control/config/locobot_base.yaml` 

However from the rtabmap launch file, there isn't an option to set this to something else, So I opt for directly modifying it.

## Mounting the arm

The arm is clamped onto top of the lidar tower, since we don't have a lidar anyway. The tower allow the arm to be mounted slightly forward while be above the camera pan-tilt to not block it too much.

![Arm mounted on locobot](medias/Arm-Mounted-On-Locobot.png)

### Linking URDF 

Inspecting the urdf, the `locobot/lidar_tower_link` seems to be a good point to publish the static tf off of. 

This link only show up when using lidar.

Inspecting the launch file chain of the locobot, seems like the urdf itself does accept the external_urdf argument, but the higher level launch file doesn't have this option. (even xslocobot_description.launch.py doesn't take it). But when running in terminal, it does seems to take it ? So let's use it.


note:
```
  <xacro:if value="${arm_type == 'mobile_wx250s'}">
    <xacro:property name="camera_tower_size" value="large"/>
  </xacro:if>
  <xacro:unless value="${arm_type == 'mobile_wx250s'}">
    <xacro:property name="camera_tower_size" value="small"/>
  </xacro:unless>
```

The robot we have is a small tower size.