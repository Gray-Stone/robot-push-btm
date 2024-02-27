# LoCoBot


## Basic hardware info

The arrived hardware requires a small amount of assembly.

The robot is sold by Trossen Robotics. The base is a iRobot Create3. Computer onboard is intel NUC. No lidar is included, only a intel D435 camera. The NUC connects to the create3 base via a ethernet connection.

* LoCoBot itself
    * irobot create 3 
    * NUC computer 
    * External battery bank for powering NUC
    * Intel D435 camera
        * Dynamixel motor as pan-tilt base for the camera
    * Additional structural hardware that puts everything together.
    * Additional caster on the rare of create3 for balance.

* Create 3 dock
* NUC power supply (for powering NUC off the wall)
* battery bank charger

A PS4 controller is included with LoCoBot kit, but won't be used in this project.

---
---


## Software system setup for the LoCoBot.

This naming convention is what trossen uses, which I will also use.
>Terminology:
> * "Remote" - Your own personal computer (desktop, laptop, etc.)
> * "Robot" or "LoCoBot" - The NUC computer on the LoCoBot

Refer to [this page](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/create3_configuration.html) for the trossen's note. 

The NUC, which runs user code, and create3 base connects over ethernet, and use ROS to communicate. The connection between NUC and create 3 uses the `192.168.186.0/24` subnet
* create3: 192.168.186.2
* NUC: 192.168.186.3

The create3 configuration and some important ROS2 config can be changed at `192.168.186.2/ros-config`

The NUC also have wifi enabled which is intended to connect to some AP that user is also on for internet and remote development. Which made NUC sitting across two subnets. This will bring some complication to ros side, which will be mentioned later.


## Development (Remote machine) setup

This is the script provided to [setup ros on remote machines](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/software_setup.html#remote-install):

https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh

These are the three main steps the script does for ros2.
```
  install_locobot_ros2
  config_rmw
```

### install_locobot_ros2:

These repose are cloned:
* Interbotix/interbotix_ros_core
    * already exists from the arm setup.
* Interbotix/interbotix_ros_toolboxes
    * already exists from the arm setup.
* Interbotix/interbotix_ros_rovers
    * added as submodule under src.

After adding the `interbotix_ros_rovers` submodule, I switched it to humble branch (which is latest on ros2)

This is also cloned but already available as apt package for iron.
* planning/moveit_visual_tools

The following interbotix ros packages are explicitly enabled (removing COLCON_IGNORE)

* interbotix_ros_toolboxes/interbotix_perception_toolbox 
    * This was pull when setting up arm side, but still ignored. I am questioning wether this is needed.
* interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface
* interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs
    * These two are python moveit example using moveit_commander, thus ignored.

The following interbotix ros packages are explicitly disabled (adding COLCON_IGNORE)

* interbotix_ros_core/interbotix_ros_xseries/interbotix_ros_xseries
    * This seems to be a meta package
* interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    * This is the xs_sdk for interfacing the xseries arms. I believe this is certainly needed for the arm side. (So how does the arm version of locobot move the arm?)
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_ros_xslocobots
* interbotix_ros_rovers/interbotix_ros_xslocobots/examples
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_control
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_moveit
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_nav
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_perception
    * This I actually want to ignored, because we don't use their perception pipeline. But the humble branch already did this.
* interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_ros_control

According to the comment, this should result in following packages being left
* interbotix_xs_msgs
* interbotix_xslocobot_descriptions
* interbotix_xslocobot_sim
* all interbotix_xs_toolboxes packages

I have decided not to ignore these packages for now, as some of them seems rather useful (and build time is not a problem for me)

Certainly a `rosdep install` is needed following the install.

I used `rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO` in the src folder.

In my specific case, this result in some navigation packages, rtapmap packages, lidar packages need to be installed.

### ENV setup

This sets some of the environment variables and push them into bashrc.

* INTERBOTIX_XSLOCOBOT_BASE_TYPE
* INTERBOTIX_WS

in our case, `export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3` is necessary.

### config_rmw

Since the locobot with create3 is effectively 3 network devices (remote machine, NUC, create3) across two subnets (NUC with ethernet port connect to create3 on one subnet, and wifi connect to remove machine on another subnet), the setup from trossen are a bit special

* fastrtps are especially selected for RMW. 
* A specific fastrtps profile is provided.
* ROS discovery server is in use, with the server running no NUC
* special routing rules are added
    * This rule let remote machine route to create3 via the NUC

## Create 3 library install.

When trying to launch just robot description on remote machine, I notice there are create3 dependencies that are necessary.

The package `irobot_create_description` is needed, but not available under iron.

I have added `https://github.com/iRobotEducation/create3_sim` as submodule and add COLCON_IGNORE to everything but the irobot_create_description pakcage.


## LoCoBot system setup



### Things Pre-configured by Trossen

Trossen has pre-configure the NUC and create3 with the ros settings (specially RMW) and cloned workspace onto the robot: https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/software_setup.html#amd64-architecture

Here is the actual install script:
https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/amd64/xslocobot_amd64_install.sh

The script does the following main steps 

* install_create3_ros2
* install_locobot_ros2
* setup_env_vars_ros2
* config_rmw
* Injecte `netplan` setting for NUC to connect to create3

### Install create3 packages

This section is actually empty. 

I guess this is because create2 is already using ros2, and part of the system. So no installation is needed.

### Install locobot ros2 

The following repo are cloned

* Interbotix/interbotix_ros_core
* Interbotix/interbotix_ros_rovers
* Interbotix/interbotix_ros_toolboxes

as well as 
```
Slamtec/sllidar_ros2
```
(which is still **not** available in ros iron)

Then two submodules under `interbotix_ros_core` are initialized
```
git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox
git submodule update --init interbotix_ros_xseries/interbotix_xs_driver
```

udev rule under `interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk` is copied and installed.

`rosdep` install is then run.

The system is setup to use discovery server (pointing on NUC )
