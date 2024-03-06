# LoCoBot

**Create3 Base can only use ros2 humble. Don't try to use iron for now**

**It take a few minutes for Create3 to boot or restart_application, just be patient**

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

This is also cloned but already available as apt package.
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


### Network setup 

Add the routing into the NUC and creaet3 subnet `192.168.186.0/24` via the NUC
```
sudo ip route add 192.168.186.0/24 via {ip of locobot}

```

Routing must be done using IP address. Here is a trick to auto find it by hostname
```
sudo ip route add 192.168.186.0/24 via $(getent hosts {locobot_hostname} |  awk '{ print $1 }')
```


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

Since the LoCoBot version I received have too old a ubuntu distro. I opt for re-imaging the entire machine and manually do the setup.

### Install create3 packages

This section is actually empty from the trosson setup script.

I guess this is because create2 is already using ros2, and part of the system. So no installation is needed.

### Install locobot ros2 

From the trossen script: 
The following repo are cloned

* Interbotix/interbotix_ros_core
* Interbotix/interbotix_ros_rovers
* Interbotix/interbotix_ros_toolboxes

as well as 
```
Slamtec/sllidar_ros2
```
which is still **not** available in ros iron. But since we don't have lidar, it's ignored.

Then two submodules under `interbotix_ros_core` are initialized
```
git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox
git submodule update --init interbotix_ros_xseries/interbotix_xs_driver
```

### My LoCoBot Setup steps

#### system setup
First, some common system level maintance for robotic setup. They are generally steps taken out of here 
```
https://nu-msr.github.io/hackathon/computer_setup.html#org5a1d4da
```

remove auto update and upgrade
```
sudo apt purge unattended-upgrades update-manager update-manager-core
sudo apt autopurge
```

remove snap completely.

```
sudo snap remove firefox
sudo snap remove snap-store
sudo snap remove gtk-common-themes
sudo snap remove gnome-42-2204
sudo snap remove snapd-desktop-integration
sudo snap remove core22
sudo snap remove bare
sudo snap remove snapd

# Then remove snap itself 

sudo apt purge snapd

rm -rf ~/snap

sudoedit /etc/environment
# Then remove snap from path
```

Also make sure ROS is setup on the system. 

```
# Download ROS encryption keys
sudo wget -O /usr/share/keyrings/ros-archive-keyring.gpg \
     https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

# Add the Repository by creating /etc/apt/sources.list.d
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu jammy main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list

# Update the package list
sudo apt update

# Install the development tools
sudo apt install ros-dev-tools

# Install ros2 iron
sudo apt install ros-iron-desktop-full

# Initialize rosdep (makes it easy to install ros dependencies)
sudo rosdep init
rosdep update
```

#### Interbotix work space setup

For my setup, I have simply clone this repo `git clone https://github.com/Gray-Stone/robot-push-btm.git` to the robot. Since the repo have marked out all the necessary interbotix folders and submodules.

cd into the repo, then:

```
git submodule init 
git submodule update
```
This will setup 4 top level repos I manually picked out. No nested submodule should be setup (or duplicate will show up).

dynamixel_workbench_toolbox will get dragged in later with the rosdep install.

udev rule under `interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk` is copied to `/etc/udev/rules.d/` and installed 

```
sudo cp interbotics-ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

To double check this step: 
```
$ ls /dev | grep ttyDXL
ttyDXL
```

source `/opt/ros/iron/setup.zsh` then
`rosdep` install is then run.

#### RMW setup

The system is setup to use discovery server (pointing on NUC )
So the following stuff need to be setup 

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE} # This is calculated live
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
```

Also a script with following content is set to be run by systemd 
```
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 &

```

This script hard coded the ros version. and as a running on boot script, it is often forgotten which might cause a bug. Thus I decide to move it to a user initiated style.


**I end up just connecting the create3 base and NUC both to wifi instead of using this discovery node**





#### Network setup

NUC needs to have its network port assigned a fixed address (and put into create3's setting) to talk to create3. 

The content of the file is at `interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/conf/99_interbotix_config_locobot.yaml`

```
sudo cp interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/conf/99_interbotix_config_locobot.yaml /etc/netplan/
sudo netplan apply
```

Also the NUC is acting as the "router" between remote machine and create3 base. Thus it need to have its ip forwarding enabled.

**I have not been getting the routing part to work, I just go back to a simple "all on same network" setup"**

https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/rmw_configuration.html#ip-forwarding-enabled

#### NTP server setup.

We want the time from create3 to be in sync with the NUC. 

Follow this [page](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/troubleshooting.html#create-3-base-clock-is-not-synchronized) to setup a NTP server and allow machines in subnet to sync to it.

### Create3 settings 

Create3's firmware needs to be updated some time, or switch to the right ROS version. I have tried to manually upload the firmware file, but not working. It's better just let Create3 connect to internet and download the firmware and install it all by itself.

There are some ros settings might need changing.

* ROS_DOMAIN_ID 
* robot_namespace
    * the default is mobile_base, but to work with interbotix, this likely need to be change to something like `locobot/mobile_base`

If communicating to create3 with the wrong ros version, it's likely to cause it to go into fault mode (red light ring).

### Create3 time 

In the advance setting of create3, the ntp.conf has the following
```
# irobot servers
	server 192.168.186.3 prefer iburst
```

Seems like this local connection between Create3 and NUC is also needed to correct create3's time.