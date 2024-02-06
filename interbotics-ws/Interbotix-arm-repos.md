

## Setting up 


### Matt's way (MSR hackthon)
There was a interbotix arm setup guide by Matt 

Here are the codes 

```
pip3 install modern-robotics
mkdir -p ~/ws/interbotix/src
cd ws/interbotix
vcs import --input https://nu-msr.github.io/hackathon/images/interbotix.repos src
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
sudo cp src/interbotix_ros_core/interbotix_ros_xseries/\
     interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d
```

Which the content of .repos file is 

```
repositories:
  interbotix_ros_core:
    type: git
    url: https://github.com/interbotix/interbotix_ros_core
    version: e77575dae762c312f7b2410db9402209fdf41609
  interbotix_ros_manipulators:
    type: git
    url: https://github.com/interbotix/interbotix_ros_manipulators
    version: 81ea963f0d36c9c342f4ee53c2fb20debd104a66
  interbotix_ros_toolboxes:
    type: git
    url: https://github.com/interbotix/interbotix_ros_toolboxes
    version: 0b35403eb84bd44c0c5e74f519bdaa2f227dde9a
  interbotix_xs_driver:
    type: git
    url: https://github.com/interbotix/interbotix_xs_driver
    version: v0.3.3
```

So the repo needed is 

* interbotix_ros_core
* interbotix_ros_manipulators
* interbotix_ros_toolboxes
* interbotix_xs_driver


### Instruction by Trossen's doc

The official repo points to this installing script 

https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh

Reading through this, Seems like it is installing these repo 

```
https://github.com/Interbotix/interbotix_ros_core.git
https://github.com/Interbotix/interbotix_ros_manipulators.git
https://github.com/Interbotix/interbotix_ros_toolboxes.git
```

Then using submodule inside `interbotix_ros_core`

```
cd interbotix_ros_core
git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox
git submodule update --init interbotix_ros_xseries/interbotix_xs_driver
```

to grab 
* dynamixel_workbench_toolbox
* interbotix_xs_driver

It optionally installed Modernrobotics library

```
cd interbotix_ros_toolboxes
git submodule update --init third_party_libraries/ModernRobotics
```

Then 
```
    rosdep install --from-paths src --ignore-src -r -y
```

## Conclusion 

Basically these four repos are needed. `dynamixel_workbench_toolbox` seems not related.
* interbotix_ros_core
    * The commit (e77575dae7) matt use is at the HEAD of humble (for now) 
* interbotix_ros_manipulators
    * already have an iron branch
* interbotix_ros_toolboxes
    * Matt's commit (0b35403eb) is quite a few commits behind on latest humble (59bbdc0). Which have some small fixes. So will go off of latest humble
* interbotix_xs_driver
    * The submodule version is at commit (c9ce74e5) Which is also v0.3.3 (same as matt)

So these 4 repos are all cloned with the correct branch. For more coherent and centralize on managing dependencies, the interbotix repo are placed into this repo as submodules.  


Then 

```
rosdep install --from-paths <src> --ignore-src -r -y
```

Despite previously had installed interbotix stuff before. there are still some packages to install. 

The `<src>` is where the 4 repos are. in examples, they are put under the `src` folder in workspace.
