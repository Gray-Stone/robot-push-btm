# Moible Robot Push Button

This project use wx200 robot arm mounted on a LoCoBot base to push button around the room. 

Majority of this project is system integration, so there might be more document then code.


A demo video of it working.

[Video of it working](https://portfolio.limerobotlab.com/nw/mobile-robot-arm-push-button/medias/Winter_Proj_Push_Water_V1_Compressed.mp4)

Overall things are in these three chunks:

* Robot arm control.
* Mobile base navigation.
* Integration of the two.

**This repo was develoepd towards ros2 iron, and I finished the arm half first with iron. Then due to irobot-create3 base only support humble, I have to swtich over. All the code compile and run in humble, but might see some reference to iron in arm document**

The structure of the repo is:

```
├── Initial-proposel
│   ├── propersal.md
│   ├── proposal-v0.pdf
│   └── Some mobile base size compare.md
├── interbotics-ws
│   ├── discovery-server-inits
│   ├── Interbotix-arm-repos.md
│   ├── LoCoBot_setup_note.md
│   ├── no_discover_setup.zsh
│   └── src
│       ├── create3_sim
│       ├── interbotix_ros_core
│       ├── interbotix_ros_manipulators
│       ├── interbotix_ros_rovers
│       ├── interbotix_ros_toolboxes
│       └── interbotix_xs_driver
├── README.MD
└── switch-flipper
    ├── arm-on-the-turtle.md
    ├── camera-notes.md
    ├── Debug-notes.md
    ├── EE-notes.md
    ├── locobot-usage-notes.md
    ├── medias
    ├── moveit-arm-control-force-issue.md
    ├── moveit-note.md
    ├── src
    │   ├── arm_control
    │   ├── mobile_control
    │   └── vision_process
    └── Trossen-arm-notes.md
```

* README.MD: This file 
* Initial-proposel: documents when initialling starting the project, only read it if you want to know how hard reality is. 
* interbotics-ws: workspace for all interbotix packages. quite a few that we depends on.
    * discovery-server-inits: Helper init scripts if using Interbotix's style for connecting LoCoBot's create3 base, NUC and remote 
    * no_discover_setup.zsh: Helper script to srouce things for LoCobot3 base to run without using discover server. The content should be very trivel. This is What I end up using as well. 
    * Interbotix-arm-repos.md: Document for how I get setup to use the interbotix arms. Also analized Trossen's repo list and explained why I picked my style.
    * LoCoBot_setup_note.md: Similar document as above, but for the LoCoBot base. There are a lot of system setup stuff for LoCoBot in here. Quite important to get things running!
    * src: **The folder I put all sub-module from interbotix (and irobot) into**. I have made a fork for those repos that need modification to run.
        * create3_sim: This I made a fork, but not necessary.
        * interbotix_ros_core
        * interbotix_ros_manipulators
        * interbotix_ros_rovers
        * interbotix_ros_toolboxes
        * interbotix_xs_driver
* switch-flipper: **Packages, codes and documents for putting everything together.**
    * arm-on-the-turtle.md: Final assembly document.
    * camera-notes.md: Camera related notes. Including camera selection.
    * EE-notes.md: Design of end-effector
    * locobot-usage-notes.md: For how to get the LoCoBot base to do slam and follow rivz2 goal-pose. This document connects after `interbotics-ws/LoCoBot_setup_note.md`.
    * medias: Pictures and videos of things. 
    * moveit-arm-control-force-issue.md: PID gain of the wx200 arm. Describe how to get arm to actually go to target, and push hard.
    * moveit-note.md : Notes to get moveit planning to work for wx200.
    * Trossen-arm-notes.md: Very early stage note, descripted how I picked the link to add custom EE.  
    * src: Actual packages with code
        * arm_control: Controls the arm. Node for requesting mvoeit plan and motion execution.
        * mobile_control: Launch the base. Also have a node for moving pan-tilt camera on base from user input in rviz2. (failed to launch arm in the same launch files)
        * vision_process: Mainly used during fixed jig trials with arm and camera on table. Good code reference here for manually calibrate camera offset.

The overall system integration have lots of pitfalls. If wanting to reproduce this project, certainly needs to read through my documents, as well as some Interbotix documents. 