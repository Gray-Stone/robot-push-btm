

## arm control 

`interbotix_xsarms_control` is the package for connect to robot and estiablish 
https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_control.html#usage

### torque limit

wx200 doesn't have 

When doing a move through set_ee_pose_matrix to flip the switch, the robot can't provide enough torque.


Force required to flip a switch might be defined by some standard, so we could math it out.

https://www.reddit.com/r/estimation/comments/9tjymd/comment/e8xu08i/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button


## Adding stuff to arm.


reference tf-tree of wx200: ![wx200-tf](medias/wx200-tf.svg)

Seems like the best cad reference point for mounting is the `wx200/fingers_link`: 

![wx200-fingers-link](medias/wx200-ee-frame-location.png)

`interbotix_xsarm_descriptions` : https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/arm_descriptions.html 
is the package give the urdf, mesh and everything of the arm.

The `interbotix_xsarms_control` package launch file also refer to the description launch file `xsarm_description.launch.py` for setting up robot and 


## Questions 

trosson perception pipeline? https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/perception_pipeline_configuration.html


## Other tips

Motion sequence suggestion for under 6 dof arms.

https://docs.trossenrobotics.com/interbotix_xsarms_docs/python_ros_interface.html#tips-best-practices