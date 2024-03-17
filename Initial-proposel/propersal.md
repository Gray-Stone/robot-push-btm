# Mobile manipulator for pushing buttons.

 <!-- 3.1 Project Description
• A short summary of the project (no more than four sentences).
• The description should clearly describe the final product (think about
what video you want to show at the end).
• If there is a source of inspiration for the project, mention it here.
• The project should have a cohesive narrative that motivates the goal. -->

## Project Description

This project aimed at using a mobile manipulator to push buttons and toggle switches instead of humans. The robot will be interact with targets (buttons and swtiches) in a semi-autonomous fashion. Robot will provide user with an visual representation of the surrounding, user will select the location of the the buttons and switchs to operate, along with necessary instruction on how to operate it, and robot will complete the rest of the operations.

The insperation of the project:  

There are planty of lights and buttons being pushed everyday. Some of them, like light switches, have connected alternative that's controable through software, however, majorities of them are not trivel to be converted to connected version. For example, the start button of a washing machine. In order to automate more everyday appliances without needing a "smart" version of all of them, a robot that can push button will bring great convenience to everyday life.


### What this project is not:

This project is not to design a complete human machine interface for teleoperating swtiching with minimal human input. Thus the final project might have a clumzy interface for operation.

This project is also not a "pure teleoperate" project. User are not expected to directly control any joints of the robot, but rather, give high level instruction on end-effector motions.

<!-- ## 3.2 Technical objectives
• A list of three technical goals: fallback, core, and reach.
• Fallback goal
– Achievable even if you experience significant technical setbacks.
– Include technical shortcuts that simplify the project.
– Should be a step on the path to accomplishing the other goals
that also helps you refine and clarify them.
– Minimal functionality for showing that you achieved a result that
fits your project narrative.
• Core goal
– Basic functionality demonstrating the essence of the project goals.
– These goals should assume that most uncertainties and challenges
are successfully overcome.
– After achieving the core goals, you should be able to demonstrate
the system acting largely in accordance with the Project Descrip-
tion.
– Assumes that almost everything goes according to plan.
• Reach goal
– Extension of the core goals with a greater degree of difficulty,
representing the most sophisticated version of the project that
can possibly be accomplished during the quarter.
– Path to accomplishing the goal may be unclear at the outset of the
project, but should become clear after the core goals are reached.
– Assumes that everything goes according to plan and that the core
goal is reached after 8 weeks. -->

## Technical objectives

### Fallback goal

Targets are choosen with specific type for easy manipulation and are placed at easy to reach locations. For example some light switches firmly moutned on the side of tables to meet the reach-envolope for the robot.

Navigation environment will be greatly simplified with complex geometries coverted off with rigid surfaces. More of a low poly 3D cubed maze.

### core goal

* Robot able to reach and operate existing light swtiches and buttons installed for human interaction. 
* Targets could be marked with additional visual tags to help locate them and simplify human interface design (as that's not part of the project's goal). 
* Manipulator have a set of pre-defined motions for different buttons and swtiches that operator can choose from. 

### Reach goal

* Manipulator able to push some buttons on not perminently fixed panels (like a keyboard on table)
* End effector force of the manipulator is tracked over time. The tack-tile feedback in most buttons and swtiches will be captured and detected.
* Robot able to identify and locate certain kinds of switches (pre-defined) visually, which allow operator to command operating certain switch in one click.


 <!-- 3.3 Learning objectives
• A list concepts or skills that you hope to learn from this project.
• Generally, there should be 1-3 learning objectives, anything else is
usually an indication that the project idea is too broad or too vague. -->

## Learning objectives

From core goal
* Mobile manipulation taks and motion planning in 3d space (with octomap or similar)
* (Potential) Manipulator mounting on mobile platform, specifically z axis elevator for increasing reach of short robots

From reach goal
* Force feedback from task
* CV object detection with machine learning (YOLO)


<!-- 
3.4 Tasks
• A list of approximately 4-8 higher-level tasks that you need to accom-
plish to fulfill your goals.
• An estimate of the time required for each task (in days or weeks).
• At least 2 of the tasks should be designated for completion within the
first two weeks of the quarter
• At least 2 of the tasks should be targeted at achieving the core goals. -->

## Tasks

1. Selecting and building of the mobile manipulator.
2. 3D slam using a combination of sensors. Contructing and using octomap
3. Motion planning the mobile base to place target in reach of the arm with high manipulability.
4. Motion planning of robot arm to manipulate switches and buttons
5. Basic interface for operator to see surranding of robot. Mark position, type and oriantation of a switch and get visual feedback to confirm the selection.
6. End effector force reading and detect the ticktile feedback
7. CV for auto detecting targets


<!-- 
3.5  Risks, Challenges and Uncertainties
• What part of the project do you think has the highest risk of failure?
• What parts of the project are you currently least sure about how to
approach?
• How do you intend to address the uncertainties (e.g., by reading or
experimenting)? -->

## Risks

### Hardware selection and custom hardware design.

Selecting a "best fit" mobile manipulator might result in some challange. Some existing options might be too small/short, while others might be too big to fit in thigher environments. If custom modification were to be made, it might take signifigent amount of time to debug the hardware (like adding a actuator on Z axis).

Could be solved by having backup robot options that's fully integrated and ready to use. Time box the hardware custimization and drop back to fallback option early on.

Having good hardware abstraction would also help the potentional of swtiching hardware if certain route are proven to be not possible within the time-constrain.

### SLAM with 3d mapping

This is something completely new to me. Planty of time need to be spent in learning the tools, reading publications, and expermenting on actual hardware.

The plan is to use complete solution packages as much as possible.

### Obstical avoidance during motion

Since the project target at human enviornment, which is very complex, it might be difficult to generate a clear 3d mapping and avoid all obstical during motion.

Could try to simplify the environment by removing objects or cover them with sensor friendly materials.

### HMI design

Designing of UI could be a huge time-sink.

Push this task to later and timebox it, and it's less of a critical part of the system.

<!-- 
3.6 Tools and References
• A list of hardware and software you think you need to complete the
project.
• If unsure, list some possibilities and criteria for choosing between them.
• If you have special hardware requests (anything over the purchasing
limit or with a long lead time) please email me.
• What reference material are you considering using (papers, blog posts,
text books)? -->

## Tools and References

### Hardware
Mobile manipulator:

The manipulator need to be able to reach light switches height (125mm) while reaching outside the base-radius and still have some manipulability (would with the help of extended end effector)

* Option 1: Ridgeback + Sawyer arm. But this is really huge. might makes it more difficult to motion plan the arm for collision free motion

* Option 2: Jackal Robot + Widow 200X robot arm (with girpper swapped out for extra dof)
    * This option require an additional moutning design to reach enough height, and/or a custom built z-lift to make up for arm's reach

Force sensor for each goal

Decently powerful computer with cabilities of live 3d SLAM onboard of the mobile base. Likely require a recent GPU if octomap-rt were to be used.

Sensors: 

* 3d Lidar or depth camera


### Reference materials

* octomap and octomap RT for 3d mapping
    * https://github.com/OctoMap/octomap
    * http://graphics.ewha.ac.kr/OctoMap-RT/