
# 
Seems like trossen robotics already have this figured out. 

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/perception_pipeline_configuration.html

## camera calibration

If camera is mounted on gripper, then a manual calibration is need. Actually it might be more of a measuring process. 

If camera is mounted to the side, then better have a calibration process.

Could mount a april tag at the tip of the arm. Then we have two transform: 
* camera to tag frame.
* arm base to arm tip frame. 

The tip frame and tag frame could be almost the same, but the actual TF tree can't be grow this way.

The tree will be 
* camera to tag
* base to tip.

if I add another frame, that's tip to base, so arm side become: 
* fake-tip -> base -> tip 

Then we can chain it like 
* camera -> tag -> fake-tip -> base -> tip. 

`fake-tip -> base` will be published afterword, basically using an invert of `base -> tip`
`tag -> fake-tip` will be a static transform.

---
### Action item 

* 3d print a tag base to hold by the tip of the arm. 
* Write the calibration script.