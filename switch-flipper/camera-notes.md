
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

## Camera selection note:

**Following conclusion is found at around 0.3 meter object to camera distance**

D405 is pure camera vision based. It is not able detect non-featured objects. For a switch panel sruface, which is flat and glossy/unifom colored. It is super noisy. 

On the other hand, D435i works very well on getting a good and mostly low noise at the same distance. This is most likely because the projected IR patterns.

However, the camera is already working at its minimal range. Any closer the camera lost the detection. 

According [to this article](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance): reducing the (native) resolution is bad and should not be done unless wanting to reduce minimal operating range.