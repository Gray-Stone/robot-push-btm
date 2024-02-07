#! /usr/bin/env python
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer as TfBuffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from rclpy.node import Node as RosNode
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rcl_interfaces.msg import ParameterDescriptor as RosParameterDescriptor
import readchar

from collections import deque
from typing import Optional

import tf2_ros


def main():
    rclpy.init()

    cal = TfCal()
    try:
        rclpy.spin(cal)
    except KeyboardInterrupt as e:
        import traceback
        print(f"Getting keyboard interrupt, with {e}")
        print(traceback.print_stack(limit=2))
    rclpy.shutdown()


class TfCal(RosNode):

    TF_PUB_PERIOD = 0.3

    def __init__(self):

        self.parent_frame = "wx200/base_link"
        self.child_link = "lollypop_ee_link"
        print(f"From {self.parent_frame} to {self.child_link}")


        super().__init__("tf_calculator")

        self._tf_buffer = TfBuffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_cal_timer = self.create_timer(self.TF_PUB_PERIOD, self.cal_timer)


    def cal_timer(self):
        maybe_tf :TransformStamped = self.get_transform(self.parent_frame,self.child_link)
        if maybe_tf:

            print(f"From {self.parent_frame} to {self.child_link}")
            print(
                f"x: {maybe_tf.transform.translation.x} y: {maybe_tf.transform.translation.y} z: {maybe_tf.transform.translation.z}"
            )
            q = maybe_tf.transform.rotation
            ax, ay, az = euler_from_quaternion((q.x, q.y, q.z, q.w))
            print(f"Rotation is rx :{ax} ry: {ay} rz: {az}")

    def get_transform(self, source_frame,target_frame) -> Optional[TransformStamped]:
        """Get the transform between two frame

        Args:
            target_frame (str): The target frame name
            source_frame (str): The source frame name

        Returns:
            Optional[TransformStamped]: None if error when getting tf. 
                TransformStamped between the frames given 
        """
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            trans = self._tf_buffer.lookup_transform(source_frame,target_frame, rclpy.time.Time())
            return trans
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            print(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            print(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            print(f"Extrapolation exception: {e}")
        return None


if __name__ == '__main__':
    main()
