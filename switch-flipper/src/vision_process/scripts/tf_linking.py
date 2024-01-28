#! /usr/bin/env python3

# Link the tf between robot amd camera.
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer as TfBuffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from rclpy.node import Node as RosNode
from threading import Thread
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy

from collections import deque
from multiprocessing import Queue

def main():
    rclpy.init()

    ctr = CameraToBase()
    rclpy.spin(ctr)
    rclpy.shutdown()


class CameraToBase(RosNode):

    TF_PUB_PERIOD = 0.5
    CAMERA_LINK_NAME = "d435i_link"
    def __init__(self):
        super().__init__("camera_robot_linker")
        self._transform_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = TfBuffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # We will do left is newer. So append left, pop right.
        self._user_input_queue = deque(maxlen=5)

        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._tf_pub_timer = self.create_timer(self.TF_PUB_PERIOD, self.publish_cycle,
                                               self._timer_callback_group)

        self._input_thread = Thread(target= self.getting_input)
        self._input_thread.start()

        self.base_to_camer_tf = TransformStamped()
        # These are world to camera base
        # self.world_to_camer_tf.translation.x = -60
        # self.world_to_camer_tf.translation.y = -176.5
        # self.world_to_camer_tf.translation.z = 143

        self.base_to_camer_tf.header.frame_id = "camera_mount"
        self.base_to_camer_tf.child_frame_id = self.CAMERA_LINK_NAME

        self.base_to_camer_tf.transform.translation.x = 0.01
        self.base_to_camer_tf.transform.translation.y = 0.01
        self.base_to_camer_tf.transform.translation.z = 0.012
        

    def publish_cycle(self):

        if len(self._user_input_queue) >0:
            new_cmd = self._user_input_queue.pop()
            print(f"Get user input: {new_cmd.__repr__()}")
        
        self.base_to_camer_tf.header.stamp = self.get_clock().now().to_msg()
        self._transform_broadcaster.sendTransform(self.base_to_camer_tf)
        # else:
            # print("Sees nothing")

    def getting_input(self):
        while True:
            # print(f"Waiting for user input")
            new_input = input()
            self._user_input_queue.appendleft(new_input)
            print(f"User inputted: {new_input.__repr__()}")



if __name__ == '__main__':
    main()
