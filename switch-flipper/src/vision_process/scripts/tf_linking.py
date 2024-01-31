#! /usr/bin/env python3

# Link the tf between robot amd camera.
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer as TfBuffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from rclpy.node import Node as RosNode
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy

import readchar

from collections import deque


def main():
    rclpy.init()

    ctr = CameraToBase()
    try:
        rclpy.spin(ctr)
    except KeyboardInterrupt as e:
        import traceback
        print(f"Getting keyboard interrupt, with {e}")
        print(traceback.print_stack(limit=2))
    rclpy.shutdown()


class CameraToBase(RosNode):

    TF_PUB_PERIOD = 0.1
    CAMERA_LINK_NAME = "d435i_link"

    SINGLE_KEY_INCREMENT = 1 / 1000  # 1mm
    HOLD_SHIFT_INCREMENT = 10 / 1000
    X_INCREMENT_KEY = "w"
    y_INCREMENT_KEY = "a"
    z_INCREMENT_KEY = "r"
    X_DECREMENT_KEY = "s"
    y_DECREMENT_KEY = "d"
    z_DECREMENT_KEY = "f"

    def __init__(self):

        super().__init__("camera_robot_linker")
        self._transform_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = TfBuffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.base_to_camer_tf = TransformStamped()
        self.base_to_camer_tf.header.frame_id = "camera_mount"
        self.base_to_camer_tf.child_frame_id = self.CAMERA_LINK_NAME
        self.base_to_camer_tf.transform.translation.x = 0.01
        self.base_to_camer_tf.transform.translation.y = 0.01
        self.base_to_camer_tf.transform.translation.z = 0.012
        # output from last try new transform:translation:  x: 0.015000000000000013  y: 0.04200000000000002  z: 0.029000000000000005 rotation:  w: 1.0  x: 0.0  y: 0.0  z: 0.0



        # We will do left is newer. So append left, pop right.
        self._user_input_queue = deque(maxlen=5)
        self._interrupt = threading.Event()

        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._tf_pub_timer = self.create_timer(self.TF_PUB_PERIOD, self.publish_cycle,
                                               self._timer_callback_group)

        self._input_thread = threading.Thread(target=self.getting_input)
        self._input_thread.start()

    def publish_cycle(self):

        if len(self._user_input_queue) > 0:
            new_k: str = self._user_input_queue.pop()
            self.modify_tf_by_key(new_k)
            print("new transform:"+self.print_transform(self.base_to_camer_tf.transform))


        self.base_to_camer_tf.header.stamp = self.get_clock().now().to_msg()
        self._transform_broadcaster.sendTransform(self.base_to_camer_tf)

        if self._interrupt.is_set():
            raise KeyboardInterrupt("Programmed keyboard interrupt")
        # else:
        # print("Sees nothing")

    def modify_tf_by_key(
        self,
        key: str,
    ):

        if key.isupper():
            print("is upper")
            increment = self.HOLD_SHIFT_INCREMENT
        elif key.islower():
            print("is lower")
            increment = self.SINGLE_KEY_INCREMENT
        else:
            return

        if key.lower() == self.X_INCREMENT_KEY:
            self.base_to_camer_tf.transform.translation.x += increment
        elif key.lower() == self.y_INCREMENT_KEY:
            self.base_to_camer_tf.transform.translation.y += increment
        elif key.lower() == self.z_INCREMENT_KEY:
            self.base_to_camer_tf.transform.translation.z += increment
        elif key.lower() == self.X_DECREMENT_KEY:
            self.base_to_camer_tf.transform.translation.x -= increment
        elif key.lower() == self.y_DECREMENT_KEY:
            self.base_to_camer_tf.transform.translation.y -= increment
        elif key.lower() == self.z_DECREMENT_KEY:
            self.base_to_camer_tf.transform.translation.z -= increment
        return

    def getting_input(self):
        while True:
            try:
                k = readchar.readkey()
                self._user_input_queue.appendleft(k)
            except KeyboardInterrupt:
                print("Interrupted from keyboard")
                print("Current transform is = " +
                      self.print_transform(self.base_to_camer_tf.transform))
                self._interrupt.set()
                break

    def print_transform(self, tf: Transform):
        return (f"translation: "
                f" x: {tf.translation.x} "
                f" y: {tf.translation.y} "
                f" z: {tf.translation.z} "
                "rotation: "
                f" w: {tf.rotation.w} "
                f" x: {tf.rotation.x} "
                f" y: {tf.rotation.y} "
                f" z: {tf.rotation.z} ")


if __name__ == '__main__':
    main()
