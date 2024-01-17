#! /usr/bin/env python3
import queue
import threading
import time

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node as RosNode
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# wx200 joint name list [waist,shoulder,elbow,wrist_angle,wrist_rotate,gripper,left_finger,right_finger]

# JS when right under the toggle [0.041417, -0.352815, 0.951068, -0.555301, 0.006135, -0.566038, 0.013381, -0.013381]


class JointIO():
 
    def __init__(self):
        self.ros_node = RosNode("receiver_node")

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=10,
                         durability=QoSDurabilityPolicy.VOLATILE)

        self.ros_node.create_subscription(JointState,"/px100/joint_states" , self.joint_state_callback,qos)

        self.jcmd_pub =  self.ros_node.create_publisher(
            JointGroupCommand, f'/px100/commands/joint_group', 10
        )

        self.js_queue :queue.Queue[JointState] = queue.Queue()
        self.last_callback_py_ns = time.monotonic_ns()

    def spin_in_thread(self):
        self.ros_spin_thread = threading.Thread(target=rclpy.spin , args = [self.ros_node])
        self.ros_spin_thread.start()

    def cmd_pos(self, pos_list):

        self.jcmd_pub.publish(JointGroupCommand(name='arm', cmd=pos_list))
        print(f"Ros msg sent {JointGroupCommand(name='arm', cmd=pos_list)}")

    def joint_state_callback(self,msg:JointState):

        # These are for debugging timing performance
        # called_time = time.monotonic_ns()
        # dt_ns = called_time - self.last_callback_py_ns
        # print(f"Callback-happened! dt {dt_ns/1e6 :.3f} ms")
        # if dt_ns / 1e6 > 120:
        #     print(f"!!!!!!!!!!!!! callback happened toooooo late !")
        # self.last_callback_py_ns = called_time

        self.js_queue.put(msg)


def main():

    robot = InterbotixManipulatorXS("px100", "arm", "gripper" , moving_time=0 , accel_time=0)
    # Without setting this, the following code will fly through right away. regardless of what's going on
    robot.arm.set_trajectory_time(3,1)
    # robot.start()
    # print("Going to sleep pose")
    # robot.arm.go_to_sleep_pose(blocking=True)
    # This can't be used on wx200 in the current config
    print("Going to home pose")
    robot.arm.go_to_home_pose()
    print("releasing gripper")
    robot.gripper.release(delay=0.5)
    # robot.gripper.gripper_state()
    print("closing gripper")
    robot.gripper.grasp(delay=1)
    # robot.gripper.gripper_state()
    print("Going back to sleep pose")
    robot.arm.go_to_sleep_pose()

    robot.shutdown()


if __name__ == '__main__':
    main()

# Simply move the arm form one point to another.







