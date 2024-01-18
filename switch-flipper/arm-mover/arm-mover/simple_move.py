#! /usr/bin/env python3
import queue
import threading
import time
import numpy as np
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

def px100_routine():
    robot = InterbotixManipulatorXS("px100", "arm", "gripper" , moving_time=0 , accel_time=0)
    # Without setting this, the following code will fly through right away. regardless of what's going on
    robot.arm.set_trajectory_time(3,1)
    robot.start()
    # This can't be used on wx200 in the current config
    print("Going to home pose")
    robot.arm.go_to_home_pose()
    print("releasing gripper")
    robot.gripper.release(delay=0.5)
    # robot.gripper.gripper_state()
    print("closing gripper")
    robot.gripper.grasp(delay=1)
    robot.gripper.gripper_state()

    # ee_matrix = robot.arm.get_ee_pose()
    # Due to the gravity droop, this will not accumulat previous error
    ee_matrix = robot.arm.get_ee_pose_command()
    print(f"Current EE = \n{ee_matrix} ")
    # This is how to do a relative move
    ee_matrix[2,3] += 0.05
    print(f"moving ee to \n{ee_matrix}")
    robot.arm.set_ee_pose_matrix(ee_matrix)
    # time.sleep(1)

    print("Finishing up, Going back to sleep pose")
    robot.arm.go_to_sleep_pose()

    robot.shutdown()

def wx200_routine():
    robot = InterbotixManipulatorXS("wx200", "arm", "gripper" , moving_time=0 , accel_time=0)
    # Without setting this, the following code will fly through right away. regardless of what's going on
    robot.arm.set_trajectory_time(3,1)
    # robot.arm.core.
    robot.start()

    # while True:
    #     ee_matrix = robot.arm.get_ee_pose()
    #     print(f"Current EE = \n{ee_matrix.tolist()} ")



    print("Starting sequence")
    # robot.arm.go_to_sleep_pose()

    ee_target_matrix = np.array([[0.9993107, -0.0295254, -0.02250294, 0.24924762],
                                 [0.0291338, 0.99942168, -0.01753621, 0.00726654],
                                 [0.02300769, 0.01686852, 0.99959297, 0.19003627],
                                 [0., 0., 0., 1.]])
    print(f"Going to prep position \n{ee_target_matrix}")
    robot.arm.set_ee_pose_matrix(ee_target_matrix)


    # while True:
    #     print(f"current joint cmd {robot.arm.get_joint_commands()}")

    print("releasing gripper")
    robot.gripper.release(delay=0.5)
    # robot.gripper.gripper_state()
    print("closing gripper")
    robot.gripper.grasp(delay=1)
    robot.gripper.gripper_state()

    # ee_matrix = robot.arm.get_ee_pose()
    # Due to the gravity droop, this will not accumulat previous error
    ee_under_switch = np.array(
        [[0.9972798962413734, -0.0737075754818405, 4.3236376512700674e-05, 0.30368641090350945],
         [0.07356378480184264, 0.9952974096251082, -0.06301933004386602, 0.022401255518282964],
         [0.004601968972478693, 0.06285109155884497, 0.9980123056212473, 0.21501519640450817],
         [0.0, 0.0, 0.0, 1.0]])

    print(f"Going to right under the switch \n{ee_under_switch} ")
    robot.arm.set_ee_pose_matrix(ee_under_switch)
    # This is how to do a relative move



    # Do a up, but pull out a little
    ee_switch_up = ee_under_switch
    ee_switch_up[2,3] += 0.04
    # ee_target_matrix[1,3] -= 0.002
    print(f"moving ee up tp \n{ee_switch_up}")
    # Changing the move time to 1 still doesn't do it very well.
    robot.arm.set_ee_pose_matrix(ee_switch_up,moving_time=1)

    # # turn just one joint.
    # last_cmd_js = robot.arm.get_joint_commands()
    # new_cmd_js = last_cmd_js.copy()
    # new_cmd_js[3] = new_cmd_js[3] - 0.15
    # print(f"Changing joint cmd from \n{last_cmd_js} \nto \n{new_cmd_js}")
    # robot.arm.set_joint_positions(new_cmd_js,moving_time=1)


    # print("Finishing up, Going back to sleep pose")
    # robot.arm.go_to_sleep_pose()

    robot.shutdown()



def main():

    wx200_routine()

if __name__ == '__main__':
    main()

# Simply move the arm form one point to another.
