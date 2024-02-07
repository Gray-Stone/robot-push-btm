#! /usr/bin/env python3

from rclpy.node import Node as RosNode
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from rclpy.wait_for_message import wait_for_message
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Header
from geometry_msgs.msg import Pose as PoseMsg

from moveit_msgs.srv import GetPositionIK as GetPositionIKSrv
from moveit_msgs.msg import PositionIKRequest as PositionIKRequestMsg

from moveit_msgs.msg import RobotState as RobotStateMsg
from moveit_msgs.msg import Constraints as MoveitConstrains
from moveit_msgs.msg import OrientationConstraint as MoveitOrientationConstraint
from moveit_msgs.msg import PositionConstraint as MoveitPositionConstraint

from moveit_msgs.msg import BoundingVolume, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import math


class EEPlaner(RosNode):

    EE_LINK_NAME = "lollypop_ee_link"
    BASE_LINK = 'base_link'

    def __init__(self):

        super().__init__("ee_planer")
        # self.group_name = "lollypop_hammer"
        self.group_name = "interbotix_arm"
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=10,
                         durability=QoSDurabilityPolicy.VOLATILE)

        self._js_callback_group = MutuallyExclusiveCallbackGroup()
        self.js_sub = self.create_subscription(JointState, "/wx200/joint_states", self.JointStateCB,
                                               qos,callback_group=self._js_callback_group)

        worked, init_js = wait_for_message(JointState,
                                           self,
                                           "/wx200/joint_states",
                                           time_to_wait=5)

        if not worked:
            raise RuntimeError("No init JS within timeout")
        self.last_js = init_js
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()

        self._tf_cal_timer = self.create_timer(1, self.IkCmdTimer, self._timer_callback_group)

        self.compute_ik_service_ = self.create_client(
            GetPositionIKSrv,
            "compute_ik",
        )

    async def IkCmdTimer(self):
        # Home pose for lollypop_ee_link
        # x: 0.436 y: 0.0 z: 0.31065
        # Good mid air pos

        # x: 0.23154 y: 0 z: 0.27571

        mid_air_pos = PoseMsg()
        mid_air_pos.position.x = 0.23154
        mid_air_pos.position.y = 0.01
        mid_air_pos.position.z = 0.27571

        await self.ComputeIK(mid_air_pos)

    def JointStateCB(self, msg: JointState):
        self.last_js = msg

    def GenEEHeader(self):
        header = Header()
        header.frame_id = self.BASE_LINK
        header.stamp = self.get_clock().now().to_msg()
        return header

    async def ComputeIK(self, target_pos: PoseMsg):

        # /compute_ik [moveit_msgs/srv/GetPositionIK]
        ik_req = PositionIKRequestMsg()
        ik_req.group_name = self.group_name
        ik_req.ik_link_name = self.EE_LINK_NAME

        ik_req.robot_state.joint_state = self.last_js

        # Give huge z axis tolerance, cuz it's a 5dof
        ik_req.constraints.orientation_constraints = [self.GetOrientationConstrain()]
        ik_req.constraints.position_constraints = [self.GetPosConstrain(target_pos)]

        response: GetPositionIKSrv.Response = await self.compute_ik_service_.call_async(
            GetPositionIKSrv.Request(ik_request=ik_req))

        sol = response.solution
        err = response.error_code

        if err != MoveItErrorCodes.SUCCESS:
            print(f"Planning finished with error: \n{err}")
            return

        print(f"Planning succeed!")
        print(sol)

    def GetOrientationConstrain(self, z_ang=0):

        # https://github.com/ros-planning/moveit_msgs/blob/0e203aaffd1f754ab3b31110eff0eaf26e1c3599/msg/OrientationConstraint.msg
        oc = MoveitOrientationConstraint()
        oc.header = self.GenEEHeader()

        # (roll about an X-axis)
        # (subsequent pitch about the Y-axis)
        # (subsequent yaw about the Z-axis)
        # We want the robot to be mostly flat into the switch, so no row or yaw.
        # Pitch should be from the line from robot to target.
        # TODO Who's xyz is Moveit's orientation constrain with? EE or base?
        # I think it's EE. regardless, our WX200 define X out, Y left, Z up.

        qw, qx, qy , qz = quaternion_from_euler(0, 0, z_ang)

        # The two angle we want flat will just get about 45 deg tolerance?
        # TODO consider lower these values
        oc.absolute_x_axis_tolerance = math.pi 
        oc.absolute_y_axis_tolerance = math.pi 
        oc.absolute_z_axis_tolerance = 2 * math.pi
        oc.orientation.w =qw
        oc.orientation.x =qx
        oc.orientation.y =qy
        oc.orientation.z =qz
        # orientation is really don't care much
        oc.weight = 0.4
        return oc

    def GetPosConstrain(self, pose: PoseMsg):
        # https://github.com/ros-planning/moveit_msgs/blob/ros2/msg/PositionConstraint.msg
        pc = MoveitPositionConstraint()
        pc.header = self.GenEEHeader()

        pc.link_name = self.EE_LINK_NAME
        bv = BoundingVolume()
        sp = SolidPrimitive()

        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [100 / 1000]  # 5mm of sphere toleranec
        bv.primitives = [sp]
        bv.primitive_poses = [pose]
        pc.constraint_region = bv
        return pc


def main():
    import rclpy
    rclpy.init()

    planer = EEPlaner()
    try:
        rclpy.spin(planer)
    except KeyboardInterrupt as e:
        import traceback
        print(f"Getting keyboard interrupt, with {e}")
        print(traceback.print_stack(limit=2))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
