#! /usr/bin/env python3

from rclpy.node import Node as RosNode
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from rclpy.wait_for_message import wait_for_message
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Header
from geometry_msgs.msg import Pose as PoseMsg ,PoseStamped

from moveit_msgs.srv import GetPositionIK as GetPositionIKSrv
from moveit_msgs.msg import PositionIKRequest as PositionIKRequestMsg

from moveit_msgs.msg import RobotState as RobotStateMsg
from moveit_msgs.msg import Constraints as MoveitConstrains
from moveit_msgs.msg import OrientationConstraint as MoveitOrientationConstraint
from moveit_msgs.msg import PositionConstraint as MoveitPositionConstraint

from moveit_msgs.msg import BoundingVolume, MoveItErrorCodes ,MotionPlanRequest, PlanningOptions
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

import math


class EEPlaner(RosNode):

    # EE_LINK_NAME = "lollypop_ee_link"
    EE_LINK_NAME = "wx200/ee_gripper_link"
    # BASE_LINK = 'wx200/base_link'
    BASE_LINK = 'world'

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


        self.compute_ik_service_ = self.create_client(
            GetPositionIKSrv,
            "compute_ik",
        )
        self.move_action_client_ = ActionClient(
            self,
            MoveGroup,
            "move_action",
        )


        self._tf_cal_timer = self.create_timer(1, self.IkCmdTimer, self._timer_callback_group)


    async def IkCmdTimer(self):
        # Home pose for lollypop_ee_link
        # x: 0.436 y: 0.0 z: 0.31065
        # Good mid air pos

        # x: 0.23154 y: 0 z: 0.27571
        # 0.2179430539986765 y: 0.0016716351397863933 z: 0.15803736261075846
        mid_air_pos = PoseStamped()
        mid_air_pos.pose.position.x = 0.2179430539986765
        mid_air_pos.pose.position.y = 0.0016716351397863933
        mid_air_pos.pose.position.z = 0.15803736261075846
        # mid_air_pos.pose.orientation.x = mid_air_pos.pose.orientation.x + 1e-10
        # mid_air_pos.pose.orientation.y = mid_air_pos.pose.orientation.y + 1e-10
        # mid_air_pos.pose.orientation.z = mid_air_pos.pose.orientation.z + 1e-10
        # mid_air_pos.pose.orientation.w = mid_air_pos.pose.orientation.w + 1e-10

        # mid_air_pos.header.frame_id = self.BASE_LINK
        # mid_air_pos.header.stamp = self.get_clock().now().to_msg()
        mid_air_pos.header = self.GenBaseHeader()
        await self.ComputeIK(mid_air_pos)
        # await self.MoveGroupPlan(mid_air_pos)

    def JointStateCB(self, msg: JointState):
        self.last_js = msg

    def GenBaseHeader(self):
        header = Header()
        header.frame_id = self.BASE_LINK
        header.stamp = self.get_clock().now().to_msg()
        return header

    async def MoveGroupPlan(self ,target_pos: PoseStamped ):


        move_req = MotionPlanRequest()
        move_req.start_state.joint_state = self.last_js

        # Do a big workspace
        move_req.workspace_parameters.header = self.GenBaseHeader()
        move_req.workspace_parameters.min_corner.x = -1.0
        move_req.workspace_parameters.min_corner.y = -1.0
        move_req.workspace_parameters.min_corner.z = -1.0
        move_req.workspace_parameters.max_corner.x = +1.0
        move_req.workspace_parameters.max_corner.y = +1.0
        move_req.workspace_parameters.max_corner.z = +1.0

        move_req.goal_constraints = [self.MakeConstrain(target_pos)]

        move_req.group_name = self.group_name
        move_req.num_planning_attempts = 5
        move_req.allowed_planning_time = 3.0

        plan_opt = PlanningOptions()
        plan_opt.plan_only = True
        move_goal = MoveGroup.Goal(request = move_req,
                                    planning_options=plan_opt);
        print("============================ move goal")
        print(move_goal)

        goal_handle = await self.move_action_client_.send_goal_async(move_goal)

        if not goal_handle.accepted:
            print("Goal not accepted")
            return
        print("Goal accepted.")

        res = await goal_handle.get_result_async()
        result = res.result
        status = res.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Goal succeeded!")
        else:
            print("Goal failed with status: {0}".format(status))

        print("======= Status")
        print(status)
        print("======= Result")
        print(result)


        print("\n=======\n\n\n ")

    def MakeConstrain(self,pose_stamped:PoseStamped)->MoveitConstrains:
        cons = MoveitConstrains()
        cons.name = self.EE_LINK_NAME
        cons.orientation_constraints = [self.GetOrientationConstrain(pose_stamped)]
        cons.position_constraints = [self.GetPosConstrain(pose_stamped)]



        return cons


    async def ComputeIK(self, target_pos: PoseStamped):

        # /compute_ik [moveit_msgs/srv/GetPositionIK]
        ik_req = PositionIKRequestMsg()
        ik_req.group_name = self.group_name
        # ik_req.ik_link_name = self.EE_LINK_NAME

        ik_req.robot_state.joint_state = self.last_js

        print("==================== Robot state")
        print(ik_req.robot_state)
        # Give huge z axis tolerance, cuz it's a 5dof
        ik_req.constraints.name = self.EE_LINK_NAME
        # ik_req.constraints.orientation_constraints = [self.GetOrientationConstrain(target_pos)]
        ik_req.constraints.position_constraints = [self.GetPosConstrain(target_pos)]
        ik_req.avoid_collisions=False
        response: GetPositionIKSrv.Response = await self.compute_ik_service_.call_async(
            GetPositionIKSrv.Request(ik_request=ik_req))

        ik_req.pose_stamped = target_pos
        sol = response.solution
        err = response.error_code

        print("==================== ik req")
        print(ik_req)
        if err != MoveItErrorCodes.SUCCESS:
            print(f"Planning finished with error: \n{err}")
            return

        print(f"Planning succeed!")
        print(sol)

        self._tf_cal_timer.cancel()

    def GetOrientationConstrain(self ,pose: PoseStamped):

        # https://github.com/ros-planning/moveit_msgs/blob/0e203aaffd1f754ab3b31110eff0eaf26e1c3599/msg/OrientationConstraint.msg
        oc = MoveitOrientationConstraint()
        # oc.header = self.GenEEHeader()
        oc.header = pose.header
        oc.link_name = self.EE_LINK_NAME

        # (roll about an X-axis)
        # (subsequent pitch about the Y-axis)
        # (subsequent yaw about the Z-axis)
        # We want the robot to be mostly flat into the switch, so no row or yaw.
        # Pitch should be from the line from robot to target.
        # TODO Who's xyz is Moveit's orientation constrain with? EE or base?
        # I think it's EE. regardless, our WX200 define X out, Y left, Z up.

        z_ang = math.atan2(pose.pose.position.y,pose.pose.position.x)
        print(f"Doing Z ang {z_ang}")
        qx, qy , qz,qw = quaternion_from_euler(0, 0, z_ang)

        # The two angle we want flat will just get about 45 deg tolerance?
        # TODO consider lower these values
        oc.absolute_x_axis_tolerance = math.pi
        oc.absolute_y_axis_tolerance = math.pi
        oc.absolute_z_axis_tolerance = math.pi
        oc.orientation.w =qw
        oc.orientation.x =qx
        oc.orientation.y =qy
        oc.orientation.z =qz
        oc.parameterization = MoveitOrientationConstraint.XYZ_EULER_ANGLES
        # orientation is really don't care much
        oc.weight = 1.0
        return oc

    def GetPosConstrain(self, pose: PoseStamped):
        # https://github.com/ros-planning/moveit_msgs/blob/ros2/msg/PositionConstraint.msg
        pc = MoveitPositionConstraint()
        pc.header = pose.header
        pc.link_name = self.EE_LINK_NAME
        bv = BoundingVolume()
        sp = SolidPrimitive()

        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [200 / 1000.0]  # 5mm of sphere toleranec
        bv.primitives = [sp]
        bv.primitive_poses = [pose.pose]
        pc.constraint_region = bv
        pc.weight= 1.0
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
