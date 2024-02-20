#include <chrono>
#include <interbotix_xs_msgs/msg/detail/joint_group_command__traits.hpp>
#include <interbotix_xs_msgs/srv/detail/motor_gains__struct.hpp>
#include <interbotix_xs_msgs/srv/detail/register_values__struct.hpp>
#include <interbotix_xs_msgs/srv/detail/robot_info__struct.hpp>
#include <memory>

#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <interbotix_xs_msgs/srv/register_values.hpp>
#include <interbotix_xs_msgs/srv/motor_gains.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include "interbotix_xs_msgs/Reboot.h"

using moveit::planning_interface::MoveGroupInterface;

namespace {
// Only can use std string until cpp20
constexpr int kDebugArrowId = 10;

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
{
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " " << *ii;
    }
    os << "]";
    return os;
}

} // namespace

class HammerMover {
public:
  HammerMover(rclcpp::Node::SharedPtr node_ptr)
      : node_ptr(node_ptr),
        // Using this kind of params need automatically declare params
        group_name(node_ptr->get_parameter_or<std::string>("group_name",
                                                           "interbotix_arm")),
        ee_link_name(node_ptr->get_parameter_or<std::string>(
            "ee_link_name", "wx200/ee_gripper_link")),
        xz_debug(node_ptr->get_parameter_or<bool>("xz_debug", false)),
        logger(node_ptr->get_logger()),
        // I can't use *this when making the interface. So not doing inheriting.
        move_group_interface(MoveGroupInterface(node_ptr, group_name))

  {

    point_subscriber =
        node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&HammerMover::ClickedPointCB, this,
                      std::placeholders::_1));

    debug_marker_pub =
        node_ptr->create_publisher<visualization_msgs::msg::Marker>("debug",
                                                                    10);

    joint_cmd_pub =
        node_ptr->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
            "/wx200/commands/joint_group", 10);

    // From tutorial, not sure if needed.
    // rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    RCLCPP_INFO_STREAM(logger, "Using group: " << group_name);
    RCLCPP_INFO_STREAM(logger, "Planning with ee link: " << ee_link_name);
    RCLCPP_WARN_STREAM(logger, "xz_debug option: " << xz_debug);
    // some simple debug
    RCLCPP_INFO_STREAM(logger,
                       "planing group ee_link_name: "
                           << move_group_interface.getEndEffectorLink());
    RCLCPP_WARN_STREAM(logger, "planing group ee name: "
                                   << move_group_interface.getEndEffector());




    // Let's try to do some dynamixal register fun

    get_motor_reg_cli =
        node_ptr->create_client<interbotix_xs_msgs::srv::RegisterValues>(
            "/wx200/get_motor_registers", 10);
    auto set_motor_pid_cli =
        node_ptr->create_client<interbotix_xs_msgs::srv::MotorGains>(
            "/wx200/set_motor_pid_gains", 10);

    // let's first print some infos

// string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
// string name              # name of the group if commanding a joint group or joint if commanding a single joint
// string reg               # register name (like Profile_Velocity, Profile_Acceleration, etc...)
// int32 value              # desired register value (only set if 'setting' a register)
// ---
// int32[] values           # current register values (only filled if 'getting' a register)

// To find reg name https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/master/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp#L1398

// Basically the name we sent, are looked up from a big table to become register numbers.
// The table changes depend on the motor used. wx200 use are xm430-w350 (except gripper)
// So after some nasty jumping around in code, we do get the register table

  while (!set_motor_pid_cli->wait_for_service(std::chrono::seconds{1})) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(logger, "service not available, waiting again...");
  }

  // This is expected, In data sheet, we have 7 motors, But J2 is doubled. So
  // the first 4 number match what we see with interbotix default

  // clang-format off
// [hammer_to_point-9] [INFO]: Getting reg value for Velocity_I_Gain
// [hammer_to_point-9] [INFO]: [ 1920 1920 1920 1920 1000]
// [hammer_to_point-9] [INFO]: Getting reg value for Velocity_P_Gain
// [hammer_to_point-9] [INFO]: [ 100 100 100 100 100]
// [hammer_to_point-9] [INFO]: Getting reg value for Position_D_Gain
// [hammer_to_point-9] [INFO]: [ 0 0 0 0 3600]
// [hammer_to_point-9] [INFO]: Getting reg value for Position_I_Gain
// [hammer_to_point-9] [INFO]: [ 0 0 0 0 0]
// [hammer_to_point-9] [INFO]: Getting reg value for Position_P_Gain
// [hammer_to_point-9] [INFO]: [ 800 800 800 800 640]
  // clang-format on

  // reg_req.
  // GetDynamixelReg("Velocity_I_Gain");
  // GetDynamixelReg("Velocity_P_Gain");
  // GetDynamixelReg("Position_D_Gain");
  // GetDynamixelReg("Position_I_Gain");
  // GetDynamixelReg("Position_P_Gain");
  
  SetMotorPID("waist", 1920, 100, 200);
  SetMotorPID("shoulder", 1920, 100, 200);
  SetMotorPID("elbow", 1920, 100, 200);
  SetMotorPID("wrist_angle", 1920, 100, 200);

  // GetDynamixelReg("Velocity_I_Gain");
  // GetDynamixelReg("Velocity_P_Gain");
  // GetDynamixelReg("Position_D_Gain");
  // GetDynamixelReg("Position_I_Gain");
  // GetDynamixelReg("Position_P_Gain");
  };

private:
  // This is actually our main logic here. for now.
  void ClickedPointCB(const geometry_msgs::msg::PointStamped &msg) {

    // TODO consider do a user input queue in future.
    RCLCPP_INFO_STREAM(logger, "Received User Command :\n"
                                   << geometry_msgs::msg::to_yaml(msg));

    // Move the robot to this point.

    // Set the xyz target
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg.point;

    if (xz_debug) {

      target_pose.position.y = 0.0;
    }

    // Set the rotation target
    RCLCPP_INFO_STREAM(logger, "target y " << target_pose.position.y
                                           << " target x "
                                           << target_pose.position.x);
    double z_angle = std::atan2(target_pose.position.y, target_pose.position.x);

    tf2::Quaternion target_q;
    // Here is the magic, the description of setEuler is mis-leading!
    target_q.setEuler(0.0, 0.0, z_angle);

    target_pose.orientation.w = target_q.w();
    target_pose.orientation.x = target_q.x();
    target_pose.orientation.y = target_q.y();
    target_pose.orientation.z = target_q.z();

    RCLCPP_INFO_STREAM(logger, "Computed target angle: " << z_angle);
    RCLCPP_INFO_STREAM(logger, "Planning target pose: \n"
                                   << geometry_msgs::msg::to_yaml(target_pose));

    // Publish a arrow to show and debug the quaternion stuff. Very Very useful!
    visualization_msgs::msg::Marker marker;
    marker.type = marker.ARROW;
    marker.header.stamp = node_ptr->get_clock()->now();
    marker.header.frame_id = "/world";
    marker.action = 0;
    marker.id = kDebugArrowId;
    marker.pose = target_pose;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    debug_marker_pub->publish(marker);

    // TODO try directly setting a link here.
    move_group_interface.setPoseTarget(target_pose, ee_link_name);
    // move_group_interface.setMaxVelocityScalingFactor(1);
    move_group_interface.setMaxAccelerationScalingFactor(1);

    // move_group_interface.setGoalOrientationTolerance(1.0);
    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    auto const plan_success =
        static_cast<bool>(move_group_interface.plan(plan_msg));

    // Execute the plan
    if (plan_success) {
      RCLCPP_INFO(logger, "======== Planning SUCCEED !!!!! !");

      std::cout << trajectory_msgs::msg::to_yaml(
          plan_msg.trajectory.joint_trajectory);
      // This is the name list
      std::vector<std::string> j_names =
          plan_msg.trajectory.joint_trajectory.joint_names;

      // This give the last joint trajectory point object.
      std::vector<double> final_js =
          plan_msg.trajectory.joint_trajectory.points.back().positions;




      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> reduced_points ;
      trajectory_msgs::msg::JointTrajectoryPoint last_point;
      int skip_every = 9;
      int count =0;
      // robot actually check trajectory starting point.
      reduced_points.push_back(plan_msg.trajectory.joint_trajectory.points.front());
      for (auto point : plan_msg.trajectory.joint_trajectory.points){
        last_point = point;
        if ((++count) < skip_every){
          continue;
        }

        count =0 ;
        reduced_points.push_back(last_point);
      }
      reduced_points.push_back(last_point);

      if (use_moveit_execute) {
        // Let's try to reduce waypoints.
        
        plan_msg.trajectory.joint_trajectory.points = reduced_points;
        move_group_interface.execute(plan_msg);



      } else {
        
        // Let's step through the commands ourself
        // Find the cmd intervel 

        
        std::chrono::nanoseconds last_time{0};
        for (auto cmd_point : plan_msg.trajectory.joint_trajectory.points) {
          std::chrono::nanoseconds new_time{
              uint64_t(cmd_point.time_from_start.nanosec +
                        cmd_point.time_from_start.sec * 1e9)};

          RCLCPP_WARN_STREAM(logger, "Sleep "
                                         << (new_time - last_time
                                         ).count() / 1e6
                                         << "ms before sending");
          rclcpp::sleep_for(new_time - last_time);
          last_time = new_time;
          auto jg_cmd = GenArmCmd(cmd_point);
          joint_cmd_pub->publish(jg_cmd);
        }

        // RCLCPP_WARN(logger,"Sending custom arm moving msg");
        // RCLCPP_WARN_STREAM(logger,
        //                    "" << interbotix_xs_msgs::msg::to_yaml(jg_cmd));

        // joint_cmd_pub->publish(jg_cmd);
      }

    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  interbotix_xs_msgs::msg::JointGroupCommand
  GenArmCmd(trajectory_msgs::msg::JointTrajectoryPoint point) {

    interbotix_xs_msgs::msg::JointGroupCommand jg_cmd;
    jg_cmd.name = "arm";
    for (auto j : point.positions){
      jg_cmd.cmd.push_back(j);
    }
    return jg_cmd;
  }

  void GetDynamixelReg(std::string reg_name) {

  // clang-format off
  // string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
  // string name              # name of the group if commanding a joint group or joint if commanding a single joint
  // string reg               # register name (like Profile_Velocity, Profile_Acceleration, etc...)
  // int32 value              # desired register value (only set if 'setting' a register)
  // ---
  // int32[] values           # current register values (only filled if 'getting' a register)
  // clang-format on

    auto reg_req =
        std::make_shared<interbotix_xs_msgs::srv::RegisterValues_Request>();
    reg_req->cmd_type = "group";
    reg_req->name = "arm";
    reg_req->reg = reg_name;

    RCLCPP_INFO_STREAM(logger, "Getting reg value for " << reg_name);

    auto result = get_motor_reg_cli->async_send_request(reg_req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_ptr, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(logger, ""<< result.get()->values);
    } else {
      RCLCPP_ERROR(logger, "Service call failed");
    }
  }
  void SetMotorPID(const std::string & joint_name , int32_t p, int32_t i, int32_t d) {

    // clang-format off
    // string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
    // string name              # name of the group if commanding a joint group or joint if commanding a single joint
    // int32 kp_pos             # acts as a pass-through to the Position_P_Gain register
    // int32 ki_pos             # acts as a pass-through to the Position_I_Gain register
    // int32 kd_pos             # acts as a pass-through to the Position_D_Gain register
    // int32 k1                 # acts as a pass-through to the Feedforward_1st_Gain register
    // int32 k2                 # acts as a pass-through to the Feedforward_2nd_Gain register
    // int32 kp_vel             # acts as a pass-through to the Velocity_P_Gain register
    // int32 ki_vel             # acts as a pass-through to the Velocity_I_Gain register
    // ---

    // clang-format on

    auto pid_req =
        std::make_shared<interbotix_xs_msgs::srv::MotorGains_Request>();

    pid_req->cmd_type = "single";
    pid_req->name = joint_name;
    pid_req->kp_pos = p;
    pid_req->ki_pos = i;
    pid_req->kd_pos = d;
    pid_req->k1 = 0;
    pid_req->k2 = 0;
    pid_req->kp_vel = 100;
    pid_req->ki_vel = 1920;

    RCLCPP_INFO_STREAM(logger, "Setting PID to " << joint_name);

    auto result = set_motor_pid_cli->async_send_request(pid_req);
    // Wait for the result.
    try{

    if (rclcpp::spin_until_future_complete(node_ptr, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(logger, "Setting successful");
    } else {
      RCLCPP_ERROR(logger, "Service call failed");
    }
    } catch (const std::exception& e){
      std::cout<<"ROS exception! " <<e.what();
    }
  }

  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  std::string group_name;
  std::string ee_link_name;
  bool xz_debug;
  rclcpp::Logger logger;
  // when false, will use interbotix JointGroupCommand
  bool use_moveit_execute =true;

  // Ros interface objects
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      debug_marker_pub;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr
      joint_cmd_pub;
rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr get_motor_reg_cli;
rclcpp::Client<interbotix_xs_msgs::srv::MotorGains>::SharedPtr set_motor_pid_cli;

  // Move group interface
  moveit::planning_interface::MoveGroupInterface move_group_interface;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "hammer_to_point",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  auto const hammer_mover = std::make_shared<HammerMover>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
