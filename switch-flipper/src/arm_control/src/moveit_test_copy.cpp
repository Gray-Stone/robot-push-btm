#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
using moveit::planning_interface::MoveGroupInterface;

template <class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  for (const auto &x : v) {
    os << '[' << x << ']';
  }
  return os;
}

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  sensor_msgs::msg::JointState js;
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");

  RCLCPP_INFO(logger, "PASSED interface construction");

  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.1;
    msg.position.y = 0.0;
    msg.position.z = 0.2803736261075846;
    return msg;
  }();

  RCLCPP_INFO(logger, "Get current robot state");

  // Print some basic robot info
  RCLCPP_ERROR(logger, "Basic robot info");
  RCLCPP_WARN_STREAM(logger, "getJoints\n" << move_group_interface.getJoints());
  RCLCPP_WARN_STREAM(logger, "getLinkNames\n"
                                 << move_group_interface.getLinkNames());
  RCLCPP_WARN_STREAM(logger,
                     "getCurrentJointValues\n"
                         << move_group_interface.getCurrentJointValues());

  RCLCPP_ERROR(logger, "More moveit specific info");
  //  move_group_interface.setPoseTarget(target_pose);
  std::string ee_link_name = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO_STREAM(logger, "ee_link_name: " << ee_link_name << std::endl);
  RCLCPP_WARN_STREAM(logger,
                     "ee name: " << move_group_interface.getEndEffector());

  auto pose_stamped =
      move_group_interface.getCurrentPose("wx200/ee_gripper_link");

  RCLCPP_WARN_STREAM(logger, "Current Pose stamped "
                                 << geometry_msgs::msg::to_yaml(pose_stamped));

  // ============= set the target

  // Try setApproximateJointValueTarget

  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setGoalPositionTolerance(0.5);

  // Actual planning.
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}