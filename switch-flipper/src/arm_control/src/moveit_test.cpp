#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");
// auto move_group_interface = MoveGroupInterface(node, "lollypop_hammer");

// Getting current robot information

RCLCPP_WARN_STREAM(
    logger, "Current Pose stamped " << geometry_msgs::msg::to_yaml(
                move_group_interface.getCurrentPose()));
RCLCPP_INFO_STREAM(logger, "ee_link_name: " << move_group_interface.getEndEffectorLink());
RCLCPP_WARN_STREAM(logger, "ee name: " << move_group_interface.getEndEffector());

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
    msg.position.x = 0.22;
    msg.position.y = 0.0;
    msg.position.z = 0.2;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}