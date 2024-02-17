#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;

  std::string group_name =
      node->get_parameter_or<std::string>("group_name", "interbotix_arm");

  auto move_group_interface = MoveGroupInterface(node, group_name);
  // auto move_group_interface = MoveGroupInterface(node, "lollypop_hammer");

  // Getting current robot information
  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.22;
    msg.position.y = 0.00;
    msg.position.z = 0.2;
    return msg;
  }();
  // Directly setting lollypop_ee_link while using interbotix_arm group
  move_group_interface.setPoseTarget(target_pose, "lollypop_ee_link");

  move_group_interface.setMaxVelocityScalingFactor(1);
  move_group_interface.setMaxAccelerationScalingFactor(1);

  // move_group_interface.setPositionTarget(0.22,0.0,0.2 ,"lollypop_ee_link");
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    RCLCPP_INFO(logger, "======== Planning SUCCEED !!!!! !");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

// https://answers.ros.org/question/302283/failed-to-fetch-current-robot-state/
// This might be the reason this failed, ros needs to be spinning.
    move_group_interface.getCurrentState();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}