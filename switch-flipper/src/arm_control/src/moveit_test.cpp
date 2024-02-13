#include <memory>
#include <rclcpp/wait_for_message.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
using moveit::planning_interface::MoveGroupInterface;
int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  sensor_msgs::msg::JointState js ;
  bool get_msg = rclcpp::wait_for_message(js,node , "/joint_states" , std::chrono::seconds{10});
  if (get_msg) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Got message \n");

  } else {
    RCLCPP_WARN_STREAM(node->get_logger(), " DId not get msg\n");
  }
  // [moveit_test-8] [WARN] [1707859549.223983825] [hello_moveit]: Got message
  // [moveit_test-8]
  // [moveit_test-8] [WARN] [1707859549.224296271] [hello_moveit]:  JS header:
  // [moveit_test-8]   stamp:
  // [moveit_test-8]     sec: 1707859549

  RCLCPP_WARN_STREAM(node->get_logger(),
                     " JS " << sensor_msgs::msg::to_yaml(js));

  // [moveit_test-8] Got message
  // [moveit_test-8] [WARN] [1707859138.723431678] [hello_moveit]:
  // time1707859138 joint size8
  // TODO can't get current state
  // [moveit_test-8] [INFO] [1707859139.841502389]
  // [moveit_ros.current_state_monitor]: Didn't receive robot state (joint
  // angles) with recent timestamp within 1.000000 seconds. Requested time
  // 1707859138.841289, but latest received state has time 0.000000.


  // std::cout<< js.header.stamp.

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");

  RCLCPP_INFO(logger, "PASSED interface construction");

  // mid_air_pos.pose.position.x = 0.2179430539986765
  // mid_air_pos.pose.position.y = 0.0016716351397863933
  // mid_air_pos.pose.position.z = 0.15803736261075846

  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.position.x = 0.21;
    msg.position.y = 0.0;
    msg.position.z = 0.15803736261075846;
    return msg;
  }();


    RCLCPP_INFO(logger, "Get current robot state");

  moveit::core::RobotStatePtr robot_state_p = move_group_interface.getCurrentState(10.5);

  move_group_interface.setStartState()
  robot_state_p->printStateInfo();

  //  move_group_interface.setPoseTarget(target_pose);
  std::string ee_link_name = move_group_interface.getEndEffectorLink();
  std::cout << "ee_link_name " << ee_link_name << std::endl;
  auto pose_stamped =
      move_group_interface.getCurrentPose("wx200/ee_gripper_link");

  pose_stamped.pose;
  std::cout << "position.x" << pose_stamped.pose.position.x << "\n";
  std::cout << "position.y" << pose_stamped.pose.position.y << "\n";
  std::cout << "position.z" << pose_stamped.pose.position.z << "\n";
  std::cout << "orientation.w" << pose_stamped.pose.orientation.w << "\n";
  std::cout << "orientation.x" << pose_stamped.pose.orientation.x << "\n";
  std::cout << "orientation.y" << pose_stamped.pose.orientation.y << "\n";
  std::cout << "orientation.z" << pose_stamped.pose.orientation.z << "\n";

  move_group_interface.setPoseTarget(pose_stamped.pose);

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
  rclcpp::shutdown();
  return 0;
}