#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

using moveit::planning_interface::MoveGroupInterface;

namespace {
// Only can use std string until cpp20
constexpr int kDebugArrowId = 10;
} // namespace

class HammerMover {
public:
  HammerMover(rclcpp::Node::SharedPtr node_ptr)
      : node_ptr(node_ptr), group_name(node_ptr->get_parameter_or<std::string>(
                                "group_name", "interbotix_arm")),
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

    // From tutorial, not sure if needed.
    // rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    RCLCPP_INFO_STREAM(logger, "Using group " << group_name);
    RCLCPP_WARN_STREAM(logger, "xz_debug option: " << xz_debug);
    // some simple debug
    RCLCPP_INFO_STREAM(
        logger, "ee_link_name: " << move_group_interface.getEndEffectorLink());
    RCLCPP_WARN_STREAM(logger,
                       "ee name: " << move_group_interface.getEndEffector());
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
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    debug_marker_pub->publish(marker);

    // TODO try directly setting a link here.
    move_group_interface.setPoseTarget(target_pose);
    // move_group_interface.setGoalOrientationTolerance(1.0);
    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    auto const plan_success =
        static_cast<bool>(move_group_interface.plan(plan_msg));

    // Execute the plan
    if (plan_success) {
      RCLCPP_INFO(logger, "======== Planning SUCCEED !!!!! !");
      move_group_interface.execute(plan_msg);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  std::string group_name;
  bool xz_debug;
  rclcpp::Logger logger;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      debug_marker_pub;
  // Ros interface objects

  // Move group interface
  moveit::planning_interface::MoveGroupInterface move_group_interface;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>("hello_moveit");
  auto const hammer_mover = std::make_shared<HammerMover>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
