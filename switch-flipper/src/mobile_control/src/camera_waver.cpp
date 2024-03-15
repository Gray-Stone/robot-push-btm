#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <rclcpp/utilities.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CameraWaver {
  public:
  CameraWaver(rclcpp::Node::SharedPtr node_ptr) : node_ptr(node_ptr) {

    // Listen to odom 
    // Timer to move camera 
    // Listen to pose estimate for other camera pointing.
  }

  void CameraTimer(){
    // Check odom for robot movement.

    // If robot moving, snap to front.

    // If no odom for a while, maybe move camera, maybe not.

    // First look down, low left, right.

    // Then second stage, look up, left, right.

    // Then 
    // Basically drawing X or infinity till instruction or moving again.

  }

  rclcpp::Node::SharedPtr node_ptr;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub_ ;
};

int main(int argc, char *argv[]){
  rclcpp::init(argc,argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "camera_waver",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  CameraWaver camera_waver{node};

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}