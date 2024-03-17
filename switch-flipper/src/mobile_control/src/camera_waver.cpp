#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>



#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <tf2/tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>


#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <rclcpp/utilities.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <rclcpp/utilities.hpp>

#include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class ListenerNode : public rclcpp::Node {
  public:
  ListenerNode(): Node("listener") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Let's try to do some dynamixal register fun
  }
  // This two thing only work if the owning class is a ros node 
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<geometry_msgs::msg::TransformStamped> GetTF(){
    try {
      auto map_base_tf = tf_buffer_->lookupTransform("wx200/base_link", "map", tf2::TimePointZero);
      return map_base_tf;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform map to wx200/base_link: " << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }

  }
  
};


class CameraWaver {
  public:
    CameraWaver(rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<ListenerNode> listener_node)
        : node_ptr(node_ptr), listener_node_(listener_node),
        logger(node_ptr->get_logger()) {

      // Listen to odom
      // Timer to move camera
      // Listen to pose estimate for other camera pointing.
      // /initialpose is the topic
      // geometry_msgs/msg/PoseWithCovarianceStamped
      joint_cmd_pub = node_ptr->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
          "/locobot/commands/joint_group", 10);

      pose_subs = node_ptr->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10, std::bind(&CameraWaver::PoseCB, this, std::placeholders::_1));
    }

  void PoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped& msg_in){
    geometry_msgs::msg::PoseWithCovarianceStamped msg_trans;
    auto map_base_tf = listener_node_->GetTF();
    if (map_base_tf) {
      // RCLCPP_INFO_STREAM(logger, "Got tf: " << geometry_msgs::msg::to_yaml(map_base_tf.value()));
      // RCLCPP_INFO_STREAM(logger, "Got point: " << geometry_msgs::msg::to_yaml(msg_in));
      tf2::doTransform(msg_in, msg_trans, map_base_tf.value());
      // RCLCPP_INFO_STREAM(logger, "Transformed Point: " << geometry_msgs::msg::to_yaml(msg_transformed));
      RCLCPP_INFO_STREAM(logger, "No Found TF");
      

    } else {
      RCLCPP_INFO_STREAM(logger, "No TF");
      return;
    }

    auto q = tf2::Quaternion(msg_trans.pose.pose.orientation.x, msg_trans.pose.pose.orientation.y,
                             msg_trans.pose.pose.orientation.z, msg_trans.pose.pose.orientation.w);


    double roll;
    double yaw;
    double pitch;

    // The fromMsg for quaternion can't be find by linker.
    tf2::getEulerYPR(q, yaw, pitch, roll);

    // yaw this is the angle 

    // if (std::abs(yaw) < 1.5) {
      interbotix_xs_msgs::msg::JointGroupCommand cmd;
      cmd.name = "camera";
      cmd.cmd = {float(yaw) , 0.2};
      joint_cmd_pub->publish(cmd);
      rclcpp::sleep_for(std::chrono::milliseconds{20});
      joint_cmd_pub->publish(cmd);
    // } else {
    //   RCLCPP_INFO_STREAM(logger,"TOO big a angle " << yaw);
    // }


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

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subs;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub_ ;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr joint_cmd_pub;

  rclcpp::Node::SharedPtr node_ptr;

  std::shared_ptr<ListenerNode> listener_node_;
  rclcpp::Logger logger;

};

int main(int argc, char *argv[]){
  rclcpp::init(argc,argv);

  auto listen_node = std::make_shared<ListenerNode>();

  auto const node = std::make_shared<rclcpp::Node>(
      "camera_waver",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  CameraWaver camera_waver{node , listen_node};


  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listen_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}