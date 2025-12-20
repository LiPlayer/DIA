#include "trajectory_control/vio_bridge.hpp"

#include <functional>

namespace trajectory_control {

VioBridge::VioBridge() : Node("vio_bridge") {
  this->declare_parameter("odom_topic", "/odometry/imu");
  std::string odom_topic = this->get_parameter("odom_topic").as_string();

  visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(10).best_effort());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&VioBridge::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
      this->get_logger(),
      "VIO Bridge started. Subscribing to odometry: %s",
      odom_topic.c_str());
}

void VioBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const uint64_t timestamp = this->now().nanoseconds() / 1000;

  px4_msgs::msg::VehicleOdometry report{};
  report.timestamp_sample = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000;
  report.timestamp = timestamp;

  report.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  report.position[0] = msg->pose.pose.position.y;
  report.position[1] = msg->pose.pose.position.x;
  report.position[2] = -msg->pose.pose.position.z;

  const auto &pose_orientation = msg->pose.pose.orientation;
  Eigen::Quaterniond q_flu_to_enu(pose_orientation.w, pose_orientation.x,
                                 pose_orientation.y, pose_orientation.z);
  Eigen::Quaterniond q_frd_to_ned;
  rotateQuaternion(q_frd_to_ned, q_flu_to_enu);
  q_frd_to_ned.normalize();
  report.q[0] = q_frd_to_ned.w();
  report.q[1] = q_frd_to_ned.x();
  report.q[2] = q_frd_to_ned.y();
  report.q[3] = q_frd_to_ned.z();

  report.velocity_frame =
      px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  report.velocity[0] = msg->twist.twist.linear.x;
  report.velocity[1] = -msg->twist.twist.linear.y;
  report.velocity[2] = -msg->twist.twist.linear.z;

  report.angular_velocity[0] = msg->twist.twist.angular.x;
  report.angular_velocity[1] = -msg->twist.twist.angular.y;
  report.angular_velocity[2] = -msg->twist.twist.angular.z;

  const auto &pose_cov = msg->pose.covariance;
  const auto &twist_cov = msg->twist.covariance;

  report.position_variance = {
      static_cast<float>(pose_cov[7]),
      static_cast<float>(pose_cov[0]),
      static_cast<float>(pose_cov[14]),
  };
  report.orientation_variance = {
      static_cast<float>(pose_cov[21]),
      static_cast<float>(pose_cov[28]),
      static_cast<float>(pose_cov[35]),
  };
  report.velocity_variance = {
      static_cast<float>(twist_cov[7]),
      static_cast<float>(twist_cov[0]),
      static_cast<float>(twist_cov[14]),
  };

  report.quality = 100;

  visual_odom_pub_->publish(report);
}

void VioBridge::rotateQuaternion(Eigen::Quaterniond &q_frd_to_ned,
                                 const Eigen::Quaterniond &q_flu_to_enu) {
  static const Eigen::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
  static const Eigen::Quaterniond q_enu_to_ned(0.0, 0.70711, 0.70711, 0.0);

  q_frd_to_ned = q_enu_to_ned * q_flu_to_enu * q_flu_to_frd.conjugate();
}

} // namespace trajectory_control
