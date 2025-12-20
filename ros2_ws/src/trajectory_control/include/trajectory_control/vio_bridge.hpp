#pragma once

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace trajectory_control {

class VioBridge : public rclcpp::Node {
public:
  VioBridge();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  static void rotateQuaternion(Eigen::Quaterniond &q_frd_to_ned,
                               const Eigen::Quaterniond &q_flu_to_enu);

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      visual_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace trajectory_control
