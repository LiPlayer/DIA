#pragma once

#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/msg/bspline.hpp>

#include <utility>
#include <vector>

namespace trajectory_control {

class TrajController : public rclcpp::Node {
public:
  TrajController();

private:
  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg);
  void cmdCallback();
  void offboardCallback();
  std::pair<double, double> calculateYaw(double t_cur,
                                         const Eigen::Vector3d &pos,
                                         const rclcpp::Time &time_now);

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_cmd_pub_;

  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;

  bool receive_traj_ = false;
  std::vector<ego_planner::UniformBspline> traj_;
  double traj_duration_ = 0.0;
  rclcpp::Time start_time_;
  rclcpp::Time time_last_;
  int traj_id_ = 0;

  double last_yaw_ = 0.0;
  double last_yaw_dot_ = 0.0;
  double time_forward_ = 1.0;
};

} // namespace trajectory_control
