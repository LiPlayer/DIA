#include "trajectory_control/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "coord_utils.hpp"

using namespace std::chrono_literals;
using ego_planner::UniformBspline;

namespace trajectory_control {

TrajController::TrajController() : Node("controller") {
  this->declare_parameter("time_forward", 1.0);
  time_forward_ = this->get_parameter("time_forward").as_double();

  traj_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "/fmu/in/trajectory_setpoint", 10);

  offboard_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);

  vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

  bspline_sub_ = this->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline", 10,
      std::bind(&TrajController::bsplineCallback, this,
                std::placeholders::_1));

  cmd_timer_ = this->create_wall_timer(
      20ms, std::bind(&TrajController::cmdCallback, this));

  offboard_timer_ = this->create_wall_timer(
      100ms, std::bind(&TrajController::offboardCallback, this));

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "PX4 Trajectory Controller started.");
}

void TrajController::bsplineCallback(
    const traj_utils::msg::Bspline::SharedPtr msg) {
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  start_time_ = rclcpp::Time(msg->start_time);
  traj_id_ = msg->traj_id;
  receive_traj_ = true;

  RCLCPP_INFO(this->get_logger(), "Received trajectory %d, duration: %.2fs",
              traj_id_, traj_duration_);
}

void TrajController::cmdCallback() {
  if (!receive_traj_)
    return;

  auto time_now = this->now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos_enu, vel_enu, acc_enu;
  double yaw_enu = 0.0, yaw_dot_enu = 0.0;

  if (t_cur >= 0.0 && t_cur < traj_duration_) {
    pos_enu = traj_[0].evaluateDeBoorT(t_cur);
    vel_enu = traj_[1].evaluateDeBoorT(t_cur);
    acc_enu = traj_[2].evaluateDeBoorT(t_cur);

    auto [yaw, yaw_dot] = calculateYaw(t_cur, pos_enu, time_now);
    yaw_enu = yaw;
    yaw_dot_enu = yaw_dot;

  } else if (t_cur >= traj_duration_) {
    pos_enu = traj_[0].evaluateDeBoorT(traj_duration_);
    vel_enu.setZero();
    acc_enu.setZero();
    yaw_enu = last_yaw_;
    yaw_dot_enu = 0.0;
  } else {
    return;
  }

  time_last_ = time_now;

  Eigen::Vector3d pos_ned = enu_to_ned(pos_enu);
  Eigen::Vector3d vel_ned = enu_to_ned(vel_enu);
  Eigen::Vector3d acc_ned = enu_to_ned(acc_enu);

  double yaw_ned = enu_yaw_to_ned(yaw_enu);
  double yaw_dot_ned = enu_yawrate_to_ned(yaw_dot_enu);

  while (yaw_ned > M_PI)
    yaw_ned -= 2.0 * M_PI;
  while (yaw_ned < -M_PI)
    yaw_ned += 2.0 * M_PI;

  px4_msgs::msg::TrajectorySetpoint setpoint;
  setpoint.timestamp = this->now().nanoseconds() / 1000;

  setpoint.position[0] = pos_ned.x();
  setpoint.position[1] = pos_ned.y();
  setpoint.position[2] = pos_ned.z();

  setpoint.velocity[0] = vel_ned.x();
  setpoint.velocity[1] = vel_ned.y();
  setpoint.velocity[2] = vel_ned.z();

  setpoint.acceleration[0] = acc_ned.x();
  setpoint.acceleration[1] = acc_ned.y();
  setpoint.acceleration[2] = acc_ned.z();

  setpoint.yaw = yaw_ned;
  setpoint.yawspeed = yaw_dot_ned;

  traj_setpoint_pub_->publish(setpoint);
}

void TrajController::offboardCallback() {
  px4_msgs::msg::OffboardControlMode mode;
  mode.timestamp = this->now().nanoseconds() / 1000;
  mode.position = true;
  mode.velocity = true;
  mode.acceleration = true;
  mode.attitude = false;
  mode.body_rate = false;

  offboard_mode_pub_->publish(mode);
}

std::pair<double, double>
TrajController::calculateYaw(double t_cur, const Eigen::Vector3d &pos,
                             const rclcpp::Time &time_now) {
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX = PI;

  double dt = (time_now - time_last_).seconds();
  if (dt < 1e-6)
    dt = 0.02;

  double t_ahead = std::min(t_cur + time_forward_, traj_duration_);
  Eigen::Vector3d dir = traj_[0].evaluateDeBoorT(t_ahead) - pos;

  double yaw_target =
      dir.norm() > 0.1 ? std::atan2(dir.y(), dir.x()) : last_yaw_;
  double max_change = YAW_DOT_MAX * dt;

  double yaw_diff = yaw_target - last_yaw_;

  while (yaw_diff > PI)
    yaw_diff -= 2.0 * PI;
  while (yaw_diff < -PI)
    yaw_diff += 2.0 * PI;

  double yaw, yaw_dot;
  if (std::abs(yaw_diff) > max_change) {
    yaw = last_yaw_ + std::copysign(max_change, yaw_diff);
    yaw_dot = std::copysign(YAW_DOT_MAX, yaw_diff);
  } else {
    yaw = yaw_target;
    yaw_dot = yaw_diff / dt;
  }

  while (yaw > PI)
    yaw -= 2.0 * PI;
  while (yaw < -PI)
    yaw += 2.0 * PI;

  yaw = 0.5 * last_yaw_ + 0.5 * yaw;
  yaw_dot = 0.5 * last_yaw_dot_ + 0.5 * yaw_dot;

  last_yaw_ = yaw;
  last_yaw_dot_ = yaw_dot;

  return {yaw, yaw_dot};
}

} // namespace trajectory_control
