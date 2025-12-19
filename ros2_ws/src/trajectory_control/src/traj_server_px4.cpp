/**
 * PX4 Trajectory Server
 *
 * Subscribes to B-spline trajectory from ego-planner and publishes
 * TrajectorySetpoint to PX4 via Micro XRCE-DDS.
 *
 * Implements VIO Bridge:
 * - Subscribes to LIO-SAM odometry (ENU)
 * - Converts to PX4 Visual Odometry (NED/FRD)
 * - Publishes to /fmu/in/vehicle_visual_odometry for EKF2 fusion
 */

#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <chrono>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <vector>

#include "coord_utils.hpp"

using namespace std::chrono_literals;
using namespace trajectory_control;
using ego_planner::UniformBspline;

class TrajServerPX4 : public rclcpp::Node {
public:
  TrajServerPX4() : Node("traj_server_px4") {
    // Parameters
    this->declare_parameter("time_forward", 1.0);
    this->declare_parameter("odom_topic", "lio_sam/mapping/odometry");
    time_forward_ = this->get_parameter("time_forward").as_double();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    // Publishers - PX4 topics
    traj_setpoint_pub_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

    offboard_mode_pub_ =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // VIO Publisher - feeds LIO odometry to PX4 EKF2
    visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(10).best_effort());

    // Subscriber - ego-planner bspline
    bspline_sub_ = this->create_subscription<traj_utils::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&TrajServerPX4::bsplineCallback, this,
                  std::placeholders::_1));

    // Subscriber - LIO-SAM odometry (for VIO bridge)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&TrajServerPX4::odomCallback, this, std::placeholders::_1));

    // Timer for trajectory evaluation and publishing (50 Hz)
    cmd_timer_ = this->create_wall_timer(
        20ms, std::bind(&TrajServerPX4::cmdCallback, this));

    // Timer for offboard control mode (must be >2Hz for PX4)
    offboard_timer_ = this->create_wall_timer(
        100ms, std::bind(&TrajServerPX4::offboardCallback, this));

    // Initialize state
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    RCLCPP_INFO(
        this->get_logger(),
        "PX4 Trajectory Server + VIO Bridge started. Subscribing to: %s",
        odom_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const uint64_t timestamp = this->now().nanoseconds() / 1000;

    px4_msgs::msg::VehicleOdometry report{};
    report.timestamp_sample = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000;
    report.timestamp = timestamp;

    // odometry position is in ENU frame and needs to be converted to NED
    report.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    report.position[0] = msg->pose.pose.position.y;
    report.position[1] = msg->pose.pose.position.x;
    report.position[2] = -msg->pose.pose.position.z;

    // odometry orientation is "body FLU->ENU" and needs to be converted in
    // "body FRD->NED"
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

    // odometry linear velocity is in body FLU and needs to be converted in
    // body FRD
    report.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    report.velocity[0] = msg->twist.twist.linear.x;
    report.velocity[1] = -msg->twist.twist.linear.y;
    report.velocity[2] = -msg->twist.twist.linear.z;

    // odometry angular velocity is in body FLU and need to be converted in
    // body FRD
    report.angular_velocity[0] = msg->twist.twist.angular.x;
    report.angular_velocity[1] = -msg->twist.twist.angular.y;
    report.angular_velocity[2] = -msg->twist.twist.angular.z;

    // Variances: follow GZBridge mapping.
    const auto &pose_cov = msg->pose.covariance;
    const auto &twist_cov = msg->twist.covariance;

    // VISION_POSITION_ESTIMATE covariance (x, y, z, roll, pitch, yaw).
    report.position_variance = {
        static_cast<float>(pose_cov[7]),  // Y -> X_ned
        static_cast<float>(pose_cov[0]),  // X -> Y_ned
        static_cast<float>(pose_cov[14]), // Z
    };
    report.orientation_variance = {
        static_cast<float>(pose_cov[21]), // roll
        static_cast<float>(pose_cov[28]), // pitch
        static_cast<float>(pose_cov[35]), // yaw
    };
    report.velocity_variance = {
        static_cast<float>(twist_cov[7]),  // Y -> X_ned
        static_cast<float>(twist_cov[0]),  // X -> Y_ned
        static_cast<float>(twist_cov[14]), // Z
    };

    report.quality = 100;

    visual_odom_pub_->publish(report);
  }

  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg) {
    // Parse position trajectory control points
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

    // Create B-spline trajectory
    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // Store trajectory and derivatives
    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative()); // velocity
    traj_.push_back(traj_[1].getDerivative()); // acceleration

    traj_duration_ = traj_[0].getTimeSum();
    start_time_ = rclcpp::Time(msg->start_time);
    traj_id_ = msg->traj_id;
    receive_traj_ = true;

    RCLCPP_INFO(this->get_logger(), "Received trajectory %d, duration: %.2fs",
                traj_id_, traj_duration_);
  }

  void cmdCallback() {
    if (!receive_traj_)
      return;

    auto time_now = this->now();
    double t_cur = (time_now - start_time_).seconds();

    Eigen::Vector3d pos_enu, vel_enu, acc_enu;
    double yaw_enu = 0.0, yaw_dot_enu = 0.0;

    if (t_cur >= 0.0 && t_cur < traj_duration_) {
      // Evaluate trajectory
      pos_enu = traj_[0].evaluateDeBoorT(t_cur);
      vel_enu = traj_[1].evaluateDeBoorT(t_cur);
      acc_enu = traj_[2].evaluateDeBoorT(t_cur);

      // Calculate yaw from velocity direction (in ENU)
      auto [yaw, yaw_dot] = calculateYaw(t_cur, pos_enu, time_now);
      yaw_enu = yaw;
      yaw_dot_enu = yaw_dot;

    } else if (t_cur >= traj_duration_) {
      // Hover at end position
      pos_enu = traj_[0].evaluateDeBoorT(traj_duration_);
      vel_enu.setZero();
      acc_enu.setZero();
      yaw_enu = last_yaw_;
      yaw_dot_enu = 0.0;
    } else {
      return; // Invalid time
    }

    time_last_ = time_now;

    // Convert ENU to NED
    // NOTE: Since VIO bridge forces PX4 frame to match LIO frame (converted),
    // we just use standard ENU->NED conversion here.
    Eigen::Vector3d pos_ned = enu_to_ned(pos_enu);
    Eigen::Vector3d vel_ned = enu_to_ned(vel_enu);
    Eigen::Vector3d acc_ned = enu_to_ned(acc_enu);

    // Convert yaw ENU to NED
    double yaw_ned = enu_yaw_to_ned(yaw_enu);
    double yaw_dot_ned = enu_yawrate_to_ned(yaw_dot_enu);

    // Normalize yaw
    while (yaw_ned > M_PI)
      yaw_ned -= 2.0 * M_PI;
    while (yaw_ned < -M_PI)
      yaw_ned += 2.0 * M_PI;

    // Publish TrajectorySetpoint
    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = this->now().nanoseconds() / 1000; // microseconds

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

  void offboardCallback() {
    // Must publish OffboardControlMode at >2Hz for PX4 to stay in offboard
    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = this->now().nanoseconds() / 1000;
    mode.position = true;
    mode.velocity = true;
    mode.acceleration = true;
    mode.attitude = false;
    mode.body_rate = false;

    offboard_mode_pub_->publish(mode);
  }

  std::pair<double, double> calculateYaw(double t_cur,
                                         const Eigen::Vector3d &pos,
                                         const rclcpp::Time &time_now) {
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX = PI; // rad/s

    double dt = (time_now - time_last_).seconds();
    if (dt < 1e-6)
      dt = 0.02; // Default to 50Hz

    // Look ahead for yaw direction
    double t_ahead = std::min(t_cur + time_forward_, traj_duration_);
    Eigen::Vector3d dir = traj_[0].evaluateDeBoorT(t_ahead) - pos;

    double yaw_target =
        dir.norm() > 0.1 ? std::atan2(dir.y(), dir.x()) : last_yaw_;
    double max_change = YAW_DOT_MAX * dt;

    // Rate limit yaw change
    double yaw_diff = yaw_target - last_yaw_;

    // Normalize to [-PI, PI]
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

    // Normalize
    while (yaw > PI)
      yaw -= 2.0 * PI;
    while (yaw < -PI)
      yaw += 2.0 * PI;

    // Low-pass filter
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;
    yaw_dot = 0.5 * last_yaw_dot_ + 0.5 * yaw_dot;

    last_yaw_ = yaw;
    last_yaw_dot_ = yaw_dot;

    return {yaw, yaw_dot};
  }

  static void rotateQuaternion(Eigen::Quaterniond &q_frd_to_ned,
                               const Eigen::Quaterniond &q_flu_to_enu) {
    // FLU (ROS) to FRD (PX4) static rotation
    static const Eigen::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
    // ENU to NED rotation (symmetric)
    static const Eigen::Quaterniond q_enu_to_ned(0.0, 0.70711, 0.70711, 0.0);

    q_frd_to_ned = q_enu_to_ned * q_flu_to_enu * q_flu_to_frd.conjugate();
  }

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;

  // Subscribers
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;

  // Trajectory state
  bool receive_traj_ = false;
  std::vector<UniformBspline> traj_;
  double traj_duration_ = 0.0;
  rclcpp::Time start_time_;
  rclcpp::Time time_last_;
  int traj_id_ = 0;

  // Yaw control state
  double last_yaw_ = 0.0;
  double last_yaw_dot_ = 0.0;
  double time_forward_ = 1.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajServerPX4>();

  RCLCPP_WARN(node->get_logger(), "[Traj Server PX4]: ready + VIO Bridge.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
