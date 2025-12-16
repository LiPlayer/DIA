#pragma once

#include <Eigen/Eigen>
#include <cmath>

namespace trajectory_control {

/**
 * Convert position/velocity from ENU to NED frame
 * ENU: East-North-Up (ROS/LIO-SAM convention)
 * NED: North-East-Down (PX4 convention)
 */
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d &enu) {
  return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

/**
 * Convert yaw angle from ENU to NED convention
 * ENU yaw: 0 = East, CCW positive
 * NED yaw: 0 = North, CW positive
 */
inline double enu_yaw_to_ned(double yaw_enu) {
  double yaw_ned = M_PI_2 - yaw_enu;
  // Normalize to [-PI, PI]
  while (yaw_ned > M_PI)
    yaw_ned -= 2.0 * M_PI;
  while (yaw_ned < -M_PI)
    yaw_ned += 2.0 * M_PI;
  return yaw_ned;
}

/**
 * Convert yaw rate from ENU to NED
 * ENU: CCW positive, NED: CW positive
 */
inline double enu_yawrate_to_ned(double yawrate_enu) { return -yawrate_enu; }

} // namespace trajectory_control
