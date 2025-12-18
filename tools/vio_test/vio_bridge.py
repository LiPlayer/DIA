#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import math
from typing import Tuple

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        # Use Gazebo simulation time for consistent timestamps
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', value=True)])

        # Input odometry topic (nav_msgs/Odometry from LIO-SAM or similar)
        self.declare_parameter('odom_topic', '/odometry/imu')
        odom_topic = self.get_parameter('odom_topic').value

        # QoS for PX4 messages
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for ROS messages
        ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: Odometry input (e.g., LIO-SAM odometry/imu)
        self.gt_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.gt_callback,
            ros_qos
        )

        # Publisher: Visual Odometry to PX4
        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )
        
        # Publisher: Ground Truth Path (Visualization)
        self.gt_path_pub = self.create_publisher(
            Path,
            '/visual/ground_truth',
            10
        )
        self.gt_path = Path()
        self.gt_path.header.frame_id = ""

        self.get_logger().info(f"VIO Bridge Node Started: {odom_topic} -> /fmu/in/vehicle_visual_odometry")

    def gt_callback(self, msg):
        vo_msg = VehicleOdometry()
        # Timestamp (match PX4 odom callback: sample + current)
        vo_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vo_msg.timestamp_sample = int(Time.from_msg(msg.header.stamp).nanoseconds / 1000)

        # Pose frame: convert ENU/FLU -> NED/FRD like GZBridge::odometryCallback
        vo_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vo_msg.position = [
            float(msg.pose.pose.position.y),            # Y_enu -> X_ned
            float(msg.pose.pose.position.x),            # X_enu -> Y_ned
            float(-msg.pose.pose.position.z),           # Z_enu -> -Z_ned
        ]

        # Orientation: rotate FLU->ENU quaternion into FRD->NED
        q_nb = self.rotate_quaternion_enu_flu_to_ned_frd(msg.pose.pose.orientation)
        vo_msg.q = [float(q_nb[0]), float(q_nb[1]), float(q_nb[2]), float(q_nb[3])]

        # Velocity: body FLU -> body FRD, keep in body frame
        vo_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        vo_msg.velocity = [
            float(msg.twist.twist.linear.x),            # +X stays forward
            float(-msg.twist.twist.linear.y),           # left -> -right
            float(-msg.twist.twist.linear.z),           # up -> -down
        ]
        vo_msg.angular_velocity = [
            float(msg.twist.twist.angular.x),
            float(-msg.twist.twist.angular.y),
            float(-msg.twist.twist.angular.z),
        ]
        
        # Variances: mirror PX4 mapping; fall back to conservative defaults
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        def safe_var(val, default):
            return default if (math.isnan(val) or val <= 0.0) else float(val)

        pos_default = 0.05
        vel_default = 0.2
        roll_pitch_default = 0.01
        yaw_default = 0.1
        vo_msg.position_variance = [
            safe_var(pose_cov[7], pos_default),   # Y -> X_ned
            safe_var(pose_cov[0], pos_default),   # X -> Y_ned
            safe_var(pose_cov[14], pos_default),  # Z
        ]
        vo_msg.orientation_variance = [
            safe_var(pose_cov[21], roll_pitch_default), # roll
            safe_var(pose_cov[28], roll_pitch_default), # pitch
            safe_var(pose_cov[35], yaw_default),        # yaw
        ]
        vo_msg.velocity_variance = [
            safe_var(twist_cov[7], vel_default),  # Y -> X_ned
            safe_var(twist_cov[0], vel_default),  # X -> Y_ned
            safe_var(twist_cov[14], vel_default), # Z
        ]
        vo_msg.quality = 100
        vo_msg.reset_counter = 0

        self.visual_odom_pub.publish(vo_msg)
        
        # Publish Ground Truth Path for visualization
        if not self.gt_path.header.frame_id:
            self.gt_path.header.frame_id = msg.header.frame_id or "map"
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.gt_path.header.stamp = pose_stamped.header.stamp
        self.gt_path.poses.append(pose_stamped)
        self.gt_path_pub.publish(self.gt_path)

    def rotate_quaternion_enu_flu_to_ned_frd(self, q_msg) -> Tuple[float, float, float, float]:
        """Replicate GZBridge::rotateQuaternion: ENU/FLU -> NED/FRD."""
        # Incoming quaternion from ROS msg is FLU->ENU with (x,y,z,w)
        q_flu_to_enu = (float(q_msg.w), float(q_msg.x), float(q_msg.y), float(q_msg.z))  # w, x, y, z

        # Static rotations
        q_flu_to_frd = (0.0, 1.0, 0.0, 0.0)
        q_enu_to_ned = (0.0, 0.70711, 0.70711, 0.0)

        q_frd_to_ned = self.quat_multiply(
            q_enu_to_ned,
            self.quat_multiply(q_flu_to_enu, self.quat_conjugate(q_flu_to_frd)),
        )
        return q_frd_to_ned

    @staticmethod
    def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        w, x, y, z = q
        return (w, -x, -y, -z)

    @staticmethod
    def quat_multiply(a: Tuple[float, float, float, float],
                      b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

def main(args=None):
    rclpy.init(args=args)
    node = VIOBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
