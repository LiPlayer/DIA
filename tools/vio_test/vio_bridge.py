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

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        # Use Gazebo simulation time for consistent timestamps
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', value=True)])

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

        # Subscriber: Ground Truth Odometry from Gazebo bridge
        self.gt_sub = self.create_subscription(
            Odometry,
            '/model/x500_gzlidar_0/odometry_with_covariance',
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

        self.get_logger().info("VIO Bridge Node Started: /model/x500_gzlidar_0/odometry_with_covariance -> /fmu/in/vehicle_visual_odometry")

    def gt_callback(self, msg):
        vo_msg = VehicleOdometry()
        
        # Timestamp
        vo_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vo_msg.timestamp_sample = int(
            Time.from_msg(msg.header.stamp).nanoseconds / 1000)

        # Pose Frame: NED
        vo_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        
        # Position ENU -> NED (x=East, y=North, z=Up -> x=North, y=East, z=Down)
        # X_ned = Y_enu
        # Y_ned = X_enu
        # Z_ned = -Z_enu
        
        # msg is nav_msgs/Odometry
        vo_msg.position = [
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.x),
            float(-msg.pose.pose.position.z)
        ]
        # Do not provide attitude; rely on IMU
        vo_msg.q = [float('nan'), float('nan'), float('nan'), float('nan')]

        # Velocity: use Gazebo ground truth (nav_msgs/Odometry)
        vo_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        vo_msg.velocity = [
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.x),
            float(-msg.twist.twist.linear.z)
        ]
        # Do not provide angular velocity
        vo_msg.angular_velocity = [float('nan'), float('nan'), float('nan')]
        
        # Variances: use provided covariances when valid, else conservative defaults
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        def safe_var(val, default):
            return default if (math.isnan(val) or val <= 0.0) else float(val)

        pos_default = 0.05
        vel_default = 0.2
        roll_pitch_default = 0.01
        yaw_default = 0.1
        vo_msg.position_variance = [
            safe_var(pose_cov[0], pos_default),
            safe_var(pose_cov[7], pos_default),
            safe_var(pose_cov[14], pos_default),
        ]
        vo_msg.orientation_variance = [
            safe_var(pose_cov[21], roll_pitch_default),
            safe_var(pose_cov[28], roll_pitch_default),
            safe_var(pose_cov[35], yaw_default),
        ]
        vo_msg.velocity_variance = [
            safe_var(twist_cov[0], vel_default),
            safe_var(twist_cov[7], vel_default),
            safe_var(twist_cov[14], vel_default),
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

def main(args=None):
    rclpy.init(args=args)
    node = VIOBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
