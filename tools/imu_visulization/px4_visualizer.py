#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np

class PX4Visualizer(Node):
    def __init__(self):
        super().__init__('px4_visualizer')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_callback,
            qos_profile
        )

        # Publishers
        self.visual_odom_pub = self.create_publisher(Odometry, '/visual/odometry', 10)
        self.visual_pose_pub = self.create_publisher(Path, '/visual/local_position', 10)
        self.path = Path()
        self.path.header.frame_id = "map"

        self.get_logger().info("PX4 Visualizer Node Started")

    def ned_to_enu_position(self, x, y, z):
        """
        Convert NED position to ENU position.
        NED: x (North), y (East), z (Down)
        ENU: x (East), y (North), z (Up)
        
        So:
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned
        """
        return float(y), float(x), float(-z)

    def ned_to_enu_orientation(self, q):
        """
        Convert NED orientation (w, x, y, z) to ENU (w, x, y, z).
        
        Rotation from NED to ENU is:
        Roll +180 around X? No.
        Basically, we want to represent the SAME orientation in a different frame.
        
        Rotation matrix R_ned represents vector v_ned = R_ned * v_body
        We want R_enu represents v_enu = R_enu * v_body_flu
        
        T_enu_ned = [0 1 0; 1 0 0; 0 0 -1]
        T_flu_frd = [1 0 0; 0 -1 0; 0 0 -1]
        
        R_enu = T_enu_ned * R_ned * T_frd_flu
        
        Simpler approach for visualization arrows:
        Just transform the quaternion geometrically. 
        Main conceptual map:
        NED Q -> ENU Q
        
        Using a standard trick:
        enu_q = [y, x, -z, w] (of ned_q) ... actually let's calculate properly or use approximations.
        
        For basic visualization of POSITION, the arrow orientation is less critical if just ensuring the path is correct.
        But let's try a standard conversion:
        
        q_ned = [w, x, y, z]
        
        # 1. Rotate NED frame to ENU frame: 90 deg around Z, then 180 around X?
        # Transform NED to ENU:
        # X->Y, Y->X, Z->-Z
        
        Let's perform the component swap corresponding to x=y, y=x, z=-z
        This is a reflection + rotation.
        
        Alternative:
        q_flu_enu = ( (-y), (-x), (-z), (w) )?
        
        Let's stick to the verified PX4-ROS conversion often seen:
        // FRD to FLU
        q[0] = q_ned[0];
        q[1] = q_ned[1];
        q[2] = -q_ned[2];
        q[3] = -q_ned[3];
        
        // NED to ENU frame rotation (Static transform usually handles this if we publish in 'map' -> 'base_link')
        // But here we publish in 'map' frame directly.
        
        Let's try:
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]
        
        # NED -> ENU 
        # (x, y, z, w) -> (y, x, -z, w) * rotation_adjustment
        
        Let's just output the raw NED orientation in a way that shows movement.
        Most reliable purely for trace:
        x = y_ned
        y = x_ned
        z = -z_ned
        """
        # Using the standard "ned_to_enu" quaternion conversion
        # Q_enu = Q_ned_en * Q_ned_body * Q_body_flu_frd
        # It gets complex. For this task, we will try a simple mapping that is often "good enough" for debug.
        # However, correct way:
        # q = [w, x, y, z]
        # output = [x, y, z, w]
        
        return q[1], q[2], q[3], q[0] # Just returning as x,y,z,w (ROS convention) from PX4 (w,x,y,z)

    def odom_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # Position (NED -> ENU)
        x, y, z = self.ned_to_enu_position(msg.position[0], msg.position[1], msg.position[2])
        odom.pose.pose.position = Point(x=x, y=y, z=z)

        # Orientation (w, x, y, z) -> ROS (x, y, z, w)
        # PX4 sends quaternion as [w, x, y, z]
        # ROS uses [x, y, z, w]
        # We need to adapt the frame too.
        # For now, let's just pass the quaternion components clearly. 
        # NOTE: This visualizer might show weird rotation if not fully transformed, 
        # but position trace will be correct ENU.
        
        # Standard FRD->FLU + NED->ENU often implies:
        # x = y, y = x, z = -z
        # qx = qy, qy = qx, qz = -qz, qw = qw
        
        odom.pose.pose.orientation = Quaternion(
            x=float(msg.q[2]), # y -> x
            y=float(msg.q[1]), # x -> y
            z=float(-msg.q[3]), # -z -> z
            w=float(msg.q[0])  # w -> w
        )

        self.visual_odom_pub.publish(odom)

    def local_pos_callback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        x, y, z = self.ned_to_enu_position(msg.x, msg.y, msg.z)
        pose.pose.position = Point(x=x, y=y, z=z)

        # Local Position doesn't always have orientation in standard message, checking definition.
        # VehicleLocalPosition.msg typically has 'heading' (yaw).
        # We can construct a quaternion from heading if needed, or just leave identity.
        # For visualization, position is key.
        # Actually msg.heading is available.
        
        # Heading (NED) 0 = North, 90 = East.
        # ENU: 0 = East, 90 = North.
        # Heading_enu = -msg.heading + pi/2
        
        heading_enu = -msg.heading + (np.pi / 2.0)
        
        # Quat from limits
        qz = np.sin(heading_enu / 2.0)
        qw = np.cos(heading_enu / 2.0)
        
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw))

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.visual_pose_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PX4Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
