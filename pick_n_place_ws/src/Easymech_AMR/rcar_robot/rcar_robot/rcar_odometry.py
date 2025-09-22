#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from encoder_msgs.msg import EncoderData  # Replace with actual encoder message type
import math
import tf2_ros
from collections import deque


class SkidSteerOdometry(Node):
    def __init__(self):
        super().__init__("skid_steer_odometry")

        # Robot parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("wheel_radius", 0.05), # in meters
                ("wheel_separation", 0.45), # Distance between left and right wheels in meters
                ("encoder_resolution", 306),  # Encoder ticks per revolution
                ("slip_factor_linear", 1.0),  # Adjust empirically
                ("slip_factor_angular", 0.48),  # Increased from 0.47 for better rotation accuracy
                ("velocity_buffer_size", 10),  # Circular buffer size for velocity smoothing
            ],
        )

        # Load parameters
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.encoder_resolution = self.get_parameter("encoder_resolution").value
        self.k_v = self.get_parameter("slip_factor_linear").value
        self.k_omega = self.get_parameter("slip_factor_angular").value
        self.buffer_size = self.get_parameter("velocity_buffer_size").value

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()
        self.first_update = True

        # Circular buffers for encoder data and timestamps
        self.encoder_buffer = deque(maxlen=self.buffer_size)
        self.time_buffer = deque(maxlen=self.buffer_size)

        # Previous encoder values (for first-time initialization)
        self.prev_enc_left_front = None
        self.prev_enc_left_back = None
        self.prev_enc_right_front = None
        self.prev_enc_right_back = None

        # ROS 2 Interfaces
        self.odom_pub = self.create_publisher(Odometry, "odom_base", 10)
        self.encoder_sub = self.create_subscription(
            EncoderData, "encoder_data", self.encoder_callback, 10
        )

        # TF Broadcaster
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Skid-Steer Odometry Node Initialized")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}m")
        self.get_logger().info(f"Wheel separation: {self.wheel_separation}m") 
        self.get_logger().info(f"Encoder resolution: {self.encoder_resolution} ticks/rev")
        self.get_logger().info(f"Linear slip factor: {self.k_v}")
        self.get_logger().info(f"Angular slip factor: {self.k_omega}")
        self.get_logger().info(f"Velocity buffer size: {self.buffer_size}")
        
        # Add a counter for debugging
        self.msg_count = 0

    def encoder_callback(self, msg):
        """
        Callback for encoder readings with circular buffer smoothing.
        """
        current_time = self.get_clock().now()
        
        # Add current encoder data and timestamp to buffers
        encoder_data = {
            'left_front': msg.enc_left_front,
            'left_back': msg.enc_left_back,
            'right_front': msg.enc_right_front,
            'right_back': msg.enc_right_back
        }
        
        self.encoder_buffer.append(encoder_data)
        self.time_buffer.append(current_time)
        
        # Need at least 2 data points to calculate velocity
        if len(self.encoder_buffer) < 2:
            return
            
        # Calculate velocities using buffered data
        vL, vR = self.compute_buffered_wheel_velocities()
        
        if vL is None or vR is None:
            return
            
        v, omega = self.compute_corrected_odometry(vL, vR)
        
        # Use the time difference for pose integration
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt > 0:
            self.x, self.y, self.theta = self.update_pose(v, omega, dt)
            self.publish_odometry(v, omega, current_time)
            
            # Periodic debug logging (every 50 messages)
            # self.msg_count += 1
            # if self.msg_count % 50 == 0:
            #     self.get_logger().info(f"vL: {vL:.3f}, vR: {vR:.3f}, v: {v:.3f}, omega: {omega:.3f}")
            
        self.prev_time = current_time

    def compute_buffered_wheel_velocities(self):
        """
        Compute wheel velocities using circular buffer for smoothing.
        Uses oldest and newest data in buffer for more stable velocity calculation.
        """
        if len(self.encoder_buffer) < 2:
            return None, None
            
        # Use the oldest and newest data points in the buffer
        oldest_data = self.encoder_buffer[0]
        newest_data = self.encoder_buffer[-1]
        oldest_time = self.time_buffer[0]
        newest_time = self.time_buffer[-1]
        
        dt = (newest_time - oldest_time).nanoseconds * 1e-9
        
        # Avoid division by very small time intervals
        if dt < 0.001:  # Less than 1ms
            return None, None
            
        # Calculate left wheel velocity
        delta_left_front = newest_data['left_front'] - oldest_data['left_front']
        delta_left_back = newest_data['left_back'] - oldest_data['left_back']
        delta_left_ticks = (delta_left_front + delta_left_back) / 2.0
        
        # Calculate right wheel velocity  
        delta_right_front = newest_data['right_front'] - oldest_data['right_front']
        delta_right_back = newest_data['right_back'] - oldest_data['right_back']
        delta_right_ticks = (delta_right_front + delta_right_back) / 2.0
        
        # Convert to linear velocities
        wheel_circumference = 2 * math.pi * self.wheel_radius
        vL = (delta_left_ticks / self.encoder_resolution) * wheel_circumference / dt
        vR = (delta_right_ticks / self.encoder_resolution) * wheel_circumference / dt
        
        return vL, vR

    def compute_wheel_velocity(
        self, front_ticks, back_ticks, prev_front, prev_back, dt
    ):
        """
        Legacy method - kept for compatibility but not used with buffered approach.
        """
        delta_front = front_ticks - prev_front
        delta_back = back_ticks - prev_back
        delta_ticks = (delta_front + delta_back) / 2.0
        wheel_circumference = 2 * math.pi * self.wheel_radius
        velocity = (delta_ticks / self.encoder_resolution) * wheel_circumference / dt
        return velocity

    def compute_corrected_odometry(self, vL, vR):
        v = ((vR + vL) / 2) * self.k_v
        omega = ((vR - vL) / self.wheel_separation) * self.k_omega
        return v, omega

    def update_pose(self, v, omega, dt):
        x = self.x + v * math.cos(self.theta) * dt
        y = self.y + v * math.sin(self.theta) * dt
        theta = self.theta + omega * dt
        return x, y, theta

    def publish_odometry(self, v, omega, current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

    def publish_transform(self, current_time):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        #self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()