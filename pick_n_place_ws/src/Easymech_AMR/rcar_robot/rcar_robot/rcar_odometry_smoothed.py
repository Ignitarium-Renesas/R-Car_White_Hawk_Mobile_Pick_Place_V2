#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from encoder_msgs.msg import EncoderData
import math
import tf2_ros
from collections import deque


class RollingAccumulator:
    """Rolling accumulator for smoothing velocity calculations."""
    
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    
    def accumulate(self, value):
        """Add a new value to the accumulator."""
        self.values.append(value)
    
    def get_rolling_mean(self):
        """Get the rolling mean of accumulated values."""
        if len(self.values) == 0:
            return 0.0
        return sum(self.values) / len(self.values)
    
    def clear(self):
        """Clear all accumulated values."""
        self.values.clear()


class SkidSteerOdometrySmoothed(Node):
    def __init__(self):
        super().__init__("skid_steer_odometry_smoothed")

        # Robot parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("wheel_radius", 0.05),  # in meters
                ("wheel_separation", 0.45),  # Distance between left and right wheels in meters
                ("encoder_resolution", 306),  # Encoder ticks per revolution
                ("slip_factor_linear", 1.0),  # Adjust empirically
                ("slip_factor_angular", 0.50),  # Adjust for better rotation accuracy
                ("accumulator_window_size", 5),  # Rolling window size for smoothing
                ("odom_frame_id", "odom"),
                ("odom_child_frame_id", "base_footprint"),
                ("linear_covariance", 0.01),
                ("yaw_covariance", 0.01),
            ],
        )

        # Load parameters
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.encoder_resolution = self.get_parameter("encoder_resolution").value
        self.k_v = self.get_parameter("slip_factor_linear").value
        self.k_omega = self.get_parameter("slip_factor_angular").value
        self.window_size = self.get_parameter("accumulator_window_size").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.odom_child_frame_id = self.get_parameter("odom_child_frame_id").value
        self.linear_covariance = self.get_parameter("linear_covariance").value
        self.yaw_covariance = self.get_parameter("yaw_covariance").value

        # Odometry pose variables (static equivalents)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.theta = 0.0
        self.past_time = 0.0
        self.first_update = True

        # Rolling accumulators for velocity smoothing
        self.linear_accumulator = RollingAccumulator(self.window_size)
        self.angular_accumulator = RollingAccumulator(self.window_size)

        # Previous encoder values
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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Skid-Steer Smoothed Odometry Node Initialized")

    def encoder_callback(self, msg):
        """
        Callback for encoder readings with rolling accumulator smoothing.
        """
        # Calculate time
        ros_now_time = self.get_clock().now()
        now_time = ros_now_time.nanoseconds * 1e-9  # Convert to seconds
        
        if self.first_update:
            self.prev_enc_left_front = msg.enc_left_front
            self.prev_enc_left_back = msg.enc_left_back
            self.prev_enc_right_front = msg.enc_right_front
            self.prev_enc_right_back = msg.enc_right_back
            self.past_time = now_time
            self.first_update = False
            return

        dt = now_time - self.past_time
        if dt <= 0:
            return

        # Calculate wheel velocities
        vL = self.compute_wheel_velocity(
            msg.enc_left_front,
            msg.enc_left_back,
            self.prev_enc_left_front,
            self.prev_enc_left_back,
            dt,
        )
        vR = self.compute_wheel_velocity(
            msg.enc_right_front,
            msg.enc_right_back,
            self.prev_enc_right_front,
            self.prev_enc_right_back,
            dt,
        )

        # Store previous encoder values
        self.prev_enc_left_front = msg.enc_left_front
        self.prev_enc_left_back = msg.enc_left_back
        self.prev_enc_right_front = msg.enc_right_front
        self.prev_enc_right_back = msg.enc_right_back

        # Calculate robot velocities
        v, omega = self.compute_corrected_odometry(vL, vR)
        
        # Accumulate velocities for smoothing
        self.linear_accumulator.accumulate(v)
        self.angular_accumulator.accumulate(omega)
        
        # Get smoothed velocities
        mean_linear = self.linear_accumulator.get_rolling_mean()
        mean_angular = self.angular_accumulator.get_rolling_mean()

        # Calculate position (only if not first iteration)
        if self.past_time != 0:
            self.pos_x += mean_linear * math.cos(self.theta) * dt
            self.pos_y += mean_linear * math.sin(self.theta) * dt
            self.theta += mean_angular * dt

        # Update past time
        self.past_time = now_time

        # Publish odometry
        self.publish_odometry(mean_linear, mean_angular, ros_now_time)
        self.publish_transform(ros_now_time)

    def compute_wheel_velocity(self, front_ticks, back_ticks, prev_front, prev_back, dt):
        """Calculate wheel velocity from encoder ticks."""
        delta_front = front_ticks - prev_front
        delta_back = back_ticks - prev_back
        delta_ticks = (delta_front + delta_back) / 2.0
        wheel_circumference = 2 * math.pi * self.wheel_radius
        velocity = (delta_ticks / self.encoder_resolution) * wheel_circumference / dt
        return velocity

    def compute_corrected_odometry(self, vL, vR):
        """Calculate robot linear and angular velocities with slip correction."""
        v = ((vR + vL) / 2) * self.k_v
        omega = ((vR - vL) / self.wheel_separation) * self.k_omega
        return v, omega

    def publish_odometry(self, mean_linear, mean_angular, current_time):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.odom_child_frame_id

        # Position
        odom_msg.pose.pose.position.x = self.pos_x
        odom_msg.pose.pose.position.y = self.pos_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        # Velocity
        odom_msg.twist.twist.linear.x = mean_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = mean_angular

        # Covariance matrices
        # Pose covariance (6x6 = 36 elements)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = self.linear_covariance   # x
        odom_msg.pose.covariance[7] = self.linear_covariance   # y
        odom_msg.pose.covariance[35] = self.yaw_covariance     # yaw

        # Twist covariance (6x6 = 36 elements)
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = self.linear_covariance   # linear x
        odom_msg.twist.covariance[7] = self.linear_covariance   # linear y
        odom_msg.twist.covariance[35] = self.yaw_covariance     # angular z

        self.odom_pub.publish(odom_msg)

    def publish_transform(self, current_time):
        """Publish TF transform from odom to base_footprint."""
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = self.odom_frame_id
        odom_trans.child_frame_id = self.odom_child_frame_id

        # Translation
        odom_trans.transform.translation.x = self.pos_x
        odom_trans.transform.translation.y = self.pos_y
        odom_trans.transform.translation.z = 0.0

        # Rotation
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation.x = quat[0]
        odom_trans.transform.rotation.y = quat[1]
        odom_trans.transform.rotation.z = quat[2]
        odom_trans.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(odom_trans)


def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerOdometrySmoothed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
