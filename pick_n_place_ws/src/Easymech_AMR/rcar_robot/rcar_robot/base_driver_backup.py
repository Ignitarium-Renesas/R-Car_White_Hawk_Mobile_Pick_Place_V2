import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from encoder_msgs.msg import EncoderData  # Custom message
import serial
import time
import re
from math import pi
import math


class UnifiedRobotNode(Node):
    def __init__(self):
        super().__init__("base_driver_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.wheel_separation = 0.5
        self.max_motor_rpm = 60  # Max motor speed (adjust as needed) in RPM
        self.wheel_rad = 0.0508  #meter

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} at {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.last_cmd_time = time.time()
        self.last_serial_time = time.time()
        self.timeout_seconds = 0.3

        # Track previous motor speeds
        self.prev_left_motor_speed = None
        self.prev_right_motor_speed = None

        # ROS interfaces
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self.encoder_publisher = self.create_publisher(EncoderData, "encoder_data", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        self.create_timer(0.1, self.read_and_publish)  # 10 Hz

        self.pattern = re.compile(
            r"\$BR:(-?\d+),BL:(-?\d+),FL:(-?\d+),FR:(-?\d+),BT:[\d.]+,Scaled\. Acc \(mg\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Gyr \(DPS\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Mag \(uT\) \[ *([-\d.]+), *([-\d.]+), *([-\d.]+) \], Tmp \(C\) \[ *([-\d.]+) \]"
        )

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left_speed = linear_x - (angular_z * self.wheel_separation / 2)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2)

        # Scale speed to motor range
        left_motor_rpm = float(left_speed * 60) / (2 * math.pi * self.wheel_rad)
        right_motor_rpm = float(right_speed * 60) / (2 * math.pi * self.wheel_rad)

        left_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, left_motor_rpm))
        right_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, right_motor_rpm))

        self.get_logger().info(f"left_motor_rpm {left_motor_rpm}")
        self.get_logger().info(f"right_motor_rpm {right_motor_rpm}")

        if (
            self.prev_left_motor_speed != left_motor_rpm or
            self.prev_right_motor_speed != right_motor_rpm
        ):
            command = f"$FR:{right_motor_rpm},FL:{left_motor_rpm},BR:{right_motor_rpm},BL:{left_motor_rpm}#"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command: {command.strip()}")

            self.prev_left_motor_speed = left_motor_rpm
            self.prev_right_motor_speed = right_motor_rpm

    def read_and_publish(self):
        latest_valid_line = None

        # Drain the serial buffer and keep the last valid line
        while self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if self.pattern.match(line):
                    latest_valid_line = line
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

        # If a valid line was found, parse and publish
        if latest_valid_line:
            match = self.pattern.match(latest_valid_line)
            if match:
                try:
                    self.last_serial_time = time.time()

                    fr, fl, br, bl, ax, ay, az, gx, gy, gz, mx, my, mz, temp = map(float, match.groups())

                    # Publish Encoder Data
                    enc_msg = EncoderData()
                    enc_msg.enc_left_front = br
                    enc_msg.enc_left_back = fl
                    enc_msg.enc_right_front = bl
                    enc_msg.enc_right_back = fr
                    self.encoder_publisher.publish(enc_msg)

                    now = self.get_clock().now().to_msg()

                    # Publish IMU
                    imu_msg = Imu()
                    imu_msg.header = Header()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = "imu_link"
                    imu_msg.linear_acceleration.x = ax * 9.80665 / 1000.0
                    imu_msg.linear_acceleration.y = ay * 9.80665 / 1000.0
                    imu_msg.linear_acceleration.z = az * 9.80665 / 1000.0
                    imu_msg.angular_velocity.x = gx * pi / 180.0
                    imu_msg.angular_velocity.y = gy * pi / 180.0
                    imu_msg.angular_velocity.z = gz * pi / 180.0
                    imu_msg.orientation_covariance[0] = -1.0
                    self.imu_pub.publish(imu_msg)

                    # Publish Magnetometer
                    mag_msg = MagneticField()
                    mag_msg.header = imu_msg.header
                    mag_msg.magnetic_field.x = mx * 1e-6
                    mag_msg.magnetic_field.y = my * 1e-6
                    mag_msg.magnetic_field.z = mz * 1e-6
                    self.mag_pub.publish(mag_msg)

                except Exception as e:
                    self.get_logger().error(f"Error while parsing values: {e}")
        else:
            # No valid line found — stale
            if time.time() - self.last_serial_time > self.timeout_seconds:
                self.get_logger().warn("No fresh serial data. Skipping publish.")

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
