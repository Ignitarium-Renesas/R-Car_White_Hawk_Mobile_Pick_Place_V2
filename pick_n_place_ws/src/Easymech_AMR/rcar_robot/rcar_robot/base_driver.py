import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from encoder_msgs.msg import EncoderData  # Custom message

import serial
import time
import re
import math
from math import pi


class UnifiedRobotNode(Node):
    def __init__(self):
        super().__init__("base_driver_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pub_rate_hz", 30.0)            # Hz: how often to read/publish
        self.declare_parameter("fresh_window_sec", 0.25)       # s: max age for data to be considered fresh

        self.wheel_separation = 0.45
        self.max_motor_rpm = 60
        self.wheel_rad = 0.050

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        pub_rate_hz = float(self.get_parameter("pub_rate_hz").get_parameter_value().double_value)
        self.fresh_window = float(self.get_parameter("fresh_window_sec").get_parameter_value().double_value)

        # Serial init
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f"Opened serial port {port} at {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.last_cmd_time = time.time()
        self.last_warn_time = 0.0

        # Track previous motor speeds to avoid redundant serial writes
        self.prev_left_motor_speed = None
        self.prev_right_motor_speed = None

        # ROS interfaces
        self.subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.encoder_publisher = self.create_publisher(EncoderData, "encoder_data", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        # Timer
        self.create_timer(1.0 / max(pub_rate_hz, 1e-3), self.read_and_publish)

        # Regex patterns
        # Example odom line: $BR:0,BL:0,FL:0,FR:0,BT:4.82#
        self.odom_pattern = re.compile(
            r"^\$BR:(-?\d+),BL:(-?\d+),FL:(-?\d+),FR:(-?\d+),BT:([-\d.]+)#?$"
        )

        # Example IMU line from your logs:
        # Scaled. Acc (mg) [  00014.16, -00033.69,  00984.86 ], Gyr (DPS) [  ... ], Mag (uT) [ ... ], Tmp (C) [  ... ]
        self.imu_pattern = re.compile(
            r"^Scaled\. Acc \(mg\) \[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\], "
            r"Gyr \(DPS\) \[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\], "
            r"Mag \(uT\) \[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\], "
            r"Tmp \(C\) \[\s*([-\d.]+)\s*\]$"
        )

        # Latest buffers:
        # latest_odom: (t, fr, bl, fl, br, bt)
        # latest_imu:  (t, ax, ay, az, gx, gy, gz, mx, my, mz, temp)
        self.latest_odom = None
        self.latest_imu = None

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        left_speed = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2.0)

        # m/s -> RPM: rpm = v / (2*pi*r) * 60
        left_motor_rpm = (left_speed / (2.0 * math.pi * self.wheel_rad)) * 60.0
        right_motor_rpm = (right_speed / (2.0 * math.pi * self.wheel_rad)) * 60.0
        self.get_logger().debug(f"Calculated motor RPM: left={left_motor_rpm}, right={right_motor_rpm}")
        # Clamp
        left_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, left_motor_rpm))
        right_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, right_motor_rpm))

        # Only write if changed
        if (self.prev_left_motor_speed != left_motor_rpm) or (self.prev_right_motor_speed != right_motor_rpm):
            # Convert to integers for the serial command
            left_motor_rpm_int = int(left_motor_rpm)
            right_motor_rpm_int = int(right_motor_rpm)
            
            # Validation checks before sending command
            if abs(left_motor_rpm_int) > self.max_motor_rpm:
                self.get_logger().warn(f"Left motor RPM {left_motor_rpm_int} exceeds max {self.max_motor_rpm}, clamping")
                left_motor_rpm_int = max(-self.max_motor_rpm, min(self.max_motor_rpm, left_motor_rpm_int))
            
            if abs(right_motor_rpm_int) > self.max_motor_rpm:
                self.get_logger().warn(f"Right motor RPM {right_motor_rpm_int} exceeds max {self.max_motor_rpm}, clamping")
                right_motor_rpm_int = max(-self.max_motor_rpm, min(self.max_motor_rpm, right_motor_rpm_int))
            
            # Additional safety checks
            if not (-self.max_motor_rpm <= left_motor_rpm_int <= self.max_motor_rpm) or \
               not (-self.max_motor_rpm <= right_motor_rpm_int <= self.max_motor_rpm):
                self.get_logger().error(f"Invalid motor speeds after validation: left={left_motor_rpm_int}, right={right_motor_rpm_int}")
                return
            
            command = f"$FR:{right_motor_rpm_int},FL:{left_motor_rpm_int},BR:{right_motor_rpm_int},BL:{left_motor_rpm_int}#"
            
            # Validate command format before sending
            if not command.startswith("$") or not command.endswith("#"):
                self.get_logger().error(f"Invalid command format: {command}")
                return
            
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(command.encode())
                    self.get_logger().debug(f"Sent command: {command.strip()}")
                else:
                    self.get_logger().warn("Serial port not available for writing")
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")

            self.prev_left_motor_speed = left_motor_rpm
            self.prev_right_motor_speed = right_motor_rpm

    def _parse_odom_line(self, line: str):
        m = self.odom_pattern.match(line)
        if not m:
            return None
        back_right_ticks, back_left_ticks, front_left_ticks, front_right_ticks, battery_voltage = m.groups()
        # Cast; keep ticks as ints then store for publish
        back_right_ticks = int(back_right_ticks)
        back_left_ticks = int(back_left_ticks) 
        front_left_ticks = int(front_left_ticks)
        front_right_ticks = int(front_right_ticks)
        battery_voltage = float(battery_voltage)
        return (time.time(), front_right_ticks, back_left_ticks, front_left_ticks, back_right_ticks, battery_voltage)

    def _parse_imu_line(self, line: str):
        m = self.imu_pattern.match(line)
        if not m:
            return None
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = map(float, m.groups())
        return (time.time(), accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)

    def _drain_serial(self):
        """Read and parse all available lines; update latest buffers."""
        if not self.serial_port:
            return

        try:
            while self.serial_port.in_waiting > 0:
                raw = self.serial_port.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("$BR"):
                    od = self._parse_odom_line(line)
                    if od:
                        self.latest_odom = od
                    else:
                        self.get_logger().debug(f"Odom line did not match: {line}")

                elif line.startswith("Scaled."):
                    imu = self._parse_imu_line(line)
                    if imu:
                        self.latest_imu = imu
                    else:
                        self.get_logger().debug(f"IMU line did not match: {line}")

                else:
                    # Ignore everything else
                    self.get_logger().debug(f"Ignored line: {line}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def _is_fresh(self, tstamp: float) -> bool:
        return (time.time() - tstamp) <= self.fresh_window

    def read_and_publish(self):
        # Drain serial buffer
        self._drain_serial()

        now_ros = self.get_clock().now().to_msg()

        # Publish encoder data if odom fresh
        if self.latest_odom and self._is_fresh(self.latest_odom[0]):
            _, front_right_ticks, back_left_ticks, front_left_ticks, back_right_ticks, _battery_voltage = self.latest_odom

            enc_msg = EncoderData()
            # Map to encoder message fields based on your robot's configuration
            enc_msg.enc_left_front = float(front_left_ticks)
            enc_msg.enc_left_back = float(back_left_ticks)
            enc_msg.enc_right_front = float(front_right_ticks)
            enc_msg.enc_right_back = float(back_right_ticks)
            self.encoder_publisher.publish(enc_msg)

        # Publish IMU and Mag if IMU fresh
        if self.latest_imu and self._is_fresh(self.latest_imu[0]):
            _, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = self.latest_imu

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = now_ros
            imu_msg.header.frame_id = "imu_link"

            # Units: mg -> m/s^2 ; dps -> rad/s ; uT -> Tesla in MagneticField below
            imu_msg.linear_acceleration.x = accel_x * 9.80665 / 1000.0
            imu_msg.linear_acceleration.y = accel_y * 9.80665 / 1000.0
            imu_msg.linear_acceleration.z = accel_z * 9.80665 / 1000.0
            imu_msg.angular_velocity.x = gyro_x * pi / 180.0
            imu_msg.angular_velocity.y = gyro_y * pi / 180.0
            imu_msg.angular_velocity.z = gyro_z * pi / 180.0
            imu_msg.orientation_covariance[0] = -1.0  # unknown orientation
            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = mag_x * 1e-6
            mag_msg.magnetic_field.y = mag_y * 1e-6
            mag_msg.magnetic_field.z = mag_z * 1e-6
            self.mag_pub.publish(mag_msg)

        # Optional: periodic warning if nothing fresh
        if (self.latest_odom is None or not self._is_fresh(self.latest_odom[0])) and \
           (self.latest_imu is None or not self._is_fresh(self.latest_imu[0])):
            now = time.time()
            if now - self.last_warn_time > 1.0:
                self.get_logger().warn("No fresh serial data recently.")
                self.last_warn_time = now


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        try:
            if getattr(node, "serial_port", None) and node.serial_port.is_open:
                node.serial_port.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
