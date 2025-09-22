import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import numpy as np

class MagnetometerCalibrator(Node):
    def __init__(self):
        super().__init__('magnetometer_calibrator')

        # Calibration parameters originally in microtesla (μT)
        # Convert ALL parameters to Tesla (T) to match the incoming data:
        self.bias = np.array([16.090469, -20.996425, 44.586515]) * 1e-6  # → Tesla
        self.soft_iron_matrix = np.array([
            [9.226377, 0.068942, -0.498326],
            [0.068942, 9.221827, 0.788710],
            [-0.498326, 0.788710, 11.933790]
        ])

        self.sub = self.create_subscription(
            MagneticField,
            '/imu/mag_raw',  # Already in Tesla
            self.mag_callback,
            10
        )
        self.pub = self.create_publisher(MagneticField, '/imu/mag', 10)

    def mag_callback(self, msg):
        # Use raw data directly in Tesla
        raw = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

        # Apply calibration: subtract bias (Tesla), then apply scale
        corrected = self.soft_iron_matrix @ (raw - self.bias)

        # Publish calibrated message (still in Tesla)
        calibrated_msg = MagneticField()
        calibrated_msg.header = msg.header
        calibrated_msg.magnetic_field.x = corrected[0]
        calibrated_msg.magnetic_field.y = corrected[1]
        calibrated_msg.magnetic_field.z = corrected[2]
        
        # Set realistic covariance for EKF fusion
        # For magnetometer data, typical noise is around 0.1-1.0 µT = 1e-7 to 1e-6 Tesla
        # After calibration, uncertainty might be ~0.5 µT = 5e-7 Tesla
        mag_variance = (5e-7) ** 2  # Square of standard deviation
        
        calibrated_msg.magnetic_field_covariance = [
            mag_variance, 0.0, 0.0,           # X variance and cross-correlations
            0.0, mag_variance, 0.0,           # Y variance and cross-correlations  
            0.0, 0.0, mag_variance            # Z variance
        ]

        self.pub.publish(calibrated_msg)

def main():
    rclpy.init()
    node = MagnetometerCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
