#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class DynamicParamClient(Node):
    def __init__(self):
        super().__init__('dynamic_param_client')

        # Internal state
        self.goal_pose = None
        self.current_pose = None
        self.threshold = 1.0  # meters
        self.switched = False

        # Subscriptions
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('DynamicParamClient initialized. Waiting for goal and odometry...')

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.switched = False 
        self.get_logger().info(
            f"New goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        if not self.goal_pose:
            return

        distance = self.compute_distance(self.current_pose, self.goal_pose)

        if distance < self.threshold and not self.switched:
            self.get_logger().info("Within threshold. Sending parameter update...")
            self.send_request()
            self.switched = True

    def compute_distance(self, current, goal):
        dx = goal.position.x - current.position.x
        dy = goal.position.y - current.position.y
        return math.sqrt(dx**2 + dy**2)

    def send_request(self):
        # Define parameters to set per node
        parameter_map = {
            '/global_costmap/global_costmap/set_parameters': [
                ('inflation_layer.inflation_radius', 0.25, ParameterType.PARAMETER_DOUBLE),
            ],
            '/local_costmap/local_costmap/set_parameters': [
                ('inflation_layer.inflation_radius', 0.5, ParameterType.PARAMETER_DOUBLE),
            ],
            '/controller_server/set_parameters': [
                ('FollowPath.max_vel_x', 0.10, ParameterType.PARAMETER_DOUBLE),
                ('FollowPath.max_vel_theta', 0.5, ParameterType.PARAMETER_DOUBLE),
            ]
        }

        for node_service, params in parameter_map.items():
            client = self.create_client(SetParameters, node_service)
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f'Service not available: {node_service}')
                continue

            request = SetParameters.Request()
            request.parameters = []

            for name, value, ptype in params:
                param = Parameter()
                param.name = name
                param.value = ParameterValue(type=ptype)

                # Set value based on type
                if ptype == ParameterType.PARAMETER_DOUBLE:
                    param.value.double_value = value
                elif ptype == ParameterType.PARAMETER_BOOL:
                    param.value.bool_value = value
                elif ptype == ParameterType.PARAMETER_STRING:
                    param.value.string_value = value
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    param.value.integer_value = value
                else:
                    self.get_logger().warn(f"Unsupported type for param {name}")
                    continue

                request.parameters.append(param)

            future = client.call_async(request)

            def callback(fut, node_service=node_service):
                if fut.result() is not None:
                    self.get_logger().info(f" Parameters updated for: {node_service}")
                else:
                    self.get_logger().error(f" Failed to update: {node_service}")

            future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicParamClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
