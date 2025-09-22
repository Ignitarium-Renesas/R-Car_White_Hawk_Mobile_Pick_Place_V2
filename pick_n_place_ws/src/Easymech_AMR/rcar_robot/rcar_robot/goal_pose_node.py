#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import yaml
from pathlib import Path
import time


class NavigationWithRevert(Node):
    def __init__(self):
        super().__init__('navigation_with_revert')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Publishers and clients
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for action server
        self._navigate_action_client.wait_for_server()
        self.get_logger().info("Navigation action server is ready.")

        # Send goal
        self.send_goal()

    def send_goal(self):

        # Define the goal coordinates
        x, y = 1.50, -0.50

        self.get_logger().info(f"Sending goal to position: ({x}, {y})")

        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        
        self.goal_pose_pub.publish(pose)
        self.get_logger().info("Published goal to /goal_pose.")
        time.sleep(2)

        # Send NavigateToPose action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._send_goal_future = self._navigate_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        if not hasattr(self, 'feedback_logged'):
            self.get_logger().info("Navigation feedback received.")
            self.feedback_logged = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Navigation goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(" Navigation goal reached successfully.")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            # Revert params after reaching goal
            self.revert_params_from_yaml('/home/himanshu/myagv_ws/src/Easymech_AMR/rcar_robot/params/original_params.yaml')
            time.sleep(1)  # Wait for service calls
            self.get_logger().info(" Parameters reverted. Shutting down.")
            rclpy.shutdown()

    def revert_params_from_yaml(self, yaml_path: str):
        if not Path(yaml_path).exists():
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            return

        with open(yaml_path, 'r') as f:
            raw = yaml.safe_load(f)

        for node, param_dict in raw.items():
            client = self.create_client(SetParameters, f"{node}/set_parameters")
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"Service not available: {node}")
                continue

            request = SetParameters.Request()
            for name, value in param_dict.items():
                param = Parameter()
                param.name = name

                # Set value by type
                if isinstance(value, float):
                    param.value.type = ParameterType.PARAMETER_DOUBLE
                    param.value.double_value = value
                elif isinstance(value, int):
                    param.value.type = ParameterType.PARAMETER_INTEGER
                    param.value.integer_value = value
                elif isinstance(value, bool):
                    param.value.type = ParameterType.PARAMETER_BOOL
                    param.value.bool_value = value
                elif isinstance(value, str):
                    param.value.type = ParameterType.PARAMETER_STRING
                    param.value.string_value = value
                else:
                    self.get_logger().warn(f"Unsupported type for param {name}")
                    continue

                request.parameters.append(param)

            future = client.call_async(request)

            def callback(fut, node=node):
                if fut.result() is not None:
                    self.get_logger().info(f"Reverted parameters for: {node}")
                else:
                    self.get_logger().error(f"Failed to revert parameters for: {node}")

            future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationWithRevert()
    rclpy.spin(node)
    node.destroy_node()
    

if __name__ == '__main__':
    main()
