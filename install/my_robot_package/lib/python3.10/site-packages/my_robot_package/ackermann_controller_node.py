#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
import yaml

class AckermannControllerNode(Node):
    def __init__(self):
        super().__init__('ackermann_controller_node')
        self.declare_parameter('ackermann_controller_config', '/home/ac/ros2_auto_robot/src/my_robot_package/config/ackermann_steering_controller.yaml')
        # Provide the default value and type
        config_file = self.get_parameter('ackermann_controller_config').value

        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        self.front_left_wheel = config['ackermann_controller']['front_left_wheel']
        self.front_right_wheel = config['ackermann_controller']['front_right_wheel']
        self.rear_left_wheel = config['ackermann_controller']['rear_left_wheel']
        self.rear_right_wheel = config['ackermann_controller']['rear_right_wheel']
        self.wheel_separation = config['ackermann_controller']['wheel_separation']
        self.wheel_radius = config['ackermann_controller']['wheel_radius']
        self.wheel_base = config['ackermann_controller']['wheel_base']
        self.steer_angle_limit = config['ackermann_controller']['steer_angle_limit']

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.steering_publisher = self.create_publisher(
            JointControllerState,
            self.front_left_wheel + '/state',
            qos_profile
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )

    def cmd_vel_callback(self, msg):
        steering_state = JointControllerState()
        steering_state.header.stamp = self.get_clock().now().to_msg()
        steering_state.set_point = msg.angular.z
        self.steering_publisher.publish(steering_state)

def main(args=None):
    rclpy.init(args=args)
    ackermann_controller_node = AckermannControllerNode()
    rclpy.spin(ackermann_controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
