import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.angular_speed = 0.9  # radians per second
        self.linear_speed = 0.3  # meters per second

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        twist_msg = Twist()

        if min_distance < 1.0:  # obstacle detected within 1 meter
            twist_msg.angular.z = self.angular_speed
        else:
            twist_msg.linear.x = self.linear_speed

        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
