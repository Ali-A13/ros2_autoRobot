import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 0.1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0
        self.angular_speed = 0.9  # radians per second
        self.linear_speed = 0.2  # meters per second

    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = self.angular_speed
        self.publisher_.publish(twist_msg)
        self.angle += self.angular_speed * 0.1  # integrate angle
        if self.angle >= (2 * 3.14159):  # complete circle
            self.angle = 0.0


def main(args=None):
    rclpy.init(args=args)
    circle_driver = CircleDriver()
    rclpy.spin(circle_driver)
    circle_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
