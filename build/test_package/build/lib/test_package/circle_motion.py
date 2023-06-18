import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.angle_ = 0.0

    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Linear velocity in meters per second
        twist_msg.angular.z = 0.5  # Angular velocity in radians per second
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    circle_driver = CircleDriver()
    rclpy.spin(circle_driver)
    circle_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
