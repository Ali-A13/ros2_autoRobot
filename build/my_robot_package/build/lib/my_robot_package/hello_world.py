#!/usr/bin/env python3

import rclpy

def hello_world():
    pass

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("helloWorld_node")
    print("HELLO WORLD!!!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    #hello_world()

if __name__ == '__main__':
    main()