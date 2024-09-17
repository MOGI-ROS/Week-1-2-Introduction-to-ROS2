#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__("python_publisher_oop")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)     # Timer period in seconds, not frequency!
        self.i = 0
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.i += 1
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    rclpy.spin(node) # Keeps the node running until it's closed with ctrl+c
    rclpy.shutdown()

if __name__ == "__main__":
    main()