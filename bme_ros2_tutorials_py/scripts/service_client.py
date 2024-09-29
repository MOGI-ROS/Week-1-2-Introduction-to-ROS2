#!/usr/bin/env python3
import sys

from bme_ros2_tutorials_interfaces.srv import CustomCalc # Import our own custom service

import rclpy
from rclpy.node import Node

class MyServiceClientAsync(Node):
    def __init__(self):
        super().__init__('my_service_client_async')
        # Create a service client
        self.cli = self.create_client(CustomCalc, 'custom_calc')
        # Check if service server is online
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create the service request
        self.req = CustomCalc.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service with the 2 parameters in the request and return result
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    my_service_client = MyServiceClientAsync()
    future = my_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    # Spin only until response arrives
    rclpy.spin_until_future_complete(my_service_client, future)
    response = future.result()
    my_service_client.get_logger().info(
        'Result of custom_calc: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.result))

    my_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()