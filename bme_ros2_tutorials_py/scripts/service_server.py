#!/usr/bin/env python3
from bme_ros2_tutorials_interfaces.srv import CustomCalc # Import our own custom service

import rclpy
from rclpy.node import Node

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')
        # Create a service server with a callback function
        self.srv = self.create_service(CustomCalc, 'custom_calc', self.custom_calc_callback)

    # A service callback has request and response parameters
    def custom_calc_callback(self, request, response):
        response.result = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    my_service_server = MyService()
    rclpy.spin(my_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()