#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__("python_publisher_with_parameter")
        self.declare_parameter("published_text", "MOGI")             # Add a parameter with a default value
        self.declare_parameter("timer_period", 1.0)                  # Add the timer_period parameter with default 1s
        self.timer_period = self.get_parameter('timer_period').value # Get the startup value of the timer_period
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Use the startup value of self.timer_period to start a timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Set a callback to listen for changes to the 'timer_period' parameter
        self.add_on_set_parameters_callback(self.update_timer_period_callback)
        self.i = 0
        self.msg = String()
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):
        self.text_ = self.get_parameter("published_text").value # Copy the parameter value into the text_ variable
        self.msg.data = f"{self.text_}: {self.i}"               # use the text_ variable for the String message
        self.i += 1
        self.get_logger().info(f'Publishing: "{self.msg.data}"')
        self.publisher_.publish(self.msg)

    def update_timer_period_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'timer_period' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                new_period = param.value
                self.get_logger().info(f'Updating timer period to {new_period} seconds')
                # Cancel the old timer and create a new one with the updated period
                self.timer.cancel()                                              # Cancel the existing timer
                self.timer = self.create_timer(new_period, self.timer_callback)  # Create a new timer
                return result
        # Return success, so updates are seen via get_parameter()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()