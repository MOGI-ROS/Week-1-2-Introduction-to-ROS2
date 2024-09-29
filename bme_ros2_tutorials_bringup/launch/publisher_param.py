#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    cpp_publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher_with_param",
        name="my_publisher",
        parameters=[{"published_text": "Parameter_from_launch"},
                    {"timer_period": 0.5}]
    )


    ld.add_action(cpp_publisher_node)
    return ld