#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    cpp_publisher_node = Node(
        package="bme_ros2_tutorials_cpp",
        executable="publisher_cpp",
        name="my_cpp_publisher",
    )

    py_publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_py_publisher",
        remappings=[
            ("topic", "another_topic")
        ]
    )

    py_subscriber_node1 = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber",
        name="my_py_subscriber1",
    )

    py_subscriber_node2 = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber_oop",
        name="my_py_subscriber2",
    )

    cpp_subscriber_node1 = Node(
        package="bme_ros2_tutorials_cpp",
        executable="subscriber_cpp",
        name="my_cpp_subscriber1",
        remappings=[
            ("topic", "another_topic")
        ]
    )

    ld.add_action(cpp_publisher_node)
    ld.add_action(py_publisher_node)
    ld.add_action(py_subscriber_node1)
    ld.add_action(py_subscriber_node2)
    ld.add_action(cpp_subscriber_node1)
    return ld