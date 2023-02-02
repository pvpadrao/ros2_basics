from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        parameters=[
            {"number_to_publish": 5},
            {"frequency": 2.0}
        ]
    )

    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone"
    )

    ld.add_action(number_publisher_node)
    ld.add_action(smartphone_node)

    return ld