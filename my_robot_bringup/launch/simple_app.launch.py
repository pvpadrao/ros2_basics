import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
             get_package_share_directory('my_robot_bringup'),
             'config',
             'params.yaml'
             )

    simple_launch = Node(
        package="my_py_pkg",
        executable="py_node",
        parameters=[config]
    )

    ld.add_action(simple_launch)

    return ld