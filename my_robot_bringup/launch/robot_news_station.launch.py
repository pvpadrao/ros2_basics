from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_news_station1_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station1",
        parameters=[
            {"robot_name": "robot1"}
        ]
    )

    robot_news_station2_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station2",
        parameters=[
            {"robot_name": "robot2"}
        ]
    )

    robot_news_station3_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station3",
        parameters=[
            {"robot_name": "robot3"}
        ]
    )

    robot_news_station4_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station4",
        parameters=[
            {"robot_name": "robot4"}
        ]
    )

    robot_news_station5_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station5",
        parameters=[
            {"robot_name": "robot5"}
        ]
    )
    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone"
    )
    
    ld.add_action(robot_news_station1_node)
    ld.add_action(robot_news_station2_node)
    ld.add_action(robot_news_station3_node)
    ld.add_action(robot_news_station4_node)
    ld.add_action(robot_news_station5_node)
    ld.add_action(smartphone_node)
    return ld