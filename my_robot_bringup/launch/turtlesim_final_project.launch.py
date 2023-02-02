from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim1"
    )


    
    closest_turtle_controller_node = Node(
        package="turtlesim_project",
        executable="turtle_controller",
        name="turtle_master",
        parameters=[
            {"turtle_master_name": "turtle1"},
            {"flag": True}
        ]
    )
    closest_turtle_spawner_node = Node(
        package="turtlesim_project",
        executable="turtle_spawner",
        name="turtle_spawner",
        parameters=[
            {"spawn_frequency": 0.8}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(closest_turtle_controller_node)
    ld.add_action(closest_turtle_spawner_node)
    return ld
