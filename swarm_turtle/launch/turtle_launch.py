from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name='swarm_turtle'

    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        parameters=[{'geometry': {'width': 1800, 'height': 100}}]
    )

    spawner = Node(
        package=package_name,  # Update with your actual package name
        executable='spawn_turtles.py',
    )

    return LaunchDescription([
        turtle,
        # spawner,
    ])
