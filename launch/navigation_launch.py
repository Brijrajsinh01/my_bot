import launch,os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name='my_bot'
    # navigation NODE
    nav=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'scripts','navigation.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    nav
    return launch.LaunchDescription([
        # Launch map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': './src/my_bot/maps/map_normal.yaml'}]
        ),

        # Launch AMCL for localization
        Node(
            package='nav2_bringup',
            executable='amcl',
            name='amcl',
            output='screen'
        ),

        # Launch planner and controller
        Node(
            package='nav2_bringup',
            executable='planner_and_controller',
            name='planner_and_controller',
            output='screen'
        ),
    ])
