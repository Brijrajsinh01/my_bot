import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    slam_toolbox_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml',
    )
    
    return LaunchDescription([
        
        Node(
            
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox_node',
            output='screen',
            arguments=[
                '-p', slam_toolbox_params,
                '--use_sim_time'
            ]
        ),
    ])
