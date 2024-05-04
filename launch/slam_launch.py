#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    package_name = 'my_bot'

    # Declare the launch argument for sim
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='True',
        description='Whether to use simulation time'
    )

    # slam_toolbox_params = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'mapper_params_online_async.yaml',
    # )
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'mapper_params_online_async.yaml']
    )
    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'foxy': 
        slam_param_name = 'params_file'

    return LaunchDescription([
        sim_arg,  # Include the declaration of 'sim' argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                slam_param_name: slam_config_path
            }.items()
        ),
    ])
