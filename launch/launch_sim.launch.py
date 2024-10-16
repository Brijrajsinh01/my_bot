import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package and directory paths
    package_name = 'my_bot'
    package_share_directory = get_package_share_directory(package_name)

    # Robot State Publisher Launch
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_directory, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo world path
    world = os.path.join(package_share_directory, 'worlds', 'obstacle.world')

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={'world': world}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '1.5708'
        ],
        output='screen'
    )

    # RViz2 configuration file path
    rviz_config_file = os.path.join(package_share_directory, 'config', 'map.rviz')

    # Launch RViz2 with the configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Launch the Navigation2 stack with the params file
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
        )]), launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(package_share_directory, 'config', 'nav2_params.yaml'),
            'map':''
        }.items()
    )

    obstacle_detector_node = Node(
        package=package_name,
        executable='obstacle_detector.py',  # Ensure the executable name is correct
        name='obstacle_detector',
        output='screen',
        emulate_tty=True  # Ensures logs are printed properly in console
    )

    base_link_coordinates_node = Node(
        package=package_name,
        executable='tf_test.py',  # Ensure the executable name is correct
        name='base_coordinates',
        output='screen',
        emulate_tty=True  # Ensures logs are printed properly in console
    )


    # Return the complete LaunchDescription
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        nav2_bringup,
        obstacle_detector_node,
        base_link_coordinates_node
    ])
