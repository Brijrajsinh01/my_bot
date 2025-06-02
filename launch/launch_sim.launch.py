import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package and directory paths
    package_name = 'my_bot'
    package_share_directory = get_package_share_directory(package_name)

    # ✅ Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_directory, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ✅ Gazebo world path
    world = os.path.join(package_share_directory, 'worlds', 'wall.world')

    # ✅ Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={'world': world}.items()
    )

    # ✅ Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '1.5708'
        ],
        output='log'  # Directing output to log
    )

    # ✅ RViz2 configuration file path
    rviz_config_file = os.path.join(package_share_directory, 'config', 'map.rviz')

    # ✅ Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',  # Directing output to log
        arguments=['-d', rviz_config_file],
    )

    # ✅ FIX: Add SLAM Toolbox (Sync Mode)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ✅ Get path to your nav2_params.yaml
    nav2_params_path = os.path.join(package_share_directory, 'config', 'nav2_params.yaml')

    # ✅ Modified Nav2 Launch with params file
    nav2_bringup = ExecuteProcess(
        cmd=[
            "ros2", "launch", "nav2_bringup", "bringup_launch.py",
            "use_sim_time:=true",
            "map:=dummy.yaml",
            f"params_file:={nav2_params_path}"  # ✅ Added params file
        ],
        output="log"  # Directing output to log
    )

    # ✅ ADD A DELAY for Nav2 (ensures SLAM publishes map first)
    delayed_nav2_bringup = TimerAction(
        period=20.0,  # ✅ Wait 20 seconds before starting Nav2
        actions=[nav2_bringup]
    )

    # ✅ Additional node
    base_link_coordinates_node = Node(
        package=package_name,
        executable='tf_test.py',
        name='base_coordinates',
        output='log',  # Directing output to log
        emulate_tty=True
    )

    # Directly control logging for all nodes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        slam_toolbox,  # ✅ SLAM Toolbox added correctly
        delayed_nav2_bringup,  # ✅ Nav2 starts AFTER SLAM
        base_link_coordinates_node
    ])
