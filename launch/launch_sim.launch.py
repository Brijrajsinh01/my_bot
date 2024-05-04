import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' 
    package_share_directory = get_package_share_directory(package_name)


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': './src/my_bot/worlds/obstacle.world'}.items()  # Specify the world file name here.
             )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot',
                                '-x', '0.0',  # specify the X-coordinate of the initial position
                                '-y', '0.0',  # specify the Y-coordinate of the initial position
                                '-z', '0.0',  # specify the Z-coordinate of the initial position
                                '-Y', '1.5708'  # specify the Yaw (orientation around the vertical axis) in radians
                                ],
                        output='screen')
    
        # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity_1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description_1',
                                '-entity', 'my_bot_1',
                                '-x', '1.0',  # specify the X-coordinate of the initial position
                                '-y', '0.0',  # specify the Y-coordinate of the initial position
                                '-z', '0.0',  # specify the Z-coordinate of the initial position
                                '-Y', '1.5708'  # specify the Yaw (orientation around the vertical axis) in radians
                                ],
                        output='screen')


    nav_l = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','script_launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    

    # Rviz2 NODE
    rviz_node= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', './src/my_bot/config/map.rviz'],

    )


    # Launch them all!
    return LaunchDescription([
        # nav_l,
        rsp,
        gazebo,
        rviz_node,
        spawn_entity,
        # spawn_entity_1,
        # slam_toolbox,
        
        
    ])
