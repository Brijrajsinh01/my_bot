import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Specify the path to your xacro file
    xacro_file = os.path.join(get_package_share_directory('ur3'), 'ur_description','arctos_urdf.xacro')

    # Create a temporary URDF file from the xacro file
    temp_urdf_file = os.path.join(get_package_share_directory('ur3'), 'ur_description','arctos_urdf.urdf')
    xacro_cmd = f'xacro {xacro_file} > {temp_urdf_file}'
    os.system(xacro_cmd)

    # Create a robot_state_publisher node
    params = {'robot_description': open(temp_urdf_file, 'r').read(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Start rviz
    rviz=Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz',
            arguments=['-d', os.path.join(get_package_share_directory('ur3'), 'ur_description', 'ur3.rviz')]
        )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        rviz
    ])


# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node

# def generate_launch_description():

#     # Check if we're told to use sim time
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # Specify the path to your URDF file
#     urdf_file = os.path.join(get_package_share_directory('ur3'), 'ur_description', 'onrobot_screwdriver.urdf')

#     # Create a robot_state_publisher node
#     with open(urdf_file, 'rb') as file:
#         urdf_content = file.read().decode('utf-16')  # Adjust the encoding if needed

#     params = {'robot_description': urdf_content, 'use_sim_time': use_sim_time}
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[params]
#     )

#     node_joint_state_publisher = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         output='screen'
#     )

#     # Start rviz
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         name='rviz',
#         arguments=['-d', os.path.join(get_package_share_directory('ur3'), 'ur_description', 'ur3.rviz')]
#     )

#     # Launch!
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use sim time if true'),

#         node_robot_state_publisher,
#         node_joint_state_publisher,
#         rviz
#     ])
