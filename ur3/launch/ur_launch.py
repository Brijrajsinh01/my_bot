import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('joint_limit_params', description='YAML file containing the joint limit values'),
        DeclareLaunchArgument('kinematics_params', description='YAML file containing the robot\'s kinematic parameters'),
        DeclareLaunchArgument('physical_params', description='YAML file containing the physical parameters of the robots'),
        DeclareLaunchArgument('visual_params', description='YAML file containing the visual model of the robots'),
        DeclareLaunchArgument('transmission_hw_interface', default_value='hardware_interface/PositionJointInterface'),
        DeclareLaunchArgument('safety_limits', default_value='false', description='Enable the safety limits controller'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15', description='The lower/upper limits in the safety controller'),
        DeclareLaunchArgument('safety_k_position', default_value='20', description='Used to set k position in the safety controller'),
        DeclareLaunchArgument('robot_model'),

        # Node to load the UR xacro
        Node(
            package='xacro',
            executable='xacro',
            output='log',  # Change to 'screen' if you want to see output on the terminal
            name='ur_description',
            arguments=[
                '$(find ur_description)/urdf/ur.xacro',
                'robot_model:=$(arg robot_model)',
                'joint_limit_params:=$(arg joint_limit_params)',
                'kinematics_params:=$(arg kinematics_params)',
                'physical_params:=$(arg physical_params)',
                'visual_params:=$(arg visual_params)',
                'transmission_hw_interface:=$(arg transmission_hw_interface)',
                'safety_limits:=$(arg safety_limits)',
                'safety_pos_margin:=$(arg safety_pos_margin)',
                'safety_k_position:=$(arg safety_k_position)'
            ],
        ),
    ])
