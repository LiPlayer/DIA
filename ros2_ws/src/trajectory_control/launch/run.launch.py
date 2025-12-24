from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'time_forward',
            default_value='1.0',
            description='Look-ahead time for yaw calculation'
        ),
        
        Node(
            package='trajectory_control',
            executable='trajectory_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'time_forward': LaunchConfiguration('time_forward'),
            }],
            remappings=[
                ('planning/bspline', '/planning/bspline'),
            ],
        ),
    ])
