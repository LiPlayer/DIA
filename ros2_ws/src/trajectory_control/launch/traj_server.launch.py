from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'time_forward',
            default_value='1.0',
            description='Look-ahead time for yaw calculation'
        ),
        
        Node(
            package='trajectory_control',
            executable='traj_server_px4',
            name='traj_server_px4',
            output='screen',
            parameters=[{
                'time_forward': LaunchConfiguration('time_forward'),
            }],
            remappings=[
                ('planning/bspline', '/planning/bspline'),
            ],
        ),
    ])
