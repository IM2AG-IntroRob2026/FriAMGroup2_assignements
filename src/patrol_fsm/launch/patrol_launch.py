from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_square_controller',
            executable='square_action_server',
            name='square_action_server',
            parameters=[{'side_length': 2.0, 'speed': 0.3}],
        ),
        Node(
            package='patrol_fsm',
            executable='patrol_node',
            name='patrol_node',
            parameters=[{
                'num_laps': 3,
                'side_length': 2.0,
                'patrol_speed': 0.3,
                'siren_duration': 3.0,
            }],
        ),
    ])
