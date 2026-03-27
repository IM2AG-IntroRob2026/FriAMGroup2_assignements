from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='toto',
        ),
        Node(
            package='turtle_fsm',
            executable='fsm_node',
            name='turtle_fsm1_node',
        ),
    ])
