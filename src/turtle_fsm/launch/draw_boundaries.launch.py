"""Launch turtlesim and the draw_boundaries FSM controller."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch turtlesim and open the boundary controller in its own terminal."""
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
        ),
        Node(
            package='turtle_fsm',
            executable='draw_boundaries',
            name='draw_boundaries_node',
        ),
    ])
