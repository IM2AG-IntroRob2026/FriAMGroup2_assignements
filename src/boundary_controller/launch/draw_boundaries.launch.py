from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments (pen color & width) ──────────────────────────────
        DeclareLaunchArgument(
            'pen_color_r', default_value='255',
            description='Pen red channel (0–255)'),
        DeclareLaunchArgument(
            'pen_color_g', default_value='0',
            description='Pen green channel (0–255)'),
        DeclareLaunchArgument(
            'pen_color_b', default_value='0',
            description='Pen blue channel (0–255)'),
        DeclareLaunchArgument(
            'pen_width', default_value='3',
            description='Pen width in pixels'),

        # ── Turtlesim simulator ────────────────────────────────────────────────
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),

        # ── Autonomous boundary-drawing FSM ────────────────────────────────────
        Node(
            package='boundary_controller',
            executable='boundary_fsm',
            name='boundary_fsm',
            output='screen',
            parameters=[{
                'pen_color_r': LaunchConfiguration('pen_color_r'),
                'pen_color_g': LaunchConfiguration('pen_color_g'),
                'pen_color_b': LaunchConfiguration('pen_color_b'),
                'pen_width':   LaunchConfiguration('pen_width'),
            }],
        ),

        # ── Keyboard teleop – opened in its own xterm window ──────────────────
        # xterm is required: apt-get install -y xterm
        Node(
            package='boundary_controller',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e',
        ),
    ])
