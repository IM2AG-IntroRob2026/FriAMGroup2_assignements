from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pen_r_arg = DeclareLaunchArgument('pen_color_r', default_value='255')
    pen_g_arg = DeclareLaunchArgument('pen_color_g', default_value='0')
    pen_b_arg = DeclareLaunchArgument('pen_color_b', default_value='0')
    pen_width_arg = DeclareLaunchArgument('pen_width', default_value='3')

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    boundary_drawer_node = Node(
        package='turtle_boundaries',
        executable='boundary_drawer',
        name='boundary_drawer',
        output='screen',
        parameters=[{
            'pen_color_r': LaunchConfiguration('pen_color_r'),
            'pen_color_g': LaunchConfiguration('pen_color_g'),
            'pen_color_b': LaunchConfiguration('pen_color_b'),
            'pen_width':   LaunchConfiguration('pen_width'),
        }],
    )

    teleop_handler_node = Node(
        package='turtle_boundaries',
        executable='teleop_handler',
        name='teleop_handler',
        output='screen',
        # Open in a separate terminal so the user can type keys
        prefix='xterm -e',
    )

    return LaunchDescription([
        pen_r_arg,
        pen_g_arg,
        pen_b_arg,
        pen_width_arg,
        turtlesim_node,
        boundary_drawer_node,
        teleop_handler_node,
    ])