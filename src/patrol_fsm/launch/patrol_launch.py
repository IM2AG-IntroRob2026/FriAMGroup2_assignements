from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare overridable arguments ──────────────────────────────────────────
    num_laps_arg    = DeclareLaunchArgument('num_laps',       default_value='1')
    side_length_arg = DeclareLaunchArgument('side_length',    default_value='1.0')
    speed_arg       = DeclareLaunchArgument('patrol_speed',   default_value='0.3')
    siren_arg       = DeclareLaunchArgument('siren_duration', default_value='3.0')

    num_laps       = LaunchConfiguration('num_laps')
    side_length    = LaunchConfiguration('side_length')
    patrol_speed   = LaunchConfiguration('patrol_speed')
    siren_duration = LaunchConfiguration('siren_duration')

    # ── Nodes ──────────────────────────────────────────────────────────────────
    square_server = Node(
        package='robot_square_controller',
        executable='square_action_server',
        name='square_action_server',
        parameters=[{
            'side_length': side_length,
            'speed': patrol_speed,
        }],
    )

    patrol = Node(
        package='patrol_fsm',
        executable='patrol_node',
        name='patrol_node',
        parameters=[{
            'num_laps':       num_laps,
            'side_length':    side_length,
            'patrol_speed':   patrol_speed,
            'siren_duration': siren_duration,
        }],
    )

    return LaunchDescription([
        num_laps_arg,
        side_length_arg,
        speed_arg,
        siren_arg,
        square_server,
        patrol,
    ])
