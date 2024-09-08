import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}]: {message}"


def generate_launch_description():
    # Declare launch arguments for DDS transport settings
    declare_transport_type = DeclareLaunchArgument(
        'transport_type',
        default_value='shm',
        description='Transport type for DDS (e.g., udp, shm)'
    )

    server = Node(
        package="basic_flask",
        executable="server.py",
        name="server",
        output="screen",
        emulate_tty=True,
        respawn=True,
        parameters=[{
            'rtps_transport': LaunchConfiguration('transport_type')
        }],
    )
    return LaunchDescription([declare_transport_type, server])