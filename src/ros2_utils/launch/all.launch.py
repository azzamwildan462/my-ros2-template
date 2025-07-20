import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

def generate_launch_description():
    
    SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        output='screen',
        respawn=True,
    )

    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {
                "ui_root_path": os.path.join(ws_path,"src/web_ui/src")
            },
        ],
        output="screen",
        respawn=True,
    )

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://172.30.37.21:8086",
            "INFLUXDB_USERNAME": "awm462",
            "INFLUXDB_PASSWORD": "wildan462",
            "INFLUXDB_ORG": "awmawm",
            "INFLUXDB_BUCKET": "ujiCoba",
            "ROBOT_NAME": "gh_template",
        }],
        output="screen",
        respawn=True,
    )

    watchdog_node = Node(
        package="ros2_utils",
        executable="watchdog_node.py",
        name="watchdog_node",
        parameters=[{
                'container_name': '/main_container',
                'watched_nodes': [
                    "{'name': 'master', 'plugin': 'Master', 'package': 'master'}",
                    "{'name': 'WiFi_Control', 'plugin': 'WiFIControl', 'package': 'communication'}"
                ]
        }],
        output="screen",
        respawn=True,
    )

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        prefix=['xterm -e'],
        output="screen",
        respawn=True,
    )

    # Compose nodes into a container
    # =================================================================================================================

    wifi_control = ComposableNode(
        name="WiFi_Control",
        plugin="WiFIControl",
        package="communication",
        parameters=[
            {
                "hotspot_ssid": "gh_template",
                "hotspot_password": "gh_template",
            },
        ],
    )

    master = ComposableNode(
        name='master',
        plugin='Master',
        package='master',
    )


    # Container
    # =============================================================

    main_container = ComposableNodeContainer(
        name='main_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            wifi_control,
            master,
        ],
        output='screen',
    )


    return LaunchDescription(
        [
            # rosapi_node,
            # ui_server,
            # rosbridge_server, 

            # telemetry,
            # keyboard_input,

            main_container,
            watchdog_node,
        ]
    )
