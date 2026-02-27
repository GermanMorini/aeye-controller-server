from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="controller_server",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/serial0",
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
                        "mode": "auto",
                        "invert_steer_from_cmd_vel": True,
                        "ws_enabled": True,
                        "ws_host": "0.0.0.0",
                        "ws_port": 8765,
                    }
                ],
            )
        ]
    )
