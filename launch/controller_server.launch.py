from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    fromll_service = LaunchConfiguration("fromll_service")
    fromll_service_fallback = LaunchConfiguration("fromll_service_fallback")
    fromll_wait_timeout_s = LaunchConfiguration("fromll_wait_timeout_s")
    map_frame = LaunchConfiguration("map_frame")
    gps_topic = LaunchConfiguration("gps_topic")
    cmd_vel_safe_topic = LaunchConfiguration("cmd_vel_safe_topic")
    brake_topic = LaunchConfiguration("brake_topic")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    teleop_cmd_topic = LaunchConfiguration("teleop_cmd_topic")
    brake_publish_count = LaunchConfiguration("brake_publish_count")
    brake_publish_interval_s = LaunchConfiguration("brake_publish_interval_s")
    manual_cmd_timeout_s = LaunchConfiguration("manual_cmd_timeout_s")
    manual_watchdog_hz = LaunchConfiguration("manual_watchdog_hz")
    nav_telemetry_hz = LaunchConfiguration("nav_telemetry_hz")
    telemetry_topic = LaunchConfiguration("telemetry_topic")
    set_goal_service = LaunchConfiguration("set_goal_service")
    cancel_goal_service = LaunchConfiguration("cancel_goal_service")
    brake_service = LaunchConfiguration("brake_service")
    set_manual_mode_service = LaunchConfiguration("set_manual_mode_service")
    get_state_service = LaunchConfiguration("get_state_service")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fromll_service", default_value="/fromLL"),
            DeclareLaunchArgument(
                "fromll_service_fallback", default_value="/navsat_transform/fromLL"
            ),
            DeclareLaunchArgument("fromll_wait_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("brake_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("teleop_cmd_topic", default_value="/cmd_vel_teleop"),
            DeclareLaunchArgument("brake_publish_count", default_value="5"),
            DeclareLaunchArgument("brake_publish_interval_s", default_value="0.1"),
            DeclareLaunchArgument("manual_cmd_timeout_s", default_value="0.4"),
            DeclareLaunchArgument("manual_watchdog_hz", default_value="10.0"),
            DeclareLaunchArgument("nav_telemetry_hz", default_value="5.0"),
            DeclareLaunchArgument(
                "telemetry_topic", default_value="/nav_command_server/telemetry"
            ),
            DeclareLaunchArgument(
                "set_goal_service", default_value="/nav_command_server/set_goal_ll"
            ),
            DeclareLaunchArgument(
                "cancel_goal_service", default_value="/nav_command_server/cancel_goal"
            ),
            DeclareLaunchArgument(
                "brake_service", default_value="/nav_command_server/brake"
            ),
            DeclareLaunchArgument(
                "set_manual_mode_service",
                default_value="/nav_command_server/set_manual_mode",
            ),
            DeclareLaunchArgument(
                "get_state_service", default_value="/nav_command_server/get_state"
            ),
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
                        "max_reverse_mps": 1.30,
                        "max_abs_angular_z": 0.4,
                        "vx_deadband_mps": 0.10,
                        "vx_min_effective_mps": 0.75,
                        "invert_steer_from_cmd_vel": True,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="nav_command_server",
                name="nav_command_server",
                output="screen",
                parameters=[
                    {
                        "fromll_service": fromll_service,
                        "fromll_service_fallback": fromll_service_fallback,
                        "fromll_wait_timeout_s": ParameterValue(
                            fromll_wait_timeout_s, value_type=float
                        ),
                        "map_frame": map_frame,
                        "gps_topic": gps_topic,
                        "cmd_vel_safe_topic": cmd_vel_safe_topic,
                        "brake_topic": brake_topic,
                        "manual_cmd_topic": manual_cmd_topic,
                        "teleop_cmd_topic": teleop_cmd_topic,
                        "brake_publish_count": ParameterValue(
                            brake_publish_count, value_type=int
                        ),
                        "brake_publish_interval_s": ParameterValue(
                            brake_publish_interval_s, value_type=float
                        ),
                        "manual_cmd_timeout_s": ParameterValue(
                            manual_cmd_timeout_s, value_type=float
                        ),
                        "manual_watchdog_hz": ParameterValue(
                            manual_watchdog_hz, value_type=float
                        ),
                        "nav_telemetry_hz": ParameterValue(
                            nav_telemetry_hz, value_type=float
                        ),
                        "telemetry_topic": telemetry_topic,
                        "set_goal_service": set_goal_service,
                        "cancel_goal_service": cancel_goal_service,
                        "brake_service": brake_service,
                        "set_manual_mode_service": set_manual_mode_service,
                        "get_state_service": get_state_service,
                    }
                ],
            ),
        ]
    )
