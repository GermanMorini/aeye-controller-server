from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="vehicle_controller_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "transport_backend": "sim_gazebo",
                        "serial_port": "/dev/null",
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
                        "max_reverse_mps": 1.30,
                        "max_abs_angular_z": 0.4,
                        "vx_deadband_mps": 0.01,
                        "vx_min_effective_mps": 0.5,
                        "invert_steer_from_cmd_vel": True,
                        "sim_cmd_vel_topic": "/cmd_vel_gazebo",
                        "sim_odom_topic": "/odom_raw",
                        "sim_joint_states_topic": "/joint_states",
                        "sim_front_left_steer_joint": "front_left_steer_joint",
                        "sim_front_right_steer_joint": "front_right_steer_joint",
                        "sim_max_steering_angle_rad": 0.5235987756,
                        "sim_telemetry_timeout_s": 0.5,
                        "sim_invert_actuation_steer_sign": True,
                        "sim_invert_measured_steer_sign": True,
                    }
                ],
            ),
        ]
    )
