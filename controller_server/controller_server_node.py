from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .control_logic import (
    DesiredCommand,
    command_from_cmd_vel,
    safe_command,
    select_effective_command,
)


def _load_comms_client_class():
    from controller_server.rpy_esp32_comms.transport import CommsClient

    return CommsClient


class ControllerServerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_server")

        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_tx_hz", 50.0)
        self.declare_parameter("max_speed_mps", 4.0)
        self.declare_parameter("max_reverse_mps", 1.30)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("telemetry_pub_hz", 10.0)
        self.declare_parameter("auto_timeout_s", 0.7)
        self.declare_parameter("max_abs_angular_z", 0.4)
        self.declare_parameter("vx_deadband_mps", 0.10)
        self.declare_parameter("vx_min_effective_mps", 0.75)
        self.declare_parameter("reverse_brake_pct", 20)
        self.declare_parameter("invert_steer_from_cmd_vel", False)
        self.declare_parameter("auto_drive_enabled", True)
        self.declare_parameter("estop_brake_pct", 100)

        self._serial_port = self.get_parameter("serial_port").value
        self._serial_baud = int(self.get_parameter("serial_baud").value)
        self._serial_tx_hz = float(self.get_parameter("serial_tx_hz").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._max_reverse_mps = float(self.get_parameter("max_reverse_mps").value)
        if self._max_reverse_mps < 0.0:
            self.get_logger().warn(
                f"Invalid max_reverse_mps={self._max_reverse_mps:.3f}; clamping to 0.0"
            )
            self._max_reverse_mps = 0.0
        self._control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self._telemetry_pub_hz = max(1.0, float(self.get_parameter("telemetry_pub_hz").value))
        self._auto_timeout_s = float(self.get_parameter("auto_timeout_s").value)
        self._max_abs_angular_z = float(self.get_parameter("max_abs_angular_z").value)
        self._vx_deadband_mps = float(self.get_parameter("vx_deadband_mps").value)
        if self._vx_deadband_mps < 0.0:
            self.get_logger().warn(
                f"Invalid vx_deadband_mps={self._vx_deadband_mps:.3f}; clamping to 0.0"
            )
            self._vx_deadband_mps = 0.0
        self._vx_min_effective_mps = float(self.get_parameter("vx_min_effective_mps").value)
        if self._vx_min_effective_mps < 0.0:
            self.get_logger().warn(
                f"Invalid vx_min_effective_mps={self._vx_min_effective_mps:.3f}; clamping to 0.0"
            )
            self._vx_min_effective_mps = 0.0
        if self._vx_min_effective_mps > self._max_speed_mps:
            self.get_logger().warn(
                "vx_min_effective_mps greater than max_speed_mps; "
                f"using max_speed_mps={self._max_speed_mps:.3f} as effective minimum"
            )
            self._vx_min_effective_mps = self._max_speed_mps
        self._reverse_brake_pct = int(self.get_parameter("reverse_brake_pct").value)
        self._invert_steer_from_cmd_vel = bool(self.get_parameter("invert_steer_from_cmd_vel").value)
        self._auto_drive_enabled = bool(self.get_parameter("auto_drive_enabled").value)
        self._estop_brake_pct = int(self.get_parameter("estop_brake_pct").value)

        self._state_lock = threading.Lock()
        self._auto_cmd = safe_command()
        self._auto_stamp_s = 0.0
        self._last_source = "init"

        CommsClient = _load_comms_client_class()
        self._client = CommsClient(
            port=self._serial_port,
            baud=self._serial_baud,
            tx_hz=self._serial_tx_hz,
            max_speed_mps=self._max_speed_mps,
            max_reverse_mps=self._max_reverse_mps,
        )
        self._client.start()

        self.create_subscription(Twist, "/cmd_vel_safe", self._on_cmd_vel_safe, 10)
        self._status_pub = self.create_publisher(String, "/controller/status", 10)
        self._telemetry_pub = self.create_publisher(String, "/controller/telemetry", 10)

        self.create_timer(1.0 / self._control_hz, self._control_tick)
        self.create_timer(1.0 / self._telemetry_pub_hz, self._telemetry_tick)

        self.get_logger().info(
            "controller_server ready "
            f"(serial={self._serial_port}@{self._serial_baud}, source=/cmd_vel_safe)"
        )

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        cmd = command_from_cmd_vel(
            linear_x=msg.linear.x,
            angular_z=msg.angular.z,
            max_speed_mps=self._max_speed_mps,
            max_reverse_mps=self._max_reverse_mps,
            vx_deadband_mps=self._vx_deadband_mps,
            vx_min_effective_mps=self._vx_min_effective_mps,
            max_abs_angular_z=self._max_abs_angular_z,
            invert_steer=self._invert_steer_from_cmd_vel,
            auto_drive_enabled=self._auto_drive_enabled,
            reverse_brake_pct=self._reverse_brake_pct,
        )
        with self._state_lock:
            self._auto_cmd = cmd
            self._auto_stamp_s = time.monotonic()
        self.get_logger().info(
            "cmd_vel_safe rx "
            f"linear_x={msg.linear.x:.3f} angular_z={msg.angular.z:.3f} -> "
            f"drive={int(cmd.drive_enabled)} estop={int(cmd.estop)} "
            f"speed_mps={cmd.speed_mps:.3f} steer_pct={cmd.steer_pct} brake_pct={cmd.brake_pct}"
        )

    def _apply_to_controller(self, cmd: DesiredCommand) -> None:
        self._client.set_drive_enabled(bool(cmd.drive_enabled))
        self._client.set_estop(bool(cmd.estop))
        self._client.set_speed_mps(float(cmd.speed_mps))
        self._client.set_steer_pct(int(cmd.steer_pct))
        self._client.set_brake_pct(int(cmd.brake_pct))

    def _control_tick(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            auto_cmd = self._auto_cmd
            auto_stamp_s = self._auto_stamp_s

        result = select_effective_command(
            now_s=now,
            auto_cmd=auto_cmd,
            auto_stamp_s=auto_stamp_s,
            auto_timeout_s=self._auto_timeout_s,
        )
        cmd = result.command

        if cmd.estop:
            cmd = DesiredCommand(
                drive_enabled=False,
                estop=True,
                speed_mps=0.0,
                steer_pct=0,
                brake_pct=max(cmd.brake_pct, self._estop_brake_pct),
            )
            source = "estop"
        else:
            source = result.source

        self._apply_to_controller(cmd)
        self._last_source = source

        status = {
            "mode": "auto",
            "source": source,
            "fresh": result.fresh,
            "global_estop": False,
            "command": asdict(cmd),
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _telemetry_tick(self) -> None:
        telemetry = self._client.get_latest_telemetry()
        stats = self._client.get_stats()
        payload = {
            "source": self._last_source,
            "telemetry": telemetry.as_dict() if telemetry is not None else None,
            "stats": asdict(stats),
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._telemetry_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._client.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
