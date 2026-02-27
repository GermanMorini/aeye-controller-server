from __future__ import annotations

import asyncio
import json
import threading
import time
from dataclasses import asdict
from typing import Any, Dict, Optional

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
    from controller_server.vendor.rpy_esp32_comms.transport import CommsClient

    return CommsClient


class ControllerServerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_server")

        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_tx_hz", 50.0)
        self.declare_parameter("max_speed_mps", 4.0)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("telemetry_pub_hz", 10.0)
        self.declare_parameter("mode", "auto")
        self.declare_parameter("manual_timeout_s", 0.7)
        self.declare_parameter("auto_timeout_s", 0.7)
        self.declare_parameter("max_abs_angular_z", 0.8)
        self.declare_parameter("reverse_brake_pct", 20)
        self.declare_parameter("auto_drive_enabled", True)
        self.declare_parameter("estop_brake_pct", 100)
        self.declare_parameter("ws_enabled", True)
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8765)

        self._serial_port = self.get_parameter("serial_port").value
        self._serial_baud = int(self.get_parameter("serial_baud").value)
        self._serial_tx_hz = float(self.get_parameter("serial_tx_hz").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self._telemetry_pub_hz = max(1.0, float(self.get_parameter("telemetry_pub_hz").value))
        self._mode = str(self.get_parameter("mode").value).strip().lower()
        if self._mode not in ("manual", "auto"):
            self.get_logger().warn(f"Invalid mode '{self._mode}', forcing auto")
            self._mode = "auto"
        self._manual_timeout_s = float(self.get_parameter("manual_timeout_s").value)
        self._auto_timeout_s = float(self.get_parameter("auto_timeout_s").value)
        self._max_abs_angular_z = float(self.get_parameter("max_abs_angular_z").value)
        self._reverse_brake_pct = int(self.get_parameter("reverse_brake_pct").value)
        self._auto_drive_enabled = bool(self.get_parameter("auto_drive_enabled").value)
        self._estop_brake_pct = int(self.get_parameter("estop_brake_pct").value)
        self._ws_enabled = bool(self.get_parameter("ws_enabled").value)
        self._ws_host = str(self.get_parameter("ws_host").value)
        self._ws_port = int(self.get_parameter("ws_port").value)

        self._state_lock = threading.Lock()
        self._manual_cmd = safe_command()
        self._manual_stamp_s = 0.0
        self._auto_cmd = safe_command()
        self._auto_stamp_s = 0.0
        self._global_estop = False
        self._last_source = "init"

        CommsClient = _load_comms_client_class()
        self._client = CommsClient(
            port=self._serial_port,
            baud=self._serial_baud,
            tx_hz=self._serial_tx_hz,
            max_speed_mps=self._max_speed_mps,
        )
        self._client.start()

        self.create_subscription(Twist, "/cmd_vel_safe", self._on_cmd_vel_safe, 10)
        self._status_pub = self.create_publisher(String, "/controller/status", 10)
        self._telemetry_pub = self.create_publisher(String, "/controller/telemetry", 10)

        self.create_timer(1.0 / self._control_hz, self._control_tick)
        self.create_timer(1.0 / self._telemetry_pub_hz, self._telemetry_tick)

        self._ws_loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._ws_server = None
        if self._ws_enabled:
            self._start_ws_server()

        self.get_logger().info(
            "controller_server ready "
            f"(mode={self._mode}, serial={self._serial_port}@{self._serial_baud}, ws={self._ws_enabled})"
        )

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        cmd = command_from_cmd_vel(
            linear_x=msg.linear.x,
            angular_z=msg.angular.z,
            max_speed_mps=self._max_speed_mps,
            max_abs_angular_z=self._max_abs_angular_z,
            auto_drive_enabled=self._auto_drive_enabled,
            reverse_brake_pct=self._reverse_brake_pct,
        )
        with self._state_lock:
            self._auto_cmd = cmd
            self._auto_stamp_s = time.monotonic()

    def _apply_to_controller(self, cmd: DesiredCommand) -> None:
        self._client.set_drive_enabled(bool(cmd.drive_enabled))
        self._client.set_estop(bool(cmd.estop))
        self._client.set_speed_mps(float(cmd.speed_mps))
        self._client.set_steer_pct(int(cmd.steer_pct))
        self._client.set_brake_pct(int(cmd.brake_pct))

    def _control_tick(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            mode = self._mode
            manual_cmd = self._manual_cmd
            manual_stamp_s = self._manual_stamp_s
            auto_cmd = self._auto_cmd
            auto_stamp_s = self._auto_stamp_s
            global_estop = self._global_estop

        result = select_effective_command(
            mode=mode,
            now_s=now,
            manual_cmd=manual_cmd,
            manual_stamp_s=manual_stamp_s,
            manual_timeout_s=self._manual_timeout_s,
            auto_cmd=auto_cmd,
            auto_stamp_s=auto_stamp_s,
            auto_timeout_s=self._auto_timeout_s,
        )
        cmd = result.command

        if global_estop or cmd.estop:
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
            "mode": mode,
            "source": source,
            "fresh": result.fresh,
            "global_estop": global_estop,
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

    def _start_ws_server(self) -> None:
        self._ws_loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._ws_thread_main, daemon=True, name="controller-ws")
        self._ws_thread.start()

    def _ws_thread_main(self) -> None:
        assert self._ws_loop is not None
        asyncio.set_event_loop(self._ws_loop)
        self._ws_loop.run_until_complete(self._ws_start())
        self._ws_loop.run_forever()

    async def _ws_start(self) -> None:
        try:
            import websockets
        except ImportError as exc:
            self.get_logger().error(f"websockets not installed: {exc}")
            return

        self._ws_server = await websockets.serve(self._ws_handler, self._ws_host, self._ws_port)
        self.get_logger().info(f"WebSocket server listening on ws://{self._ws_host}:{self._ws_port}")

    async def _ws_handler(self, websocket) -> None:
        await websocket.send(json.dumps({"ok": True, "message": "controller_server ready"}, ensure_ascii=True))
        async for raw in websocket:
            response = self._handle_ws_raw(raw)
            await websocket.send(json.dumps(response, ensure_ascii=True))

    def _parse_optional_bool(self, value: Any, field_name: str) -> bool:
        if isinstance(value, bool):
            return value
        raise ValueError(f"'{field_name}' must be boolean")

    def _handle_ws_raw(self, raw: str) -> Dict[str, Any]:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            return {"ok": False, "error": f"invalid_json: {exc}"}

        if not isinstance(data, dict):
            return {"ok": False, "error": "payload must be object"}

        try:
            with self._state_lock:
                if "mode" in data:
                    mode = str(data["mode"]).strip().lower()
                    if mode not in ("manual", "auto"):
                        raise ValueError("mode must be 'manual' or 'auto'")
                    self._mode = mode

                if "estop" in data:
                    self._global_estop = self._parse_optional_bool(data["estop"], "estop")

                cmd = DesiredCommand(**asdict(self._manual_cmd))
                if "drive" in data:
                    cmd.drive_enabled = self._parse_optional_bool(data["drive"], "drive")
                if "speed_mps" in data:
                    cmd.speed_mps = float(data["speed_mps"])
                if "steer_pct" in data:
                    cmd.steer_pct = int(data["steer_pct"])
                if "brake_pct" in data:
                    cmd.brake_pct = int(data["brake_pct"])
                if "cmd_estop" in data:
                    cmd.estop = self._parse_optional_bool(data["cmd_estop"], "cmd_estop")

                cmd.speed_mps = max(0.0, min(self._max_speed_mps, float(cmd.speed_mps)))
                cmd.steer_pct = int(max(-100, min(100, int(cmd.steer_pct))))
                cmd.brake_pct = int(max(0, min(100, int(cmd.brake_pct))))

                if any(k in data for k in ("drive", "speed_mps", "steer_pct", "brake_pct", "cmd_estop")):
                    self._manual_cmd = cmd
                    self._manual_stamp_s = time.monotonic()

                snapshot = {
                    "mode": self._mode,
                    "global_estop": self._global_estop,
                    "manual_command": asdict(self._manual_cmd),
                }
            return {"ok": True, "state": snapshot}

        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": str(exc)}

    def destroy_node(self) -> bool:
        if self._ws_loop is not None:
            if self._ws_server is not None:
                async def _close_ws():
                    self._ws_server.close()
                    await self._ws_server.wait_closed()

                fut = asyncio.run_coroutine_threadsafe(_close_ws(), self._ws_loop)
                try:
                    fut.result(timeout=1.0)
                except Exception:
                    pass
            self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=1.0)

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
