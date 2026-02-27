from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class DesiredCommand:
    drive_enabled: bool = False
    estop: bool = False
    speed_mps: float = 0.0
    steer_pct: int = 0
    brake_pct: int = 0


@dataclass(slots=True)
class ArbitrationResult:
    command: DesiredCommand
    source: str
    fresh: bool


def safe_command() -> DesiredCommand:
    return DesiredCommand(drive_enabled=False, estop=False, speed_mps=0.0, steer_pct=0, brake_pct=0)


def command_from_cmd_vel(
    linear_x: float,
    angular_z: float,
    max_speed_mps: float,
    max_abs_angular_z: float,
    invert_steer: bool,
    auto_drive_enabled: bool,
    reverse_brake_pct: int,
) -> DesiredCommand:
    max_speed = max(0.0, float(max_speed_mps))
    speed = clamp(float(linear_x), 0.0, max_speed)

    steer = 0
    angular_scale = max(0.01, abs(float(max_abs_angular_z)))
    steer_ratio = clamp(float(angular_z) / angular_scale, -1.0, 1.0)
    steer = int(round(steer_ratio * 100.0))
    if bool(invert_steer):
        steer = -steer

    brake = 0
    if float(linear_x) < 0.0:
        brake = int(clamp(float(reverse_brake_pct), 0.0, 100.0))
        speed = 0.0

    return DesiredCommand(
        drive_enabled=bool(auto_drive_enabled),
        estop=False,
        speed_mps=speed,
        steer_pct=steer,
        brake_pct=brake,
    )


def select_effective_command(
    mode: str,
    now_s: float,
    manual_cmd: DesiredCommand,
    manual_stamp_s: float,
    manual_timeout_s: float,
    auto_cmd: DesiredCommand,
    auto_stamp_s: float,
    auto_timeout_s: float,
) -> ArbitrationResult:
    manual_fresh = (now_s - manual_stamp_s) <= max(0.0, manual_timeout_s)
    auto_fresh = (now_s - auto_stamp_s) <= max(0.0, auto_timeout_s)

    if mode == "manual":
        if manual_fresh:
            return ArbitrationResult(command=manual_cmd, source="manual", fresh=True)
        return ArbitrationResult(command=safe_command(), source="manual_timeout", fresh=False)

    if auto_fresh:
        return ArbitrationResult(command=auto_cmd, source="auto", fresh=True)
    return ArbitrationResult(command=safe_command(), source="auto_timeout", fresh=False)
