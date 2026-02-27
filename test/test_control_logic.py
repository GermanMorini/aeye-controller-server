from controller_server.control_logic import (
    DesiredCommand,
    command_from_cmd_vel,
    select_effective_command,
)


def test_command_from_cmd_vel_clamps_and_scales() -> None:
    cmd = command_from_cmd_vel(
        linear_x=9.0,
        angular_z=2.0,
        max_speed_mps=4.0,
        max_abs_angular_z=0.8,
        auto_drive_enabled=True,
        reverse_brake_pct=30,
    )
    assert cmd.drive_enabled is True
    assert cmd.speed_mps == 4.0
    assert cmd.steer_pct == 100
    assert cmd.brake_pct == 0


def test_command_from_cmd_vel_negative_speed_brakes() -> None:
    cmd = command_from_cmd_vel(
        linear_x=-0.5,
        angular_z=0.0,
        max_speed_mps=4.0,
        max_abs_angular_z=0.8,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.0
    assert cmd.brake_pct == 25


def test_select_effective_command_manual_timeout() -> None:
    now_s = 20.0
    manual_cmd = DesiredCommand(drive_enabled=True, speed_mps=1.0)
    auto_cmd = DesiredCommand(drive_enabled=True, speed_mps=2.0)
    result = select_effective_command(
        mode="manual",
        now_s=now_s,
        manual_cmd=manual_cmd,
        manual_stamp_s=10.0,
        manual_timeout_s=0.5,
        auto_cmd=auto_cmd,
        auto_stamp_s=19.8,
        auto_timeout_s=0.5,
    )
    assert result.source == "manual_timeout"
    assert result.command.drive_enabled is False
    assert result.command.speed_mps == 0.0


def test_select_effective_command_auto_fresh() -> None:
    now_s = 20.0
    manual_cmd = DesiredCommand(drive_enabled=True, speed_mps=1.0)
    auto_cmd = DesiredCommand(drive_enabled=True, speed_mps=2.0)
    result = select_effective_command(
        mode="auto",
        now_s=now_s,
        manual_cmd=manual_cmd,
        manual_stamp_s=19.5,
        manual_timeout_s=0.5,
        auto_cmd=auto_cmd,
        auto_stamp_s=19.9,
        auto_timeout_s=0.5,
    )
    assert result.source == "auto"
    assert result.command.speed_mps == 2.0
