from controller_server.control_logic import DesiredCommand
from controller_server.logging_policy import should_log_command


def _cmd(
    *,
    drive_enabled: bool = True,
    estop: bool = False,
    speed_mps: float = 1.0,
    steer_pct: int = 10,
    brake_pct: int = 0,
) -> DesiredCommand:
    return DesiredCommand(
        drive_enabled=drive_enabled,
        estop=estop,
        speed_mps=speed_mps,
        steer_pct=steer_pct,
        brake_pct=brake_pct,
    )


def test_should_log_first_command() -> None:
    decision = should_log_command(
        current_cmd=_cmd(),
        last_logged_cmd=None,
        now_s=10.0,
        last_log_s=None,
        heartbeat_s=5.0,
    )
    assert decision.should_log is True
    assert decision.reason == "first"


def test_should_not_log_same_command_before_heartbeat() -> None:
    cmd = _cmd()
    decision = should_log_command(
        current_cmd=cmd,
        last_logged_cmd=cmd,
        now_s=13.0,
        last_log_s=10.0,
        heartbeat_s=5.0,
    )
    assert decision.should_log is False
    assert decision.reason == "suppressed"


def test_should_log_same_command_on_heartbeat() -> None:
    cmd = _cmd()
    decision = should_log_command(
        current_cmd=cmd,
        last_logged_cmd=cmd,
        now_s=15.0,
        last_log_s=10.0,
        heartbeat_s=5.0,
    )
    assert decision.should_log is True
    assert decision.reason == "heartbeat"


def test_should_log_immediately_when_command_changes() -> None:
    decision = should_log_command(
        current_cmd=_cmd(speed_mps=1.5),
        last_logged_cmd=_cmd(speed_mps=1.0),
        now_s=12.0,
        last_log_s=10.0,
        heartbeat_s=5.0,
    )
    assert decision.should_log is True
    assert decision.reason == "changed"


def test_should_log_when_any_field_changes() -> None:
    base = _cmd()
    variants = [
        _cmd(drive_enabled=False),
        _cmd(estop=True),
        _cmd(speed_mps=0.7),
        _cmd(steer_pct=20),
        _cmd(brake_pct=15),
    ]
    for variant in variants:
        decision = should_log_command(
            current_cmd=variant,
            last_logged_cmd=base,
            now_s=12.0,
            last_log_s=10.0,
            heartbeat_s=5.0,
        )
        assert decision.should_log is True
        assert decision.reason == "changed"
