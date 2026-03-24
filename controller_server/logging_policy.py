from __future__ import annotations

from dataclasses import dataclass

from .control_logic import DesiredCommand


@dataclass(frozen=True)
class CommandLogDecision:
    should_log: bool
    reason: str


def should_log_command(
    *,
    current_cmd: DesiredCommand,
    last_logged_cmd: DesiredCommand | None,
    now_s: float,
    last_log_s: float | None,
    heartbeat_s: float = 5.0,
) -> CommandLogDecision:
    if last_logged_cmd is None or last_log_s is None:
        return CommandLogDecision(should_log=True, reason="first")

    if current_cmd != last_logged_cmd:
        return CommandLogDecision(should_log=True, reason="changed")

    if now_s - last_log_s >= heartbeat_s:
        return CommandLogDecision(should_log=True, reason="heartbeat")

    return CommandLogDecision(should_log=False, reason="suppressed")
