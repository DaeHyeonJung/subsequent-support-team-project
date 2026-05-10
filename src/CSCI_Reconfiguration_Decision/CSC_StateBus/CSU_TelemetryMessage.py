from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class UavTelemetryMessage:
    uid: str
    time_s: float
    formation_id: int
    role: str
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    roll_rad: float
    battery_pct: float
    link_ok: bool
    vehicle_health: str
    payload_ok: bool
