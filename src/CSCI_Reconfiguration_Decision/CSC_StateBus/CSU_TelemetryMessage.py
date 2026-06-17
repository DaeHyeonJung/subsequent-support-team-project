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
    z_m: float
    heading_rad: float
    flight_path_rad: float
    speed_mps: float
    longitudinal_accel_mps2: float
    vertical_accel_mps2: float
    roll_rad: float
    roll_rate_rad_s: float
    flight_path_rate_rad_s: float
    battery_pct: float
    battery_discharge_progress: float
    cell_voltage_v: float
    link_ok: bool
    vehicle_health: str
    payload_ok: bool
