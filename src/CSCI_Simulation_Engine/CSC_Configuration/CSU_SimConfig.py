from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.1
    duration: float = 80.0
    speed_mps: float = 15.0
    roll_time_constant_s: float = 1.2
    roll_pd_kp: float = 6.25
    roll_pd_kd: float = 5.0
    speed_control_kp: float = 1.4
    flight_path_kp: float = 1.8
    flight_path_kd: float = 1.2
    max_roll_deg: float = 30.0
    max_roll_rate_deg_s: float = 90.0
    max_roll_accel_deg_s2: float = 180.0
    max_flight_path_deg: float = 18.0
    max_flight_path_rate_deg_s: float = 20.0
    max_flight_path_accel_deg_s2: float = 35.0
    min_speed_mps: float = 5.0
    max_speed_mps: float = 35.0
    max_accel_mps2: float = 3.0
    max_decel_mps2: float = 4.0
    aircraft_mass_kg: float = 6.0
    wing_area_m2: float = 0.55
    air_density_kg_m3: float = 1.225
    lift_coefficient: float = 0.65
    drag_coefficient: float = 0.06
    thrust_coefficient: float = 0.08
    output_dir: Path = Path("outputs")
