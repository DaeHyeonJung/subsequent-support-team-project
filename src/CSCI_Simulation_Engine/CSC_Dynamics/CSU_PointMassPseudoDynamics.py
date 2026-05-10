from __future__ import annotations

import math

from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


G = 9.80665


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def step_uav(uav: UavState, roll_cmd_rad: float, cfg: SimConfig) -> None:
    if not uav.available:
        return

    max_roll_rad = math.radians(cfg.max_roll_deg)
    commanded_roll = clamp(roll_cmd_rad, -max_roll_rad, max_roll_rad)

    roll_dot = (commanded_roll - uav.roll_rad) / cfg.roll_time_constant_s
    uav.roll_rad += roll_dot * cfg.dt

    yaw_rate_rad_s = G / max(uav.speed_mps, 1.0) * math.tan(uav.roll_rad)
    uav.heading_rad += yaw_rate_rad_s * cfg.dt

    uav.x_m += uav.speed_mps * math.cos(uav.heading_rad) * cfg.dt
    uav.y_m += uav.speed_mps * math.sin(uav.heading_rad) * cfg.dt
