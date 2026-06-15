from __future__ import annotations

import math

from src.CSCI_Guidance_Control.CSC_Controller.CSU_RollPDController import RollPDController
from src.CSCI_Guidance_Control.CSC_Controller.CSU_SpeedController import SpeedController
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def step_uav(
    uav: UavState,
    roll_cmd_rad: float,
    cfg: SimConfig,
    speed_cmd_mps: float | None = None,
    flight_path_cmd_rad: float | None = None,
) -> None:
    if not uav.available:
        return

    if speed_cmd_mps is None:
        speed_cmd_mps = uav.speed_mps

    desired_speed_mps = clamp(speed_cmd_mps, cfg.min_speed_mps, cfg.max_speed_mps)
    speed_controller = SpeedController(
        kp=cfg.speed_control_kp,
        max_accel_mps2=cfg.max_accel_mps2,
        max_decel_mps2=cfg.max_decel_mps2,
    )
    uav.longitudinal_accel_mps2 = speed_controller.compute_accel_command(
        desired_speed_mps=desired_speed_mps,
        current_speed_mps=uav.speed_mps,
    )
    uav.speed_mps += uav.longitudinal_accel_mps2 * cfg.dt
    uav.speed_mps = clamp(uav.speed_mps, cfg.min_speed_mps, cfg.max_speed_mps)

    if flight_path_cmd_rad is None:
        flight_path_cmd_rad = 0.0
    max_flight_path_rad = math.radians(cfg.max_flight_path_deg)
    desired_flight_path_rad = clamp(flight_path_cmd_rad, -max_flight_path_rad, max_flight_path_rad)
    flight_path_error_rad = desired_flight_path_rad - uav.flight_path_rad
    flight_path_accel_rad_s2 = cfg.flight_path_kp * flight_path_error_rad - cfg.flight_path_kd * uav.flight_path_rate_rad_s
    max_flight_path_accel_rad_s2 = math.radians(cfg.max_flight_path_accel_deg_s2)
    flight_path_accel_rad_s2 = clamp(
        flight_path_accel_rad_s2,
        -max_flight_path_accel_rad_s2,
        max_flight_path_accel_rad_s2,
    )
    max_flight_path_rate_rad_s = math.radians(cfg.max_flight_path_rate_deg_s)
    uav.flight_path_rate_rad_s += flight_path_accel_rad_s2 * cfg.dt
    uav.flight_path_rate_rad_s = clamp(
        uav.flight_path_rate_rad_s,
        -max_flight_path_rate_rad_s,
        max_flight_path_rate_rad_s,
    )
    uav.flight_path_rad += uav.flight_path_rate_rad_s * cfg.dt
    uav.flight_path_rad = clamp(uav.flight_path_rad, -max_flight_path_rad, max_flight_path_rad)

    max_roll_rad = math.radians(cfg.max_roll_deg)
    desired_roll_rad = clamp(roll_cmd_rad, -max_roll_rad, max_roll_rad)

    roll_controller = RollPDController(kp=cfg.roll_pd_kp, kd=cfg.roll_pd_kd)
    roll_accel_rad_s2 = roll_controller.compute_roll_accel_command(
        desired_roll_rad=desired_roll_rad,
        current_roll_rad=uav.roll_rad,
        current_roll_rate_rad_s=uav.roll_rate_rad_s,
    )
    max_roll_accel_rad_s2 = math.radians(cfg.max_roll_accel_deg_s2)
    roll_accel_rad_s2 = clamp(roll_accel_rad_s2, -max_roll_accel_rad_s2, max_roll_accel_rad_s2)

    max_roll_rate_rad_s = math.radians(cfg.max_roll_rate_deg_s)
    uav.roll_rate_rad_s += roll_accel_rad_s2 * cfg.dt
    uav.roll_rate_rad_s = clamp(uav.roll_rate_rad_s, -max_roll_rate_rad_s, max_roll_rate_rad_s)
    uav.roll_rad += uav.roll_rate_rad_s * cfg.dt
    uav.roll_rad = clamp(uav.roll_rad, -max_roll_rad, max_roll_rad)

    if abs(uav.roll_rad) >= max_roll_rad and uav.roll_rad * uav.roll_rate_rad_s > 0.0:
        uav.roll_rate_rad_s = 0.0

    gravity_mps2 = cfg.gravity_mps2
    yaw_rate_rad_s = gravity_mps2 / max(uav.speed_mps * max(math.cos(uav.flight_path_rad), 0.2), 1.0) * math.tan(uav.roll_rad)
    uav.heading_rad += yaw_rate_rad_s * cfg.dt

    dynamic_pressure_pa = 0.5 * cfg.air_density_kg_m3 * uav.speed_mps * uav.speed_mps
    lift_n = dynamic_pressure_pa * cfg.wing_area_m2 * cfg.lift_coefficient
    drag_n = dynamic_pressure_pa * cfg.wing_area_m2 * cfg.drag_coefficient
    thrust_n = cfg.thrust_coefficient * cfg.air_density_kg_m3 * cfg.wing_area_m2 * uav.speed_mps * uav.speed_mps
    force_vertical_n = lift_n * math.cos(uav.roll_rad) - cfg.aircraft_mass_kg * gravity_mps2 * math.cos(uav.flight_path_rad)
    uav.vertical_accel_mps2 = force_vertical_n / max(cfg.aircraft_mass_kg, 1e-6)
    _ = drag_n
    _ = thrust_n

    horizontal_speed_mps = uav.speed_mps * math.cos(uav.flight_path_rad)
    uav.x_m += horizontal_speed_mps * math.cos(uav.heading_rad) * cfg.dt
    uav.y_m += horizontal_speed_mps * math.sin(uav.heading_rad) * cfg.dt
    uav.z_m += uav.speed_mps * math.sin(uav.flight_path_rad) * cfg.dt
