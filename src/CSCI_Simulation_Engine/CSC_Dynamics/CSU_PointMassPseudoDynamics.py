from __future__ import annotations

import math
from dataclasses import dataclass

from src.CSCI_Guidance_Control.CSC_Controller.CSU_RollPDController import RollPDController
from src.CSCI_Guidance_Control.CSC_Controller.CSU_SpeedController import SpeedController
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


G = 9.80665


@dataclass(frozen=True)
class _DynamicsState:
    x_m: float
    y_m: float
    z_m: float
    speed_mps: float
    heading_rad: float
    flight_path_rad: float
    flight_path_rate_rad_s: float
    roll_rad: float
    roll_rate_rad_s: float


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

    if cfg.dt <= 0.0:
        return

    initial_state = _DynamicsState(
        x_m=uav.x_m,
        y_m=uav.y_m,
        z_m=uav.z_m,
        speed_mps=uav.speed_mps,
        heading_rad=uav.heading_rad,
        flight_path_rad=uav.flight_path_rad,
        flight_path_rate_rad_s=uav.flight_path_rate_rad_s,
        roll_rad=uav.roll_rad,
        roll_rate_rad_s=uav.roll_rate_rad_s,
    )
    next_state = _rk4_step(
        initial_state,
        dt_s=cfg.dt,
        roll_cmd_rad=roll_cmd_rad,
        cfg=cfg,
        speed_cmd_mps=speed_cmd_mps,
        flight_path_cmd_rad=flight_path_cmd_rad,
    )
    next_state = _clamp_integrated_state(next_state, cfg)

    uav.x_m = next_state.x_m
    uav.y_m = next_state.y_m
    uav.z_m = next_state.z_m
    uav.speed_mps = next_state.speed_mps
    uav.heading_rad = next_state.heading_rad
    uav.flight_path_rad = next_state.flight_path_rad
    uav.flight_path_rate_rad_s = next_state.flight_path_rate_rad_s
    uav.roll_rad = next_state.roll_rad
    uav.roll_rate_rad_s = next_state.roll_rate_rad_s

    final_derivative = _state_derivative(
        next_state,
        roll_cmd_rad=roll_cmd_rad,
        cfg=cfg,
        speed_cmd_mps=speed_cmd_mps,
        flight_path_cmd_rad=flight_path_cmd_rad,
    )
    uav.longitudinal_accel_mps2 = final_derivative.speed_mps
    uav.vertical_accel_mps2 = _calculate_vertical_accel(next_state, cfg)


def _rk4_step(
    state: _DynamicsState,
    dt_s: float,
    roll_cmd_rad: float,
    cfg: SimConfig,
    speed_cmd_mps: float | None,
    flight_path_cmd_rad: float | None,
) -> _DynamicsState:
    def derivative(current: _DynamicsState) -> _DynamicsState:
        return _state_derivative(
            current,
            roll_cmd_rad=roll_cmd_rad,
            cfg=cfg,
            speed_cmd_mps=speed_cmd_mps,
            flight_path_cmd_rad=flight_path_cmd_rad,
        )

    k1 = derivative(state)
    k2 = derivative(_add_scaled_state(state, k1, dt_s * 0.5))
    k3 = derivative(_add_scaled_state(state, k2, dt_s * 0.5))
    k4 = derivative(_add_scaled_state(state, k3, dt_s))

    return _DynamicsState(
        x_m=state.x_m + dt_s / 6.0 * (k1.x_m + 2.0 * k2.x_m + 2.0 * k3.x_m + k4.x_m),
        y_m=state.y_m + dt_s / 6.0 * (k1.y_m + 2.0 * k2.y_m + 2.0 * k3.y_m + k4.y_m),
        z_m=state.z_m + dt_s / 6.0 * (k1.z_m + 2.0 * k2.z_m + 2.0 * k3.z_m + k4.z_m),
        speed_mps=state.speed_mps
        + dt_s / 6.0 * (k1.speed_mps + 2.0 * k2.speed_mps + 2.0 * k3.speed_mps + k4.speed_mps),
        heading_rad=state.heading_rad
        + dt_s / 6.0 * (k1.heading_rad + 2.0 * k2.heading_rad + 2.0 * k3.heading_rad + k4.heading_rad),
        flight_path_rad=state.flight_path_rad
        + dt_s
        / 6.0
        * (
            k1.flight_path_rad
            + 2.0 * k2.flight_path_rad
            + 2.0 * k3.flight_path_rad
            + k4.flight_path_rad
        ),
        flight_path_rate_rad_s=state.flight_path_rate_rad_s
        + dt_s
        / 6.0
        * (
            k1.flight_path_rate_rad_s
            + 2.0 * k2.flight_path_rate_rad_s
            + 2.0 * k3.flight_path_rate_rad_s
            + k4.flight_path_rate_rad_s
        ),
        roll_rad=state.roll_rad
        + dt_s / 6.0 * (k1.roll_rad + 2.0 * k2.roll_rad + 2.0 * k3.roll_rad + k4.roll_rad),
        roll_rate_rad_s=state.roll_rate_rad_s
        + dt_s
        / 6.0
        * (k1.roll_rate_rad_s + 2.0 * k2.roll_rate_rad_s + 2.0 * k3.roll_rate_rad_s + k4.roll_rate_rad_s),
    )


def _state_derivative(
    state: _DynamicsState,
    roll_cmd_rad: float,
    cfg: SimConfig,
    speed_cmd_mps: float | None,
    flight_path_cmd_rad: float | None,
) -> _DynamicsState:
    speed_mps = clamp(state.speed_mps, cfg.min_speed_mps, cfg.max_speed_mps)

    if speed_cmd_mps is None:
        speed_cmd_mps = speed_mps
    desired_speed_mps = clamp(speed_cmd_mps, cfg.min_speed_mps, cfg.max_speed_mps)
    speed_controller = SpeedController(
        kp=cfg.speed_control_kp,
        max_accel_mps2=cfg.max_accel_mps2,
        max_decel_mps2=cfg.max_decel_mps2,
    )
    speed_dot_mps2 = speed_controller.compute_accel_command(
        desired_speed_mps=desired_speed_mps,
        current_speed_mps=speed_mps,
    )

    max_flight_path_rad = math.radians(cfg.max_flight_path_deg)
    if flight_path_cmd_rad is None:
        flight_path_cmd_rad = 0.0
    flight_path_rad = clamp(state.flight_path_rad, -max_flight_path_rad, max_flight_path_rad)
    desired_flight_path_rad = clamp(flight_path_cmd_rad, -max_flight_path_rad, max_flight_path_rad)
    flight_path_error_rad = desired_flight_path_rad - flight_path_rad
    flight_path_accel_rad_s2 = (
        cfg.flight_path_kp * flight_path_error_rad
        - cfg.flight_path_kd * state.flight_path_rate_rad_s
    )
    max_flight_path_accel_rad_s2 = math.radians(cfg.max_flight_path_accel_deg_s2)
    flight_path_accel_rad_s2 = clamp(
        flight_path_accel_rad_s2,
        -max_flight_path_accel_rad_s2,
        max_flight_path_accel_rad_s2,
    )

    max_roll_rad = math.radians(cfg.max_roll_deg)
    roll_rad = clamp(state.roll_rad, -max_roll_rad, max_roll_rad)
    desired_roll_rad = clamp(roll_cmd_rad, -max_roll_rad, max_roll_rad)
    roll_controller = RollPDController(kp=cfg.roll_pd_kp, kd=cfg.roll_pd_kd)
    roll_accel_rad_s2 = roll_controller.compute_roll_accel_command(
        desired_roll_rad=desired_roll_rad,
        current_roll_rad=roll_rad,
        current_roll_rate_rad_s=state.roll_rate_rad_s,
    )
    max_roll_accel_rad_s2 = math.radians(cfg.max_roll_accel_deg_s2)
    roll_accel_rad_s2 = clamp(roll_accel_rad_s2, -max_roll_accel_rad_s2, max_roll_accel_rad_s2)

    yaw_rate_rad_s = G / max(speed_mps * max(math.cos(flight_path_rad), 0.2), 1.0) * math.tan(roll_rad)
    horizontal_speed_mps = speed_mps * math.cos(flight_path_rad)

    return _DynamicsState(
        x_m=horizontal_speed_mps * math.cos(state.heading_rad),
        y_m=horizontal_speed_mps * math.sin(state.heading_rad),
        z_m=speed_mps * math.sin(flight_path_rad),
        speed_mps=speed_dot_mps2,
        heading_rad=yaw_rate_rad_s,
        flight_path_rad=state.flight_path_rate_rad_s,
        flight_path_rate_rad_s=flight_path_accel_rad_s2,
        roll_rad=state.roll_rate_rad_s,
        roll_rate_rad_s=roll_accel_rad_s2,
    )


def _add_scaled_state(state: _DynamicsState, derivative: _DynamicsState, scale: float) -> _DynamicsState:
    return _DynamicsState(
        x_m=state.x_m + derivative.x_m * scale,
        y_m=state.y_m + derivative.y_m * scale,
        z_m=state.z_m + derivative.z_m * scale,
        speed_mps=state.speed_mps + derivative.speed_mps * scale,
        heading_rad=state.heading_rad + derivative.heading_rad * scale,
        flight_path_rad=state.flight_path_rad + derivative.flight_path_rad * scale,
        flight_path_rate_rad_s=state.flight_path_rate_rad_s + derivative.flight_path_rate_rad_s * scale,
        roll_rad=state.roll_rad + derivative.roll_rad * scale,
        roll_rate_rad_s=state.roll_rate_rad_s + derivative.roll_rate_rad_s * scale,
    )


def _clamp_integrated_state(state: _DynamicsState, cfg: SimConfig) -> _DynamicsState:
    max_flight_path_rad = math.radians(cfg.max_flight_path_deg)
    max_flight_path_rate_rad_s = math.radians(cfg.max_flight_path_rate_deg_s)
    max_roll_rad = math.radians(cfg.max_roll_deg)
    max_roll_rate_rad_s = math.radians(cfg.max_roll_rate_deg_s)

    roll_rad = clamp(state.roll_rad, -max_roll_rad, max_roll_rad)
    roll_rate_rad_s = clamp(state.roll_rate_rad_s, -max_roll_rate_rad_s, max_roll_rate_rad_s)
    if abs(roll_rad) >= max_roll_rad and roll_rad * roll_rate_rad_s > 0.0:
        roll_rate_rad_s = 0.0

    return _DynamicsState(
        x_m=state.x_m,
        y_m=state.y_m,
        z_m=state.z_m,
        speed_mps=clamp(state.speed_mps, cfg.min_speed_mps, cfg.max_speed_mps),
        heading_rad=state.heading_rad,
        flight_path_rad=clamp(state.flight_path_rad, -max_flight_path_rad, max_flight_path_rad),
        flight_path_rate_rad_s=clamp(
            state.flight_path_rate_rad_s,
            -max_flight_path_rate_rad_s,
            max_flight_path_rate_rad_s,
        ),
        roll_rad=roll_rad,
        roll_rate_rad_s=roll_rate_rad_s,
    )


def _calculate_vertical_accel(state: _DynamicsState, cfg: SimConfig) -> float:
    speed_mps = clamp(state.speed_mps, cfg.min_speed_mps, cfg.max_speed_mps)
    dynamic_pressure_pa = 0.5 * cfg.air_density_kg_m3 * speed_mps * speed_mps
    lift_n = dynamic_pressure_pa * cfg.wing_area_m2 * cfg.lift_coefficient
    force_vertical_n = lift_n * math.cos(state.roll_rad) - cfg.aircraft_mass_kg * G * math.cos(state.flight_path_rad)
    return force_vertical_n / max(cfg.aircraft_mass_kg, 1e-6)
