from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_continuous_are

from src.CSCI_Guidance_Control.CSC_Controller.CSU_VerticalReducedModel import (
    VerticalReducedState,
    build_vertical_reduced_model,
    build_vertical_reduced_state,
)


@dataclass(frozen=True)
class VerticalLQRConfig:
    """Tuning weights and physical limits for the vertical LQR controller."""

    altitude_error_weight: float = 1.0
    flight_path_angle_weight: float = 14.0
    flight_path_rate_weight: float = 2.0
    flight_path_command_weight: float = 120.0
    max_flight_path_cmd_deg: float = 4.8
    max_flight_path_cmd_rate_deg_s: float = 6.0


@dataclass
class VerticalLQRController:
    """Continuous-time LQR controller for the reduced altitude channel.

    The LQR state is:

        x = [e_z, gamma, gamma_dot]^T
        e_z = z - z_cmd

    The control input is commanded flight-path angle:

        u = gamma_cmd

    The flight-path inner-loop dynamics are included in the reduced state-space
    model, so the outer-loop LQR does not assume gamma_cmd changes altitude
    instantly.
    """

    config: VerticalLQRConfig = field(default_factory=VerticalLQRConfig)
    _last_gamma_cmd_by_id: dict[str, float] = field(default_factory=dict)
    _gain_cache: dict[tuple[float, float, float], tuple[float, float, float]] = field(default_factory=dict)

    def compute_flight_path_command(
        self,
        *,
        altitude_m: float,
        commanded_altitude_m: float,
        speed_mps: float,
        flight_path_rad: float,
        flight_path_rate_rad_s: float,
        flight_path_kp: float,
        flight_path_kd: float,
        dt_s: float,
        command_id: str = "default",
        update_state: bool = True,
        apply_rate_limit: bool = True,
    ) -> float:
        state = build_vertical_reduced_state(
            altitude_m=altitude_m,
            commanded_altitude_m=commanded_altitude_m,
            flight_path_rad=flight_path_rad,
            flight_path_rate_rad_s=flight_path_rate_rad_s,
        )
        gain = self.compute_gain(
            speed_mps=speed_mps,
            flight_path_kp=flight_path_kp,
            flight_path_kd=flight_path_kd,
        )
        raw_cmd_rad = self.compute_raw_command(state=state, gain=gain)
        limited_cmd_rad = self._apply_limits(raw_cmd_rad, dt_s, command_id, apply_rate_limit)
        if update_state:
            self._last_gamma_cmd_by_id[command_id] = limited_cmd_rad
        return limited_cmd_rad

    def compute_gain(
        self,
        *,
        speed_mps: float,
        flight_path_kp: float,
        flight_path_kd: float,
    ) -> tuple[float, float, float]:
        cache_key = (
            round(max(speed_mps, 1e-6), 2),
            round(flight_path_kp, 3),
            round(flight_path_kd, 3),
        )
        if cache_key in self._gain_cache:
            return self._gain_cache[cache_key]

        model = build_vertical_reduced_model(
            speed_mps=cache_key[0],
            flight_path_kp=cache_key[1],
            flight_path_kd=cache_key[2],
        )
        a_matrix = np.array(model.a_matrix, dtype=float)
        b_matrix = np.array(model.b_matrix, dtype=float)
        q_matrix = np.diag(
            [
                self.config.altitude_error_weight,
                self.config.flight_path_angle_weight,
                self.config.flight_path_rate_weight,
            ]
        )
        r_matrix = np.array([[self.config.flight_path_command_weight]], dtype=float)

        riccati_solution = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain_matrix = np.linalg.solve(r_matrix, b_matrix.T @ riccati_solution)
        gain = tuple(float(value) for value in gain_matrix[0])
        self._gain_cache[cache_key] = gain
        return gain

    @staticmethod
    def compute_raw_command(
        *,
        state: VerticalReducedState,
        gain: tuple[float, float, float],
    ) -> float:
        k_altitude, k_flight_path, k_flight_path_rate = gain
        return -(
            k_altitude * state.altitude_error_m
            + k_flight_path * state.flight_path_rad
            + k_flight_path_rate * state.flight_path_rate_rad_s
        )

    def reset(self, command_id: str | None = None, gamma_cmd_rad: float = 0.0) -> None:
        if command_id is None:
            self._last_gamma_cmd_by_id.clear()
            return
        self._last_gamma_cmd_by_id[command_id] = gamma_cmd_rad

    def _apply_limits(
        self,
        gamma_cmd_rad: float,
        dt_s: float,
        command_id: str,
        apply_rate_limit: bool,
    ) -> float:
        max_cmd_rad = math.radians(self.config.max_flight_path_cmd_deg)
        gamma_cmd_rad = max(-max_cmd_rad, min(gamma_cmd_rad, max_cmd_rad))

        if dt_s <= 0.0 or not apply_rate_limit:
            return gamma_cmd_rad

        last_gamma_cmd_rad = self._last_gamma_cmd_by_id.get(command_id, 0.0)
        max_delta_rad = math.radians(self.config.max_flight_path_cmd_rate_deg_s) * dt_s
        lower = last_gamma_cmd_rad - max_delta_rad
        upper = last_gamma_cmd_rad + max_delta_rad
        return max(lower, min(gamma_cmd_rad, upper))
