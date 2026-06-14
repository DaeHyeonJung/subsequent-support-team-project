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
    vertical_speed_weight: float = 2.5
    flight_path_command_weight: float = 90.0
    flight_path_response_bandwidth_rad_s: float = 1.8
    max_flight_path_cmd_deg: float = 5.5
    max_flight_path_cmd_rate_deg_s: float = 8.0


@dataclass
class VerticalLQRController:
    """Continuous-time LQR controller for the reduced altitude channel.

    The LQR state is:

        x = [e_z, z_dot]^T
        e_z = z - z_cmd

    The control input is the commanded flight-path angle:

        u = gamma_cmd

    The feedback law is:

        gamma_cmd = -K x

    K is computed from the continuous algebraic Riccati equation using Q/R
    weights in `VerticalLQRConfig`.
    """

    config: VerticalLQRConfig = field(default_factory=VerticalLQRConfig)
    _last_gamma_cmd_by_id: dict[str, float] = field(default_factory=dict)
    _gain_cache: dict[float, tuple[float, float]] = field(default_factory=dict)

    def compute_flight_path_command(
        self,
        *,
        altitude_m: float,
        commanded_altitude_m: float,
        speed_mps: float,
        flight_path_rad: float,
        dt_s: float,
        command_id: str = "default",
        update_state: bool = True,
        apply_rate_limit: bool = True,
    ) -> float:
        state = build_vertical_reduced_state(
            altitude_m=altitude_m,
            commanded_altitude_m=commanded_altitude_m,
            speed_mps=speed_mps,
            flight_path_rad=flight_path_rad,
        )
        k_altitude, k_vertical_speed = self.compute_gain(speed_mps=speed_mps)
        raw_cmd_rad = self.compute_raw_command(
            state=state,
            k_altitude=k_altitude,
            k_vertical_speed=k_vertical_speed,
        )
        limited_cmd_rad = self._apply_limits(raw_cmd_rad, dt_s, command_id, apply_rate_limit)
        if update_state:
            self._last_gamma_cmd_by_id[command_id] = limited_cmd_rad
        return limited_cmd_rad

    def compute_gain(self, *, speed_mps: float) -> tuple[float, float]:
        cache_key = round(max(speed_mps, 1e-6), 2)
        if cache_key in self._gain_cache:
            return self._gain_cache[cache_key]

        model = build_vertical_reduced_model(
            speed_mps=cache_key,
            flight_path_response_bandwidth_rad_s=self.config.flight_path_response_bandwidth_rad_s,
        )
        a_matrix = np.array(model.a_matrix, dtype=float)
        b_matrix = np.array(model.b_matrix, dtype=float)
        q_matrix = np.diag(
            [
                self.config.altitude_error_weight,
                self.config.vertical_speed_weight,
            ]
        )
        r_matrix = np.array([[self.config.flight_path_command_weight]], dtype=float)

        riccati_solution = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain_matrix = np.linalg.solve(r_matrix, b_matrix.T @ riccati_solution)
        gain = (float(gain_matrix[0, 0]), float(gain_matrix[0, 1]))
        self._gain_cache[cache_key] = gain
        return gain

    @staticmethod
    def compute_raw_command(
        *,
        state: VerticalReducedState,
        k_altitude: float,
        k_vertical_speed: float,
    ) -> float:
        return -(
            k_altitude * state.altitude_error_m
            + k_vertical_speed * state.vertical_speed_mps
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
