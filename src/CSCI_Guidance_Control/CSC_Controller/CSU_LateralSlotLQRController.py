from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_continuous_are

from src.CSCI_Guidance_Control.CSC_Controller.CSU_LateralSlotReducedModel import (
    LateralSlotReducedState,
    build_lateral_slot_reduced_model,
    build_lateral_slot_reduced_state,
)


@dataclass(frozen=True)
class LateralSlotLQRConfig:
    """Tuning weights and limits for the reduced lateral slot controller."""

    lateral_error_weight: float = 0.7
    lateral_speed_weight: float = 1.6
    heading_offset_command_weight: float = 120.0
    lateral_response_bandwidth_rad_s: float = 0.8
    max_heading_offset_cmd_deg: float = 12.0
    max_heading_offset_cmd_rate_deg_s: float = 14.0


@dataclass
class LateralSlotLQRController:
    """Continuous-time LQR controller for lateral slot tracking.

    The LQR state is:

        x = [e_x, x_dot]^T
        e_x = x - x_cmd

    The control input is the heading offset from the nominal northbound
    formation heading:

        u = psi_offset

    The feedback law is:

        psi_offset = -K x

    K is computed from the continuous algebraic Riccati equation using Q/R
    weights in `LateralSlotLQRConfig`.
    """

    config: LateralSlotLQRConfig = field(default_factory=LateralSlotLQRConfig)
    _last_heading_offset_by_id: dict[str, float] = field(default_factory=dict)
    _gain_cache: dict[float, tuple[float, float]] = field(default_factory=dict)

    def compute_heading_offset_command(
        self,
        *,
        lateral_position_m: float,
        commanded_lateral_position_m: float,
        speed_mps: float,
        heading_rad: float,
        dt_s: float,
        command_id: str = "default",
        update_state: bool = True,
        apply_rate_limit: bool = True,
    ) -> float:
        state = build_lateral_slot_reduced_state(
            lateral_position_m=lateral_position_m,
            commanded_lateral_position_m=commanded_lateral_position_m,
            speed_mps=speed_mps,
            heading_rad=heading_rad,
        )
        k_lateral_error, k_lateral_speed = self.compute_gain(speed_mps=speed_mps)
        raw_cmd_rad = self.compute_raw_command(
            state=state,
            k_lateral_error=k_lateral_error,
            k_lateral_speed=k_lateral_speed,
        )
        limited_cmd_rad = self._apply_limits(raw_cmd_rad, dt_s, command_id, apply_rate_limit)
        if update_state:
            self._last_heading_offset_by_id[command_id] = limited_cmd_rad
        return limited_cmd_rad

    def compute_gain(self, *, speed_mps: float) -> tuple[float, float]:
        cache_key = round(max(speed_mps, 1e-6), 2)
        if cache_key in self._gain_cache:
            return self._gain_cache[cache_key]

        model = build_lateral_slot_reduced_model(
            speed_mps=cache_key,
            lateral_response_bandwidth_rad_s=self.config.lateral_response_bandwidth_rad_s,
        )
        a_matrix = np.array(model.a_matrix, dtype=float)
        b_matrix = np.array(model.b_matrix, dtype=float)
        q_matrix = np.diag(
            [
                self.config.lateral_error_weight,
                self.config.lateral_speed_weight,
            ]
        )
        r_matrix = np.array([[self.config.heading_offset_command_weight]], dtype=float)

        riccati_solution = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain_matrix = np.linalg.solve(r_matrix, b_matrix.T @ riccati_solution)
        gain = (float(gain_matrix[0, 0]), float(gain_matrix[0, 1]))
        self._gain_cache[cache_key] = gain
        return gain

    @staticmethod
    def compute_raw_command(
        *,
        state: LateralSlotReducedState,
        k_lateral_error: float,
        k_lateral_speed: float,
    ) -> float:
        return -(
            k_lateral_error * state.lateral_error_m
            + k_lateral_speed * state.lateral_speed_mps
        )

    def reset(self, command_id: str | None = None, heading_offset_cmd_rad: float = 0.0) -> None:
        if command_id is None:
            self._last_heading_offset_by_id.clear()
            return
        self._last_heading_offset_by_id[command_id] = heading_offset_cmd_rad

    def _apply_limits(
        self,
        heading_offset_cmd_rad: float,
        dt_s: float,
        command_id: str,
        apply_rate_limit: bool,
    ) -> float:
        max_cmd_rad = math.radians(self.config.max_heading_offset_cmd_deg)
        heading_offset_cmd_rad = max(-max_cmd_rad, min(heading_offset_cmd_rad, max_cmd_rad))

        if dt_s <= 0.0 or not apply_rate_limit:
            return heading_offset_cmd_rad

        last_heading_offset_rad = self._last_heading_offset_by_id.get(command_id, 0.0)
        max_delta_rad = math.radians(self.config.max_heading_offset_cmd_rate_deg_s) * dt_s
        lower = last_heading_offset_rad - max_delta_rad
        upper = last_heading_offset_rad + max_delta_rad
        return max(lower, min(heading_offset_cmd_rad, upper))
