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
    """Tuning weights and limits for the roll-command lateral LQR."""

    lateral_error_weight: float = 0.9
    heading_error_weight: float = 7.0
    roll_angle_weight: float = 1.2
    roll_rate_weight: float = 0.35
    roll_command_weight: float = 55.0
    max_roll_cmd_deg: float = 14.0
    max_roll_cmd_rate_deg_s: float = 30.0


@dataclass
class LateralSlotLQRController:
    """Continuous-time LQR controller for lateral slot tracking.

    The LQR state is:

        x = [e_x, psi_err, phi, phi_dot]^T

    and the control input is commanded roll:

        u = phi_cmd

    This lets the outer-loop slot controller account for heading and roll
    dynamics directly instead of commanding a heading offset that is then
    converted again by a separate heading controller.
    """

    config: LateralSlotLQRConfig = field(default_factory=LateralSlotLQRConfig)
    _last_roll_cmd_by_id: dict[str, float] = field(default_factory=dict)
    _gain_cache: dict[tuple[float, float, float, float], tuple[float, float, float, float]] = field(default_factory=dict)

    def compute_roll_command(
        self,
        *,
        lateral_position_m: float,
        commanded_lateral_position_m: float,
        speed_mps: float,
        heading_rad: float,
        roll_rad: float,
        roll_rate_rad_s: float,
        gravity_mps2: float,
        roll_pd_kp: float,
        roll_pd_kd: float,
        dt_s: float,
        command_id: str = "default",
        update_state: bool = True,
        apply_rate_limit: bool = True,
    ) -> float:
        state = build_lateral_slot_reduced_state(
            lateral_position_m=lateral_position_m,
            commanded_lateral_position_m=commanded_lateral_position_m,
            heading_rad=heading_rad,
            roll_rad=roll_rad,
            roll_rate_rad_s=roll_rate_rad_s,
        )
        gain = self.compute_gain(
            speed_mps=speed_mps,
            gravity_mps2=gravity_mps2,
            roll_pd_kp=roll_pd_kp,
            roll_pd_kd=roll_pd_kd,
        )
        raw_cmd_rad = self.compute_raw_command(state=state, gain=gain)
        limited_cmd_rad = self._apply_limits(raw_cmd_rad, dt_s, command_id, apply_rate_limit)
        if update_state:
            self._last_roll_cmd_by_id[command_id] = limited_cmd_rad
        return limited_cmd_rad

    def compute_gain(
        self,
        *,
        speed_mps: float,
        gravity_mps2: float,
        roll_pd_kp: float,
        roll_pd_kd: float,
    ) -> tuple[float, float, float, float]:
        cache_key = (
            round(max(speed_mps, 1e-6), 2),
            round(gravity_mps2, 3),
            round(roll_pd_kp, 3),
            round(roll_pd_kd, 3),
        )
        if cache_key in self._gain_cache:
            return self._gain_cache[cache_key]

        model = build_lateral_slot_reduced_model(
            speed_mps=cache_key[0],
            gravity_mps2=cache_key[1],
            roll_pd_kp=cache_key[2],
            roll_pd_kd=cache_key[3],
        )
        a_matrix = np.array(model.a_matrix, dtype=float)
        b_matrix = np.array(model.b_matrix, dtype=float)
        q_matrix = np.diag(
            [
                self.config.lateral_error_weight,
                self.config.heading_error_weight,
                self.config.roll_angle_weight,
                self.config.roll_rate_weight,
            ]
        )
        r_matrix = np.array([[self.config.roll_command_weight]], dtype=float)

        riccati_solution = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain_matrix = np.linalg.solve(r_matrix, b_matrix.T @ riccati_solution)
        gain = tuple(float(value) for value in gain_matrix[0])
        self._gain_cache[cache_key] = gain
        return gain

    @staticmethod
    def compute_raw_command(
        *,
        state: LateralSlotReducedState,
        gain: tuple[float, float, float, float],
    ) -> float:
        k_lateral_error, k_heading_error, k_roll, k_roll_rate = gain
        return -(
            k_lateral_error * state.lateral_error_m
            + k_heading_error * state.heading_error_rad
            + k_roll * state.roll_rad
            + k_roll_rate * state.roll_rate_rad_s
        )

    def reset(self, command_id: str | None = None, roll_cmd_rad: float = 0.0) -> None:
        if command_id is None:
            self._last_roll_cmd_by_id.clear()
            return
        self._last_roll_cmd_by_id[command_id] = roll_cmd_rad

    def _apply_limits(
        self,
        roll_cmd_rad: float,
        dt_s: float,
        command_id: str,
        apply_rate_limit: bool,
    ) -> float:
        max_cmd_rad = math.radians(self.config.max_roll_cmd_deg)
        roll_cmd_rad = max(-max_cmd_rad, min(roll_cmd_rad, max_cmd_rad))

        if dt_s <= 0.0 or not apply_rate_limit:
            return roll_cmd_rad

        last_roll_cmd_rad = self._last_roll_cmd_by_id.get(command_id, 0.0)
        max_delta_rad = math.radians(self.config.max_roll_cmd_rate_deg_s) * dt_s
        lower = last_roll_cmd_rad - max_delta_rad
        upper = last_roll_cmd_rad + max_delta_rad
        return max(lower, min(roll_cmd_rad, upper))
