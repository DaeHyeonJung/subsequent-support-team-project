from __future__ import annotations

import math
from dataclasses import dataclass


Matrix4x4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]
Matrix4x1 = tuple[tuple[float], tuple[float], tuple[float], tuple[float]]


@dataclass(frozen=True)
class LateralSlotReducedState:
    """Reduced lateral state for roll-command slot-tracking control.

    State convention:
        x = [e_x, psi_err, phi, phi_dot]^T

    where:
        e_x     = x - x_cmd
        psi_err = psi - 90 deg
        phi     = roll angle
        phi_dot = roll rate

    The simulator flies northbound in formation. Around psi = 90 deg,
    x_dot ~= -V * psi_err. Positive roll increases heading, which moves the
    aircraft left in the top-view x axis and reduces a positive e_x.
    """

    lateral_error_m: float
    heading_error_rad: float
    roll_rad: float
    roll_rate_rad_s: float


@dataclass(frozen=True)
class LateralSlotReducedModel:
    """Linearized lateral roll-command model for LQR design.

    The control input is commanded roll:

        u = phi_cmd

    The inner roll loop is approximated using the same second-order structure
    as the simulator's RollPDController:

        phi_ddot = kp * (phi_cmd - phi) - kd * phi_dot

    The reduced lateral dynamics are:

        e_x_dot     = -V * psi_err
        psi_err_dot = g / V * phi
        phi_dot     = phi_dot
        phi_ddot    = -kp * phi - kd * phi_dot + kp * phi_cmd

    Therefore:

        x_dot = A x + B u
    """

    speed_mps: float
    gravity_mps2: float
    roll_pd_kp: float
    roll_pd_kd: float

    @property
    def a_matrix(self) -> Matrix4x4:
        speed_mps = max(self.speed_mps, 1e-6)
        return (
            (0.0, -speed_mps, 0.0, 0.0),
            (0.0, 0.0, self.gravity_mps2 / speed_mps, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, -self.roll_pd_kp, -self.roll_pd_kd),
        )

    @property
    def b_matrix(self) -> Matrix4x1:
        return (
            (0.0,),
            (0.0,),
            (0.0,),
            (self.roll_pd_kp,),
        )


def wrap_angle_rad(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def build_lateral_slot_reduced_state(
    *,
    lateral_position_m: float,
    commanded_lateral_position_m: float,
    heading_rad: float,
    roll_rad: float,
    roll_rate_rad_s: float,
    nominal_heading_rad: float = math.pi / 2.0,
) -> LateralSlotReducedState:
    return LateralSlotReducedState(
        lateral_error_m=lateral_position_m - commanded_lateral_position_m,
        heading_error_rad=wrap_angle_rad(heading_rad - nominal_heading_rad),
        roll_rad=roll_rad,
        roll_rate_rad_s=roll_rate_rad_s,
    )


def build_lateral_slot_reduced_model(
    *,
    speed_mps: float,
    gravity_mps2: float,
    roll_pd_kp: float,
    roll_pd_kd: float,
) -> LateralSlotReducedModel:
    return LateralSlotReducedModel(
        speed_mps=max(speed_mps, 1e-6),
        gravity_mps2=gravity_mps2,
        roll_pd_kp=roll_pd_kp,
        roll_pd_kd=roll_pd_kd,
    )
