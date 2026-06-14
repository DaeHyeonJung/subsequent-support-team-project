from __future__ import annotations

import math
from dataclasses import dataclass


Matrix2x2 = tuple[tuple[float, float], tuple[float, float]]
Matrix2x1 = tuple[tuple[float], tuple[float]]


@dataclass(frozen=True)
class VerticalReducedState:
    """Reduced vertical state for altitude-channel controller design.

    State convention:
        x = [e_z, z_dot]^T
        e_z = z - z_cmd

    A positive altitude error means the UAV is above the commanded altitude.
    A stabilizing feedback should therefore command a negative flight-path
    angle when e_z is positive.
    """

    altitude_error_m: float
    vertical_speed_mps: float


@dataclass(frozen=True)
class VerticalReducedModel:
    """Linearized vertical model for LQR design.

    The full simulator commands flight path angle, not vertical acceleration.
    For the LQR design model, the vertical speed response is approximated as:

        e_z_dot   = z_dot
        z_dot_dot = -a_gamma * z_dot + a_gamma * V * gamma_cmd

    where:
        e_z       = z - z_cmd
        z_dot     = V * sin(gamma)
        gamma_cmd = commanded flight-path angle [rad]
        a_gamma   = vertical-speed response bandwidth [1/s]

    Therefore:

        x_dot = A x + B u

        A = [[0, 1],
             [0, -a_gamma]]

        B = [[0],
             [a_gamma * V]]
    """

    speed_mps: float
    flight_path_response_bandwidth_rad_s: float

    @property
    def a_matrix(self) -> Matrix2x2:
        a_gamma = self.flight_path_response_bandwidth_rad_s
        return (
            (0.0, 1.0),
            (0.0, -a_gamma),
        )

    @property
    def b_matrix(self) -> Matrix2x1:
        a_gamma = self.flight_path_response_bandwidth_rad_s
        return (
            (0.0,),
            (a_gamma * self.speed_mps,),
        )


def build_vertical_reduced_state(
    *,
    altitude_m: float,
    commanded_altitude_m: float,
    speed_mps: float,
    flight_path_rad: float,
) -> VerticalReducedState:
    return VerticalReducedState(
        altitude_error_m=altitude_m - commanded_altitude_m,
        vertical_speed_mps=speed_mps * math.sin(flight_path_rad),
    )


def build_vertical_reduced_model(
    *,
    speed_mps: float,
    flight_path_response_bandwidth_rad_s: float,
) -> VerticalReducedModel:
    return VerticalReducedModel(
        speed_mps=max(speed_mps, 1e-6),
        flight_path_response_bandwidth_rad_s=max(flight_path_response_bandwidth_rad_s, 1e-6),
    )
