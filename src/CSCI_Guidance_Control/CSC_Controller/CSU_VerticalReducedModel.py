from __future__ import annotations

from dataclasses import dataclass


Matrix3x3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]
Matrix3x1 = tuple[tuple[float], tuple[float], tuple[float]]


@dataclass(frozen=True)
class VerticalReducedState:
    """Reduced vertical state for flight-path-command LQR design.

    State convention:
        x = [e_z, gamma, gamma_dot]^T
        e_z = z - z_cmd

    A positive altitude error means the UAV is above the commanded altitude.
    A stabilizing feedback should therefore command a negative flight-path
    angle when e_z is positive.
    """

    altitude_error_m: float
    flight_path_rad: float
    flight_path_rate_rad_s: float


@dataclass(frozen=True)
class VerticalReducedModel:
    """Linearized vertical model with flight-path inner-loop dynamics.

    The simulator does not move altitude directly from gamma_cmd. It first
    passes gamma_cmd through a second-order flight-path response:

        gamma_ddot = kp * (gamma_cmd - gamma) - kd * gamma_dot

    Around small flight-path angles:

        e_z_dot = V * gamma

    Therefore:

        x_dot = A x + B u

        x = [e_z, gamma, gamma_dot]^T
        u = gamma_cmd

        A = [[0, V,   0],
             [0, 0,   1],
             [0, -kp, -kd]]

        B = [[0],
             [0],
             [kp]]
    """

    speed_mps: float
    flight_path_kp: float
    flight_path_kd: float

    @property
    def a_matrix(self) -> Matrix3x3:
        return (
            (0.0, self.speed_mps, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, -self.flight_path_kp, -self.flight_path_kd),
        )

    @property
    def b_matrix(self) -> Matrix3x1:
        return (
            (0.0,),
            (0.0,),
            (self.flight_path_kp,),
        )


def build_vertical_reduced_state(
    *,
    altitude_m: float,
    commanded_altitude_m: float,
    flight_path_rad: float,
    flight_path_rate_rad_s: float,
) -> VerticalReducedState:
    return VerticalReducedState(
        altitude_error_m=altitude_m - commanded_altitude_m,
        flight_path_rad=flight_path_rad,
        flight_path_rate_rad_s=flight_path_rate_rad_s,
    )


def build_vertical_reduced_model(
    *,
    speed_mps: float,
    flight_path_kp: float,
    flight_path_kd: float,
) -> VerticalReducedModel:
    return VerticalReducedModel(
        speed_mps=max(speed_mps, 1e-6),
        flight_path_kp=flight_path_kp,
        flight_path_kd=flight_path_kd,
    )
