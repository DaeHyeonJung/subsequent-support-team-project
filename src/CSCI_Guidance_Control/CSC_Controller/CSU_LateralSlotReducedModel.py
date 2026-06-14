from __future__ import annotations

import math
from dataclasses import dataclass


Matrix2x2 = tuple[tuple[float, float], tuple[float, float]]
Matrix2x1 = tuple[tuple[float], tuple[float]]


@dataclass(frozen=True)
class LateralSlotReducedState:
    """Reduced lateral state for slot-tracking controller design.

    State convention:
        x = [e_x, x_dot]^T
        e_x = x - x_cmd

    A positive lateral error means the UAV is to the right of its commanded
    slot in the top-view x axis. A stabilizing heading-offset command should
    therefore turn the aircraft left, which is a positive offset from the
    nominal 90 deg cruise heading in this simulator.
    """

    lateral_error_m: float
    lateral_speed_mps: float


@dataclass(frozen=True)
class LateralSlotReducedModel:
    """Linearized lateral model for slot-tracking LQR design.

    The simulator commands heading offset indirectly through the heading and
    roll loops. For a small heading offset around nominal northbound flight:

        x_dot_ref ~= -V * psi_offset

    A first-order response from heading offset to lateral speed is approximated
    as:

        e_x_dot   = x_dot
        x_dot_dot = -a_lat * x_dot - a_lat * V * psi_offset

    Therefore:

        x_dot = A x + B u

        A = [[0, 1],
             [0, -a_lat]]

        B = [[0],
             [-a_lat * V]]
    """

    speed_mps: float
    lateral_response_bandwidth_rad_s: float

    @property
    def a_matrix(self) -> Matrix2x2:
        a_lat = self.lateral_response_bandwidth_rad_s
        return (
            (0.0, 1.0),
            (0.0, -a_lat),
        )

    @property
    def b_matrix(self) -> Matrix2x1:
        a_lat = self.lateral_response_bandwidth_rad_s
        return (
            (0.0,),
            (-a_lat * self.speed_mps,),
        )


def build_lateral_slot_reduced_state(
    *,
    lateral_position_m: float,
    commanded_lateral_position_m: float,
    speed_mps: float,
    heading_rad: float,
) -> LateralSlotReducedState:
    return LateralSlotReducedState(
        lateral_error_m=lateral_position_m - commanded_lateral_position_m,
        lateral_speed_mps=speed_mps * math.cos(heading_rad),
    )


def build_lateral_slot_reduced_model(
    *,
    speed_mps: float,
    lateral_response_bandwidth_rad_s: float,
) -> LateralSlotReducedModel:
    return LateralSlotReducedModel(
        speed_mps=max(speed_mps, 1e-6),
        lateral_response_bandwidth_rad_s=max(lateral_response_bandwidth_rad_s, 1e-6),
    )
