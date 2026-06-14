from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


@dataclass(frozen=True)
class PotentialFieldVector3D:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0

    @property
    def norm_m(self) -> float:
        return math.sqrt(self.x_m * self.x_m + self.y_m * self.y_m + self.z_m * self.z_m)

    def limited(self, max_norm_m: float) -> "PotentialFieldVector3D":
        norm_m = self.norm_m
        if norm_m <= max_norm_m or norm_m <= 1e-9:
            return self
        scale = max_norm_m / norm_m
        return PotentialFieldVector3D(
            x_m=self.x_m * scale,
            y_m=self.y_m * scale,
            z_m=self.z_m * scale,
        )


@dataclass(frozen=True)
class PotentialField3DConfig:
    protected_radius_m: float = 7.0
    command_radius_m: float = 9.5
    activation_radius_m: float = 32.0
    lookahead_time_s: float = 8.0
    minimum_distance_m: float = 1.0
    repulsive_gain_m: float = 8.0
    emergency_gain_m: float = 22.0
    max_vector_norm_m: float = 20.0
    vertical_bias_m: float = 2.0
    emergency_vertical_bias_m: float = 7.8
    horizontal_weight: float = 1.0
    vertical_weight: float = 0.62


class PotentialField3DAvoidance:
    """3D inter-UAV potential-field avoidance for guidance command shaping.

    The output is a bounded guidance-space vector. It should be combined with
    the slot tracking vector and then converted to heading, speed, and flight
    path commands by the guidance layer.
    """

    def __init__(self, config: PotentialField3DConfig | None = None) -> None:
        self.config = config or PotentialField3DConfig()

    def compute_avoidance_vector(
        self,
        ownship: UavState,
        traffic: Iterable[UavState],
    ) -> PotentialFieldVector3D:
        total_x = 0.0
        total_y = 0.0
        total_z = 0.0

        for intruder in traffic:
            if intruder.uid == ownship.uid or not intruder.available or intruder.vehicle_health == "KILLED":
                continue

            dx_m = ownship.x_m - intruder.x_m
            dy_m = ownship.y_m - intruder.y_m
            dz_m = ownship.z_m - intruder.z_m
            distance_now_m = math.sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m)
            closest_dx_m, closest_dy_m, closest_dz_m, time_to_cpa_s = self._closest_approach_vector(
                ownship,
                intruder,
                dx_m,
                dy_m,
                dz_m,
            )
            closest_distance_m = math.sqrt(
                closest_dx_m * closest_dx_m
                + closest_dy_m * closest_dy_m
                + closest_dz_m * closest_dz_m
            )

            if distance_now_m > self.config.activation_radius_m and closest_distance_m > self.config.command_radius_m:
                continue
            if time_to_cpa_s <= 1e-6 and distance_now_m > self.config.command_radius_m:
                continue

            risk_distance_m = min(distance_now_m, closest_distance_m)
            is_emergency = risk_distance_m < self.config.command_radius_m

            if closest_distance_m < distance_now_m:
                dx_m, dy_m, dz_m = closest_dx_m, closest_dy_m, closest_dz_m

            horizontal_distance_m = math.hypot(dx_m, dy_m)
            required_vertical_bias_m = (
                self.config.emergency_vertical_bias_m if is_emergency else self.config.vertical_bias_m
            )
            if abs(dz_m) < required_vertical_bias_m and horizontal_distance_m < self.config.activation_radius_m:
                dz_m = self._vertical_separation_sign(ownship.uid, intruder.uid) * required_vertical_bias_m

            weighted_dx_m = self.config.horizontal_weight * dx_m
            weighted_dy_m = self.config.horizontal_weight * dy_m
            weighted_dz_m = self.config.vertical_weight * dz_m
            distance_m = math.sqrt(
                weighted_dx_m * weighted_dx_m
                + weighted_dy_m * weighted_dy_m
                + weighted_dz_m * weighted_dz_m
            )

            if distance_m >= self.config.activation_radius_m:
                continue

            safe_distance_m = max(distance_m, self.config.minimum_distance_m)
            urgency = max(0.0, 1.0 - time_to_cpa_s / max(self.config.lookahead_time_s, 1e-6))
            protected_term = max(0.0, self.config.command_radius_m / safe_distance_m - 1.0)
            activation_term = max(0.0, (self.config.activation_radius_m - safe_distance_m) / self.config.activation_radius_m)
            magnitude_m = self.config.repulsive_gain_m * (protected_term + urgency * activation_term)
            if is_emergency:
                penetration_ratio = (self.config.command_radius_m - risk_distance_m) / self.config.command_radius_m
                magnitude_m += self.config.emergency_gain_m * (1.0 + 2.0 * max(0.0, penetration_ratio))
            total_x += magnitude_m * weighted_dx_m / safe_distance_m
            total_y += magnitude_m * weighted_dy_m / safe_distance_m
            total_z += magnitude_m * weighted_dz_m / safe_distance_m

        return PotentialFieldVector3D(total_x, total_y, total_z).limited(self.config.max_vector_norm_m)

    def _closest_approach_vector(
        self,
        ownship: UavState,
        intruder: UavState,
        dx_m: float,
        dy_m: float,
        dz_m: float,
    ) -> tuple[float, float, float, float]:
        own_vx, own_vy, own_vz = self._velocity_vector(ownship)
        int_vx, int_vy, int_vz = self._velocity_vector(intruder)
        rel_vx = own_vx - int_vx
        rel_vy = own_vy - int_vy
        rel_vz = own_vz - int_vz
        rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz
        if rel_speed_sq <= 1e-9:
            return dx_m, dy_m, dz_m, 0.0

        time_to_cpa_s = -((dx_m * rel_vx + dy_m * rel_vy + dz_m * rel_vz) / rel_speed_sq)
        time_to_cpa_s = max(0.0, min(time_to_cpa_s, self.config.lookahead_time_s))
        return (
            dx_m + rel_vx * time_to_cpa_s,
            dy_m + rel_vy * time_to_cpa_s,
            dz_m + rel_vz * time_to_cpa_s,
            time_to_cpa_s,
        )

    @staticmethod
    def _velocity_vector(uav: UavState) -> tuple[float, float, float]:
        horizontal_speed_mps = uav.speed_mps * math.cos(uav.flight_path_rad)
        return (
            horizontal_speed_mps * math.cos(uav.heading_rad),
            horizontal_speed_mps * math.sin(uav.heading_rad),
            uav.speed_mps * math.sin(uav.flight_path_rad),
        )

    @staticmethod
    def _vertical_separation_sign(ownship_uid: str, intruder_uid: str) -> float:
        return 1.0 if ownship_uid > intruder_uid else -1.0
