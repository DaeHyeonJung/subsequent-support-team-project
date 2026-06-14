from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BasicLOS3DGuidanceResult:
    desired_heading_rad: float
    desired_flight_path_rad: float
    distance_to_target_m: float
    horizontal_distance_to_target_m: float
    arrived: bool


@dataclass(frozen=True)
class BasicLOS3DGuidance:
    arrival_radius_m: float = 4.0

    def compute_desired_command(
        self,
        current_x_m: float,
        current_y_m: float,
        current_z_m: float,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        fallback_heading_rad: float,
        fallback_flight_path_rad: float,
    ) -> BasicLOS3DGuidanceResult:
        dx_m = target_x_m - current_x_m
        dy_m = target_y_m - current_y_m
        dz_m = target_z_m - current_z_m
        horizontal_distance_m = math.hypot(dx_m, dy_m)
        distance_m = math.sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m)

        if distance_m <= self.arrival_radius_m:
            return BasicLOS3DGuidanceResult(
                desired_heading_rad=fallback_heading_rad,
                desired_flight_path_rad=fallback_flight_path_rad,
                distance_to_target_m=distance_m,
                horizontal_distance_to_target_m=horizontal_distance_m,
                arrived=True,
            )

        return BasicLOS3DGuidanceResult(
            desired_heading_rad=math.atan2(dy_m, dx_m),
            desired_flight_path_rad=math.atan2(dz_m, max(horizontal_distance_m, 1e-6)),
            distance_to_target_m=distance_m,
            horizontal_distance_to_target_m=horizontal_distance_m,
            arrived=False,
        )
