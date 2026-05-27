from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BasicLOSGuidanceResult:
    desired_heading_rad: float
    distance_to_target_m: float
    arrived: bool


@dataclass(frozen=True)
class BasicLOSGuidance:
    arrival_radius_m: float = 3.0

    def compute_desired_heading(
        self,
        current_x_m: float,
        current_y_m: float,
        target_x_m: float,
        target_y_m: float,
        fallback_heading_rad: float,
    ) -> BasicLOSGuidanceResult:
        dx_m = target_x_m - current_x_m
        dy_m = target_y_m - current_y_m
        distance_to_target_m = math.hypot(dx_m, dy_m)

        if distance_to_target_m <= self.arrival_radius_m:
            return BasicLOSGuidanceResult(
                desired_heading_rad=fallback_heading_rad,
                distance_to_target_m=distance_to_target_m,
                arrived=True,
            )

        return BasicLOSGuidanceResult(
            desired_heading_rad=math.atan2(dy_m, dx_m),
            distance_to_target_m=distance_to_target_m,
            arrived=False,
        )
