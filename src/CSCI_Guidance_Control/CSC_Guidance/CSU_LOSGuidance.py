from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LOSGuidanceResult:
    desired_heading_rad: float
    lookahead_x_m: float
    lookahead_y_m: float
    distance_to_target_m: float
    cross_track_error_m: float
    path_progress_ratio: float
    arrived: bool


@dataclass(frozen=True)
class LOSGuidance:
    lookahead_distance_m: float = 25.0
    arrival_radius_m: float = 3.0

    def compute_desired_heading(
        self,
        current_x_m: float,
        current_y_m: float,
        path_start_x_m: float,
        path_start_y_m: float,
        path_end_x_m: float,
        path_end_y_m: float,
        fallback_heading_rad: float,
    ) -> LOSGuidanceResult:
        path_x_m = path_end_x_m - path_start_x_m
        path_y_m = path_end_y_m - path_start_y_m
        path_length_m = math.hypot(path_x_m, path_y_m)
        distance_to_target_m = math.hypot(path_end_x_m - current_x_m, path_end_y_m - current_y_m)

        if path_length_m <= 1e-9:
            return LOSGuidanceResult(
                desired_heading_rad=fallback_heading_rad,
                lookahead_x_m=path_end_x_m,
                lookahead_y_m=path_end_y_m,
                distance_to_target_m=distance_to_target_m,
                cross_track_error_m=0.0,
                path_progress_ratio=1.0,
                arrived=distance_to_target_m <= self.arrival_radius_m,
            )

        unit_x = path_x_m / path_length_m
        unit_y = path_y_m / path_length_m
        rel_x_m = current_x_m - path_start_x_m
        rel_y_m = current_y_m - path_start_y_m
        along_track_m = rel_x_m * unit_x + rel_y_m * unit_y
        clamped_along_track_m = max(0.0, min(along_track_m, path_length_m))

        closest_x_m = path_start_x_m + clamped_along_track_m * unit_x
        closest_y_m = path_start_y_m + clamped_along_track_m * unit_y
        cross_track_error_m = math.hypot(current_x_m - closest_x_m, current_y_m - closest_y_m)

        lookahead_along_track_m = min(clamped_along_track_m + self.lookahead_distance_m, path_length_m)
        lookahead_x_m = path_start_x_m + lookahead_along_track_m * unit_x
        lookahead_y_m = path_start_y_m + lookahead_along_track_m * unit_y
        desired_heading_rad = math.atan2(lookahead_y_m - current_y_m, lookahead_x_m - current_x_m)

        return LOSGuidanceResult(
            desired_heading_rad=desired_heading_rad,
            lookahead_x_m=lookahead_x_m,
            lookahead_y_m=lookahead_y_m,
            distance_to_target_m=distance_to_target_m,
            cross_track_error_m=cross_track_error_m,
            path_progress_ratio=clamped_along_track_m / path_length_m,
            arrived=distance_to_target_m <= self.arrival_radius_m,
        )
