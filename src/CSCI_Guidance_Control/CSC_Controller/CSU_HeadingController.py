from __future__ import annotations

import math
from dataclasses import dataclass


G = 9.80665


def wrap_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to the [-pi, pi] range."""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class HeadingController:
    heading_gain: float = 1.4
    max_yaw_rate_rad_s: float = math.radians(40.0)
    min_speed_mps: float = 1.0

    def compute_roll_command(
        self,
        current_heading_rad: float,
        desired_heading_rad: float,
        speed_mps: float,
    ) -> float:
        heading_error_rad = wrap_angle_rad(desired_heading_rad - current_heading_rad)
        yaw_rate_cmd_rad_s = self.heading_gain * heading_error_rad
        yaw_rate_cmd_rad_s = max(
            -self.max_yaw_rate_rad_s,
            min(yaw_rate_cmd_rad_s, self.max_yaw_rate_rad_s),
        )

        safe_speed_mps = max(speed_mps, self.min_speed_mps)
        return math.atan(safe_speed_mps / G * yaw_rate_cmd_rad_s)
