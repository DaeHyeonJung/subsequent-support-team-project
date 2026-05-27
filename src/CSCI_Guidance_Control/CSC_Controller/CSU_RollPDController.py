from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RollPDController:
    kp: float
    kd: float

    def compute_roll_accel_command(
        self,
        desired_roll_rad: float,
        current_roll_rad: float,
        current_roll_rate_rad_s: float,
        desired_roll_rate_rad_s: float = 0.0,
    ) -> float:
        roll_error_rad = desired_roll_rad - current_roll_rad
        roll_rate_error_rad_s = desired_roll_rate_rad_s - current_roll_rate_rad_s
        return self.kp * roll_error_rad + self.kd * roll_rate_error_rad_s
