from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SpeedController:
    kp: float
    max_accel_mps2: float
    max_decel_mps2: float

    def compute_accel_command(self, desired_speed_mps: float, current_speed_mps: float) -> float:
        speed_error_mps = desired_speed_mps - current_speed_mps
        accel_cmd_mps2 = self.kp * speed_error_mps
        return max(-self.max_decel_mps2, min(accel_cmd_mps2, self.max_accel_mps2))
