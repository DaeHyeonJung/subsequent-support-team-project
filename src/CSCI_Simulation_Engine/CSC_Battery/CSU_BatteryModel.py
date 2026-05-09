from __future__ import annotations

from src.CSCI_Simulation_Engine.CSC_Battery.CSU_BatteryConfig import BatteryDrainConfig


class BatteryModel:
    """Calculates UAV battery drain from elapsed time, speed, and mission role."""

    def __init__(self, config: BatteryDrainConfig | None = None) -> None:
        self.config = config or BatteryDrainConfig()

    def update_battery(
        self,
        battery_pct: float,
        dt_s: float,
        speed_mps: float,
        role: str,
    ) -> float:
        """Return the battery percentage after one simulation step."""

        if dt_s <= 0.0:
            return self._clamp_battery(battery_pct)

        drain_pct = self.calculate_drain_pct(
            dt_s=dt_s,
            speed_mps=speed_mps,
            role=role,
        )
        return self._clamp_battery(battery_pct - drain_pct)

    def calculate_drain_pct(self, dt_s: float, speed_mps: float, role: str) -> float:
        """Return the battery percentage consumed during the given time interval."""

        if dt_s <= 0.0:
            return 0.0

        role_factor = self.config.role_drain_factor.get(role, 1.0)
        speed_factor = self._calculate_speed_factor(speed_mps)

        return self.config.base_drain_pct_per_s * dt_s * role_factor * speed_factor

    def _calculate_speed_factor(self, speed_mps: float) -> float:
        safe_speed_mps = max(speed_mps, 0.0)
        speed_ratio = safe_speed_mps / self.config.reference_speed_mps
        speed_ratio = max(speed_ratio, self.config.min_speed_factor)
        return speed_ratio**self.config.speed_exponent

    @staticmethod
    def _clamp_battery(battery_pct: float) -> float:
        return max(0.0, min(100.0, battery_pct))
