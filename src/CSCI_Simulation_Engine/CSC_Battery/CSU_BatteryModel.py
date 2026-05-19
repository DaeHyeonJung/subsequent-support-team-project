from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass

from src.CSCI_Simulation_Engine.CSC_Battery.CSU_BatteryConfig import BatteryDrainConfig


@dataclass(frozen=True)
class BatteryState:
    """Calculated battery state after a simulation step."""

    battery_pct: float
    discharge_progress: float
    cell_voltage_v: float
    speed_weight: float
    role_weight: float
    battery_variation_factor: float
    tau_effective_s: float


class BatteryModel:
    """Calculates UAV battery state from a LiHV discharge curve."""

    def __init__(self, config: BatteryDrainConfig | None = None) -> None:
        self.config = config or BatteryDrainConfig()
        self._voltage_curve_points = self._prepare_voltage_curve_points()
        self._discharge_percents = [point[0] for point in self._voltage_curve_points]

    def update_battery(
        self,
        battery_pct: float,
        dt_s: float,
        speed_mps: float,
        role: str,
        battery_variation_factor: float = 1.0,
    ) -> float:
        """Return the battery percentage after one simulation step.

        This method keeps the older call sites working. New code can use
        calculate_next_state() to also read discharge progress and cell voltage.
        """

        current_progress = self._progress_from_battery_pct(battery_pct)
        state = self.calculate_next_state(
            discharge_progress=current_progress,
            dt_s=dt_s,
            speed_mps=speed_mps,
            role=role,
            battery_variation_factor=battery_variation_factor,
        )
        return state.battery_pct

    def calculate_next_state(
        self,
        discharge_progress: float,
        dt_s: float,
        speed_mps: float,
        role: str,
        battery_variation_factor: float = 1.0,
    ) -> BatteryState:
        """Return the curve-based battery state after one simulation step."""

        current_progress = self._clamp_progress(discharge_progress)
        speed_weight = self.calculate_speed_weight(speed_mps)
        role_weight = self.calculate_role_weight(role)
        safe_variation_factor = max(battery_variation_factor, 1e-9)
        tau_effective_s = self.calculate_tau_effective_s(speed_weight, role_weight, safe_variation_factor)

        if dt_s <= 0.0:
            next_progress = current_progress
        else:
            next_progress = self._clamp_progress(current_progress + dt_s / tau_effective_s)

        cell_voltage_v = self.cell_voltage_at_progress(next_progress)
        battery_pct = self._battery_pct_from_progress(next_progress)
        return BatteryState(
            battery_pct=battery_pct,
            discharge_progress=next_progress,
            cell_voltage_v=cell_voltage_v,
            speed_weight=speed_weight,
            role_weight=role_weight,
            battery_variation_factor=safe_variation_factor,
            tau_effective_s=tau_effective_s,
        )

    def calculate_speed_weight(self, speed_mps: float) -> float:
        safe_speed_mps = max(speed_mps, 0.0)
        speed_ratio = safe_speed_mps / self.config.reference_speed_mps
        weighted_speed = speed_ratio**self.config.speed_exponent
        return max(weighted_speed, self.config.min_speed_factor)

    def calculate_role_weight(self, role: str) -> float:
        return self.config.role_weight.get(role, 1.0)

    def calculate_tau_effective_s(
        self,
        speed_weight: float,
        role_weight: float,
        battery_variation_factor: float = 1.0,
    ) -> float:
        total_weight = max(speed_weight * role_weight * battery_variation_factor, 1e-9)
        return self.config.tau_base_s / total_weight

    def cell_voltage_at_progress(self, discharge_progress: float) -> float:
        progress_pct = self._clamp_progress(discharge_progress) * 100.0
        return self.cell_voltage_at_discharge_percent(progress_pct)

    def cell_voltage_at_discharge_percent(self, discharge_percent: float) -> float:
        """Return interpolated single-cell voltage for a discharge percentage."""

        discharge_percent = max(0.0, min(100.0, discharge_percent))
        points = self._voltage_curve_points

        if discharge_percent <= points[0][0]:
            return points[0][1]
        if discharge_percent >= points[-1][0]:
            return points[-1][1]

        upper_idx = bisect_left(self._discharge_percents, discharge_percent)
        lower_percent, lower_voltage = points[upper_idx - 1]
        upper_percent, upper_voltage = points[upper_idx]
        span = upper_percent - lower_percent
        if span <= 0.0:
            return upper_voltage

        ratio = (discharge_percent - lower_percent) / span
        return lower_voltage + ratio * (upper_voltage - lower_voltage)

    def _prepare_voltage_curve_points(self) -> tuple[tuple[float, float], ...]:
        points = tuple(sorted(self.config.voltage_curve_points, key=lambda point: point[0]))
        if len(points) < 2:
            raise ValueError("Battery voltage curve requires at least two anchor points.")
        return points

    @staticmethod
    def _progress_from_battery_pct(battery_pct: float) -> float:
        return BatteryModel._clamp_progress(1.0 - BatteryModel._clamp_battery(battery_pct) / 100.0)

    @staticmethod
    def _battery_pct_from_progress(discharge_progress: float) -> float:
        return BatteryModel._clamp_battery((1.0 - BatteryModel._clamp_progress(discharge_progress)) * 100.0)

    @staticmethod
    def _clamp_progress(discharge_progress: float) -> float:
        return max(0.0, min(1.0, discharge_progress))

    @staticmethod
    def _clamp_battery(battery_pct: float) -> float:
        return max(0.0, min(100.0, battery_pct))
