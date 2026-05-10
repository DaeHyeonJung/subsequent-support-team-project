from __future__ import annotations

from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_OperationalState import UavOperationalState
from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_TelemetryMessage import UavTelemetryMessage


class AvailabilityEvaluator:
    def __init__(self, minimum_battery_pct: float = 20.0, telemetry_timeout_s: float = 2.0) -> None:
        self.minimum_battery_pct = minimum_battery_pct
        self.telemetry_timeout_s = telemetry_timeout_s

    def evaluate(self, message: UavTelemetryMessage, current_time_s: float) -> UavOperationalState:
        if current_time_s - message.time_s > self.telemetry_timeout_s:
            return UavOperationalState(message, False, "TELEMETRY_TIMEOUT")
        if not message.link_ok:
            return UavOperationalState(message, False, "LINK_LOSS")
        if message.vehicle_health != "OK":
            return UavOperationalState(message, False, f"VEHICLE_{message.vehicle_health}")
        if not message.payload_ok:
            return UavOperationalState(message, False, "PAYLOAD_FAULT")
        if message.battery_pct < self.minimum_battery_pct:
            return UavOperationalState(message, False, "LOW_BATTERY")
        return UavOperationalState(message, True, "")
