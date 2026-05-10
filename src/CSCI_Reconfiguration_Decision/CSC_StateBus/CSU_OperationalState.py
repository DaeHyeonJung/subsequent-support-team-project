from __future__ import annotations

from dataclasses import dataclass

from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_TelemetryMessage import UavTelemetryMessage


@dataclass(frozen=True)
class UavOperationalState:
    telemetry: UavTelemetryMessage
    available: bool
    unavailable_reason: str
