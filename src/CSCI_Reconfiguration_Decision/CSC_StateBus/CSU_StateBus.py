from __future__ import annotations

import math

from src.CSCI_Reconfiguration_Decision.CSC_Availability.CSU_AvailabilityEvaluator import AvailabilityEvaluator
from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_OperationalState import UavOperationalState
from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_TelemetryMessage import UavTelemetryMessage
from src.CSCI_Simulation_Engine.CSC_Interface.CSU_SimulationPort import SimulationSnapshot


class StateBus:
    def __init__(self, availability_evaluator: AvailabilityEvaluator | None = None) -> None:
        self.availability_evaluator = availability_evaluator or AvailabilityEvaluator()
        self._latest_telemetry: dict[str, UavTelemetryMessage] = {}

    def publish_telemetry(self, message: UavTelemetryMessage) -> None:
        self._latest_telemetry[message.uid] = message

    def update_from_simulation_snapshot(self, snapshot: SimulationSnapshot) -> None:
        for uav in snapshot.uavs:
            self.publish_telemetry(
                UavTelemetryMessage(
                    uid=uav.uid,
                    time_s=snapshot.time_s,
                    formation_id=uav.formation_id,
                    role=uav.role,
                    x_m=uav.x_m,
                    y_m=uav.y_m,
                    heading_rad=uav.heading_rad,
                    speed_mps=uav.speed_mps,
                    roll_rad=uav.roll_rad,
                    battery_pct=uav.battery_pct,
                    link_ok=uav.link_ok,
                    vehicle_health=uav.vehicle_health,
                    payload_ok=uav.payload_ok,
                )
            )

    def latest_telemetry(self) -> list[UavTelemetryMessage]:
        return list(self._latest_telemetry.values())

    def operational_states(self, current_time_s: float) -> list[UavOperationalState]:
        return [
            self.availability_evaluator.evaluate(message, current_time_s)
            for message in self.latest_telemetry()
        ]

    def available_uavs(self, current_time_s: float) -> list[UavOperationalState]:
        return [state for state in self.operational_states(current_time_s) if state.available]

    def distance_matrix_m(self) -> dict[tuple[str, str], float]:
        messages = self.latest_telemetry()
        distances: dict[tuple[str, str], float] = {}
        for left_index, left in enumerate(messages):
            for right in messages[left_index + 1 :]:
                distance_m = math.hypot(left.x_m - right.x_m, left.y_m - right.y_m)
                distances[(left.uid, right.uid)] = distance_m
                distances[(right.uid, left.uid)] = distance_m
        return distances
