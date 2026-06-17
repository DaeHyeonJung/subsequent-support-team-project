from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from src.CSCI_Reconfiguration_Decision.CSC_RolePriority.CSU_RolePriority import (
    ROLE_PRIORITY_WEIGHT,
    get_role_priority_weight,
)
from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_OperationalState import UavOperationalState


@dataclass(frozen=True)
class CandidatePriorityConfig:
    role_score_weight: float = 0.70
    battery_score_weight: float = 0.30


@dataclass(frozen=True)
class ReconfigurationCandidate:
    uid: str
    formation_id: int
    role: str
    x_m: float
    y_m: float
    battery_pct: float
    cell_voltage_v: float
    role_score: float
    battery_score: float
    priority_score: float
    priority_reason: str


class CandidatePriorityEvaluator:
    def __init__(
        self,
        config: CandidatePriorityConfig | None = None,
        role_priority_weights: Mapping[str, float] | None = None,
    ) -> None:
        self.config = config or CandidatePriorityConfig()
        self.role_priority_weights = dict(role_priority_weights or ROLE_PRIORITY_WEIGHT)

    def set_role_priority_weight(self, role: str, weight: float) -> None:
        self.role_priority_weights[role] = self._clamp_unit(weight)

    def get_role_priority_weights(self) -> dict[str, float]:
        return dict(self.role_priority_weights)

    def rank_candidates(self, operational_states: Iterable[UavOperationalState]) -> list[ReconfigurationCandidate]:
        candidates = [
            self.evaluate_candidate(state)
            for state in operational_states
            if state.available
        ]
        return sorted(candidates, key=lambda candidate: (-candidate.priority_score, candidate.uid))

    def evaluate_candidate(self, state: UavOperationalState) -> ReconfigurationCandidate:
        telemetry = state.telemetry
        role_score = self._clamp_unit(get_role_priority_weight(telemetry.role, self.role_priority_weights))
        battery_score = self._clamp_unit(telemetry.battery_pct / 100.0)
        priority_score = self._weighted_score(role_score, battery_score)
        priority_reason = f"role={role_score:.2f}, battery={battery_score:.2f}"

        return ReconfigurationCandidate(
            uid=telemetry.uid,
            formation_id=telemetry.formation_id,
            role=telemetry.role,
            x_m=telemetry.x_m,
            y_m=telemetry.y_m,
            battery_pct=telemetry.battery_pct,
            cell_voltage_v=telemetry.cell_voltage_v,
            role_score=role_score,
            battery_score=battery_score,
            priority_score=priority_score,
            priority_reason=priority_reason,
        )

    def _weighted_score(self, role_score: float, battery_score: float) -> float:
        total_weight = self.config.role_score_weight + self.config.battery_score_weight
        if total_weight <= 0.0:
            return 0.0

        raw_score = (
            self.config.role_score_weight * role_score
            + self.config.battery_score_weight * battery_score
        ) / total_weight
        return self._clamp_unit(raw_score)

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(0.0, min(1.0, value))
