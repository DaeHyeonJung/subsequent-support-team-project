from __future__ import annotations

import random
from collections.abc import Sequence
from dataclasses import dataclass

from src.CSCI_Simulation_Engine.CSC_Failure.CSU_KillEventConfig import KillEventConfig
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import FormationId, Role, UavState


@dataclass(frozen=True)
class KillEvent:
    event_index: int
    scheduled_time_s: float
    applied_time_s: float
    uid: str
    formation_id: FormationId
    role: Role


class RandomKillEventModel:

    def __init__(self, config: KillEventConfig | None = None) -> None:
        self.config = config or KillEventConfig()
        self._rng = random.Random(self.config.random_seed)
        self._executed_event_indexes: set[int] = set()
        self._formation_kill_quota: dict[FormationId, int] | None = None
        self._formation_kill_count: dict[FormationId, int] = {}
        self.kill_events: list[KillEvent] = []

    def reset(self) -> None:
        self._rng = random.Random(self.config.random_seed)
        self._executed_event_indexes.clear()
        self._formation_kill_quota = None
        self._formation_kill_count.clear()
        self.kill_events.clear()

    def scheduled_time_s(self, event_index: int) -> float:
        if event_index < 0 or event_index >= self.config.total_kills:
            raise IndexError("event_index is outside the configured kill event range")
        return self.config.start_time_s + event_index * self.config.interval_s

    def apply_due_events(self, current_time_s: float, uavs: Sequence[UavState]) -> list[KillEvent]:
        self._ensure_formation_kill_quota(uavs)
        applied_events: list[KillEvent] = []

        for event_index in self._due_event_indexes(current_time_s):
            selected_uav = self._select_kill_candidate(uavs)
            if selected_uav is None:
                raise RuntimeError("No valid UAV kill candidate is available for the configured constraints")

            self._apply_kill(selected_uav)
            self._executed_event_indexes.add(event_index)
            self._formation_kill_count[selected_uav.formation_id] += 1

            event = KillEvent(
                event_index=event_index,
                scheduled_time_s=self.scheduled_time_s(event_index),
                applied_time_s=current_time_s,
                uid=selected_uav.uid,
                formation_id=selected_uav.formation_id,
                role=selected_uav.role,
            )
            self.kill_events.append(event)
            applied_events.append(event)

        return applied_events

    def _due_event_indexes(self, current_time_s: float) -> list[int]:
        return [
            event_index
            for event_index in range(self.config.total_kills)
            if event_index not in self._executed_event_indexes
            and current_time_s + self.config.time_tolerance_s >= self.scheduled_time_s(event_index)
        ]

    def _ensure_formation_kill_quota(self, uavs: Sequence[UavState]) -> None:
        if self._formation_kill_quota is not None:
            return

        formation_ids = sorted({uav.formation_id for uav in uavs})
        if len(formation_ids) != 2:
            raise ValueError("RandomKillEventModel expects exactly two formations")

        heavy_formation_id = self._rng.choice(formation_ids)
        light_formation_id = formation_ids[0] if formation_ids[1] == heavy_formation_id else formation_ids[1]

        self._formation_kill_quota = {
            heavy_formation_id: self.config.heavy_formation_kills,
            light_formation_id: self.config.light_formation_kills,
        }
        self._formation_kill_count = {formation_id: 0 for formation_id in formation_ids}

    def _select_kill_candidate(self, uavs: Sequence[UavState]) -> UavState | None:
        if self._formation_kill_quota is None:
            raise RuntimeError("formation kill quota has not been initialized")

        viable_formation_candidates: list[tuple[FormationId, list[UavState]]] = []
        for formation_id, kill_quota in self._formation_kill_quota.items():
            if self._formation_kill_count[formation_id] >= kill_quota:
                continue

            candidates = self._kill_candidates_for_formation(uavs, formation_id)
            if candidates:
                viable_formation_candidates.append((formation_id, candidates))

        if not viable_formation_candidates:
            return None

        _, candidates = self._rng.choice(viable_formation_candidates)
        return self._rng.choice(candidates)

    def _kill_candidates_for_formation(
        self,
        uavs: Sequence[UavState],
        formation_id: FormationId,
    ) -> list[UavState]:
        return [
            uav
            for uav in uavs
            if uav.formation_id == formation_id
            and self._can_be_killed(uav)
            and self._keeps_minimum_protected_role_survivors(uav, uavs)
        ]

    def _can_be_killed(self, uav: UavState) -> bool:
        return uav.available and uav.link_ok and uav.vehicle_health == "OK"

    def _keeps_minimum_protected_role_survivors(
        self,
        candidate: UavState,
        uavs: Sequence[UavState],
    ) -> bool:
        if candidate.role != self.config.protected_role:
            return True

        surviving_protected_role_count = sum(
            1
            for uav in uavs
            if uav.role == self.config.protected_role
            and uav.vehicle_health != self.config.killed_vehicle_health
        )
        return surviving_protected_role_count - 1 >= self.config.min_surviving_protected_role

    def _apply_kill(self, uav: UavState) -> None:
        uav.available = False
        uav.link_ok = False
        uav.vehicle_health = self.config.killed_vehicle_health
        uav.payload_ok = False
