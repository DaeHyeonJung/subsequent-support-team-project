from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class SlotReferenceGeneratorConfig:
    """Reference shaping limits for formation slot commands."""

    max_lateral_reference_speed_mps: float = 6.0


@dataclass
class SlotReferenceGenerator:
    """Generate smoothed lateral slot references for LQR tracking.

    Reconfiguration can create a step change in final slot position. Feeding
    that directly to the lateral controller can produce a large initial roll
    command and lateral overshoot. This generator rate-limits the commanded x
    reference so the guidance input is continuous in time.
    """

    config: SlotReferenceGeneratorConfig = field(default_factory=SlotReferenceGeneratorConfig)
    _reference_x_by_id: dict[str, float] = field(default_factory=dict)

    def reset(self, uid: str | None = None) -> None:
        if uid is None:
            self._reference_x_by_id.clear()
            return
        self._reference_x_by_id.pop(uid, None)

    def update_lateral_reference(
        self,
        *,
        uid: str,
        current_x_m: float,
        final_target_x_m: float,
        dt_s: float,
        update_state: bool = True,
    ) -> float:
        reference_x_m = self._reference_x_by_id.get(uid, current_x_m)
        if dt_s <= 0.0:
            return reference_x_m

        error_m = final_target_x_m - reference_x_m
        max_step_m = self.config.max_lateral_reference_speed_mps * dt_s
        if abs(error_m) <= max_step_m:
            next_reference_x_m = final_target_x_m
        else:
            next_reference_x_m = reference_x_m + max_step_m * (1.0 if error_m > 0.0 else -1.0)

        if update_state:
            self._reference_x_by_id[uid] = next_reference_x_m
        return next_reference_x_m
