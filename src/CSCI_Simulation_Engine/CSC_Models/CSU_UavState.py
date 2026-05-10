from __future__ import annotations

from dataclasses import dataclass, field


Role = str
FormationId = int


@dataclass
class UavState:
    uid: str
    formation_id: FormationId
    role: Role
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    battery_pct: float = 100.0
    available: bool = True
    link_ok: bool = True
    vehicle_health: str = "OK"
    payload_ok: bool = True
    roll_rad: float = 0.0
    history: list[tuple[float, float, float, float, float]] = field(default_factory=list)

    def record(self, t_s: float) -> None:
        self.history.append((t_s, self.x_m, self.y_m, self.heading_rad, self.roll_rad))
