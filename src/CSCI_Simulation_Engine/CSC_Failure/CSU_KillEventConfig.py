from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class KillEventConfig:
    """Configuration for scheduled random UAV kill events."""

    start_time_s: float = 5.0
    interval_s: float = 0.5
    total_kills: int = 3
    heavy_formation_kills: int = 2
    light_formation_kills: int = 1
    protected_role: str = "recon"
    min_surviving_protected_role: int = 1
    killed_vehicle_health: str = "KILLED"
    random_seed: int | None = None
    time_tolerance_s: float = 1e-9

    def __post_init__(self) -> None:
        if self.start_time_s < 0.0:
            raise ValueError("start_time_s must be non-negative")
        if self.interval_s <= 0.0:
            raise ValueError("interval_s must be positive")
        if self.total_kills <= 0:
            raise ValueError("total_kills must be positive")
        if self.heavy_formation_kills <= 0:
            raise ValueError("heavy_formation_kills must be positive")
        if self.light_formation_kills <= 0:
            raise ValueError("light_formation_kills must be positive")
        if self.heavy_formation_kills + self.light_formation_kills != self.total_kills:
            raise ValueError("formation kill counts must add up to total_kills")
        if self.min_surviving_protected_role < 0:
            raise ValueError("min_surviving_protected_role must be non-negative")
        if self.time_tolerance_s < 0.0:
            raise ValueError("time_tolerance_s must be non-negative")
