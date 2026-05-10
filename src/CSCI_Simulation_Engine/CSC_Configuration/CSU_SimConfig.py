from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.1
    duration: float = 80.0
    speed_mps: float = 15.0
    roll_time_constant_s: float = 1.2
    max_roll_deg: float = 30.0
    output_dir: Path = Path("outputs")
