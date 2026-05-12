from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class BatteryDrainConfig:
    """Configuration values used by the UAV battery drain model."""

    # Battery percentage points consumed per second at reference speed.
    base_drain_pct_per_s: float = 0.05

    # Speed used as the baseline for speed-dependent battery consumption.
    reference_speed_mps: float = 15.0

    # Keeps a small standby drain even when the UAV is moving slowly.
    min_speed_factor: float = 0.2

    # Larger values make high-speed flight consume battery more aggressively.
    speed_exponent: float = 1.3

    # Payload/mission-role factor. Larger factor means faster battery drain.
    role_drain_factor: dict[str, float] = field(
        default_factory=lambda: {
            "recon": 1.40,
            "strike": 1.20,
            "decoy": 1.10,
        }
    )
