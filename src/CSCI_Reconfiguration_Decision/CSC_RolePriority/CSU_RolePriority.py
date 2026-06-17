from __future__ import annotations

from collections.abc import Mapping


ROLE_PRIORITY_WEIGHT: dict[str, float] = {
    "recon": 1.00,
    "strike": 0.80,
    "decoy": 0.70,
}


def get_role_priority_weight(role: str, weights: Mapping[str, float] | None = None) -> float:
    source = weights or ROLE_PRIORITY_WEIGHT
    return source.get(role, 0.50)
