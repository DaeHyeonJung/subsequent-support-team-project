from __future__ import annotations


ROLE_PRIORITY_WEIGHT: dict[str, float] = {
    "recon": 1.00,
    "strike": 0.80,
    "decoy": 0.70,
}


def get_role_priority_weight(role: str) -> float:
    return ROLE_PRIORITY_WEIGHT.get(role, 0.50)
