from __future__ import annotations


ROLE_PRIORITY_WEIGHT: dict[str, float] = {
    "recon": 1.00,
    "strike": 0.75,
    "decoy": 0.55,
}


def get_role_priority_weight(role: str) -> float:
    return ROLE_PRIORITY_WEIGHT.get(role, 0.50)
