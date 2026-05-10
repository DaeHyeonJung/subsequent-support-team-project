from __future__ import annotations

import math

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


def straight_roll_command(_: UavState, __: float) -> float:
    return 0.0


def turn_demo_roll_command(uav: UavState, t_s: float) -> float:
    if 20.0 <= t_s <= 45.0:
        direction = 1.0 if uav.formation_id == 1 else -1.0
        return direction * math.radians(18.0)
    return 0.0
