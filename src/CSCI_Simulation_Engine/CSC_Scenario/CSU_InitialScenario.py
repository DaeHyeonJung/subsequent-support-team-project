from __future__ import annotations

import math

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import Role, UavState


def build_initial_uavs(speed_mps: float) -> list[UavState]:
    roles: list[Role] = ["recon", "strike", "strike", "decoy", "decoy"]
    uavs: list[UavState] = []
    slot_spacing_m = 8.0

    start_y_m = -55.0
    for formation_id, center_x_m in [(1, -28.0), (2, 28.0)]:
        for slot, role in enumerate(roles):
            uid = f"F{formation_id}-U{slot + 1}"
            uavs.append(
                UavState(
                    uid=uid,
                    formation_id=formation_id,
                    role=role,
                    x_m=center_x_m + (slot - 2) * slot_spacing_m,
                    y_m=start_y_m,
                    heading_rad=math.radians(90.0),
                    speed_mps=speed_mps,
                )
            )

    return uavs
