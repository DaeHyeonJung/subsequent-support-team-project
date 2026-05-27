from __future__ import annotations

import csv
import math
from pathlib import Path

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


def write_csv(uavs: list[UavState], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "uav_id",
                "formation_id",
                "role",
                "available",
                "battery_pct",
                "link_ok",
                "vehicle_health",
                "payload_ok",
                "x_m",
                "y_m",
                "heading_deg",
                "roll_deg",
            ]
        )
        for uav in uavs:
            for t_s, x_m, y_m, heading_rad, roll_rad, battery_pct in uav.history:
                writer.writerow(
                    [
                        f"{t_s:.2f}",
                        uav.uid,
                        uav.formation_id,
                        uav.role,
                        int(uav.available),
                        f"{battery_pct:.1f}",
                        int(uav.link_ok),
                        uav.vehicle_health,
                        int(uav.payload_ok),
                        f"{x_m:.3f}",
                        f"{y_m:.3f}",
                        f"{math.degrees(heading_rad):.3f}",
                        f"{math.degrees(roll_rad):.3f}",
                    ]
                )
