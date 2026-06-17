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
                "cell_voltage_v",
                "battery_discharge_progress",
                "link_ok",
                "vehicle_health",
                "payload_ok",
                "x_m",
                "y_m",
                "z_m",
                "speed_mps",
                "heading_deg",
                "flight_path_deg",
                "roll_deg",
                "roll_rate_deg_s",
                "flight_path_rate_deg_s",
                "longitudinal_accel_mps2",
                "vertical_accel_mps2",
            ]
        )
        for uav in uavs:
            for (
                t_s,
                x_m,
                y_m,
                z_m,
                heading_rad,
                flight_path_rad,
                speed_mps,
                roll_rad,
                roll_rate_rad_s,
                flight_path_rate_rad_s,
                longitudinal_accel_mps2,
                vertical_accel_mps2,
            ) in uav.history:
                writer.writerow(
                    [
                        f"{t_s:.2f}",
                        uav.uid,
                        uav.formation_id,
                        uav.role,
                        int(uav.available),
                        f"{uav.battery_pct:.1f}",
                        f"{uav.cell_voltage_v:.4f}",
                        f"{uav.battery_discharge_progress:.6f}",
                        int(uav.link_ok),
                        uav.vehicle_health,
                        int(uav.payload_ok),
                        f"{x_m:.3f}",
                        f"{y_m:.3f}",
                        f"{z_m:.3f}",
                        f"{speed_mps:.3f}",
                        f"{math.degrees(heading_rad):.3f}",
                        f"{math.degrees(flight_path_rad):.3f}",
                        f"{math.degrees(roll_rad):.3f}",
                        f"{math.degrees(roll_rate_rad_s):.3f}",
                        f"{math.degrees(flight_path_rate_rad_s):.3f}",
                        f"{longitudinal_accel_mps2:.3f}",
                        f"{vertical_accel_mps2:.3f}",
                    ]
                )
