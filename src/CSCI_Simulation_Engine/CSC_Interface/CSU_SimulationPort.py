from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


@dataclass(frozen=True)
class UavSnapshot:
    uid: str
    formation_id: int
    role: str
    x_m: float
    y_m: float
    z_m: float
    heading_rad: float
    flight_path_rad: float
    speed_mps: float
    longitudinal_accel_mps2: float
    vertical_accel_mps2: float
    roll_rad: float
    roll_rate_rad_s: float
    flight_path_rate_rad_s: float
    battery_pct: float
    battery_discharge_progress: float
    cell_voltage_v: float
    available: bool
    link_ok: bool
    vehicle_health: str
    payload_ok: bool


@dataclass(frozen=True)
class SimulationSnapshot:
    time_s: float
    uavs: list[UavSnapshot]


class SimulationPort(Protocol):
    def publish(self, snapshot: SimulationSnapshot) -> None:
        pass


class NullSimulationPort:
    def publish(self, snapshot: SimulationSnapshot) -> None:
        _ = snapshot


def build_snapshot(time_s: float, uavs: list[UavState]) -> SimulationSnapshot:
    return SimulationSnapshot(
        time_s=time_s,
        uavs=[
            UavSnapshot(
                uid=uav.uid,
                formation_id=uav.formation_id,
                role=uav.role,
                x_m=uav.x_m,
                y_m=uav.y_m,
                z_m=uav.z_m,
                heading_rad=uav.heading_rad,
                flight_path_rad=uav.flight_path_rad,
                speed_mps=uav.speed_mps,
                longitudinal_accel_mps2=uav.longitudinal_accel_mps2,
                vertical_accel_mps2=uav.vertical_accel_mps2,
                roll_rad=uav.roll_rad,
                roll_rate_rad_s=uav.roll_rate_rad_s,
                flight_path_rate_rad_s=uav.flight_path_rate_rad_s,
                battery_pct=uav.battery_pct,
                battery_discharge_progress=uav.battery_discharge_progress,
                cell_voltage_v=uav.cell_voltage_v,
                available=uav.available,
                link_ok=uav.link_ok,
                vehicle_health=uav.vehicle_health,
                payload_ok=uav.payload_ok,
            )
            for uav in uavs
        ],
    )
