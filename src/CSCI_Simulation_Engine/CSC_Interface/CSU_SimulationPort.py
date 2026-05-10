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
    heading_rad: float
    speed_mps: float
    roll_rad: float
    battery_pct: float
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
                heading_rad=uav.heading_rad,
                speed_mps=uav.speed_mps,
                roll_rad=uav.roll_rad,
                battery_pct=uav.battery_pct,
                available=uav.available,
                link_ok=uav.link_ok,
                vehicle_health=uav.vehicle_health,
                payload_ok=uav.payload_ok,
            )
            for uav in uavs
        ],
    )
