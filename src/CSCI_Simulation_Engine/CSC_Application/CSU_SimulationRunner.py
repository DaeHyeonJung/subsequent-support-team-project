from __future__ import annotations

from collections.abc import Callable

from src.CSCI_Simulation_Engine.CSC_Battery import BatteryModel
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Dynamics.CSU_PointMassPseudoDynamics import step_uav
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState
from src.CSCI_Simulation_Engine.CSC_Scenario.CSU_InitialScenario import build_initial_uavs


def run_simulation(
    cfg: SimConfig,
    roll_command: Callable[[UavState, float], float],
) -> list[UavState]:
    uavs = build_initial_uavs(cfg.speed_mps)
    battery_model = BatteryModel()
    steps = int(cfg.duration / cfg.dt)

    for step in range(steps + 1):
        t_s = step * cfg.dt
        for uav in uavs:
            uav.record(t_s)
            step_uav(uav, roll_command(uav, t_s), cfg)
            uav.battery_pct = battery_model.update_battery(
                battery_pct=uav.battery_pct,
                dt_s=cfg.dt,
                speed_mps=uav.speed_mps,
                role=uav.role,
            )

    return uavs
