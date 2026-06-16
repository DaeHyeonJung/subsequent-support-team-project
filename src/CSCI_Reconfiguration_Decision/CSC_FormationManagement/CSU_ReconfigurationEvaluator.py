from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable

import numpy as np

from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import AllocatedSlot
from src.CSCI_Guidance_Control.CSC_CollisionAvoidance import PotentialField3DAvoidance
from src.CSCI_Guidance_Control.CSC_Controller.CSU_HeadingController import HeadingController
from src.CSCI_Simulation_Engine.CSC_Battery import BatteryModel
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Dynamics.CSU_PointMassPseudoDynamics import step_uav
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState


VelocityCommand = Callable[[np.ndarray, np.ndarray, list[np.ndarray]], np.ndarray]
TrackingCommand = Callable[
    [UavState, float, float, float, float, float, float, float],
    tuple[float, float, float],
]


@dataclass(frozen=True)
class ReconfigurationEvaluation:
    average_battery_drain_pct: float
    transition_time_s: float
    converged: bool


@dataclass
class _PreviewUav:
    uid: str
    role: str
    x_m: float
    y_m: float
    battery_pct: float
    discharge_progress: float
    battery_variation_factor: float


def evaluate_reconfiguration_plan(
    uavs: list[UavState],
    assignments: dict[str, AllocatedSlot],
    battery_model: BatteryModel,
    velocity_command: VelocityCommand,
    base_speed_mps: float,
    dt_s: float,
    max_duration_s: float = 60.0,
    arrival_tolerance_m: float = 1.0,
) -> ReconfigurationEvaluation:
    preview_uavs = [
        _PreviewUav(
            uid=uav.uid,
            role=uav.role,
            x_m=uav.x_m,
            y_m=uav.y_m,
            battery_pct=uav.battery_pct,
            discharge_progress=uav.battery_discharge_progress,
            battery_variation_factor=uav.battery_variation_factor,
        )
        for uav in uavs
        if uav.uid in assignments
    ]
    if not preview_uavs or dt_s <= 0.0:
        return ReconfigurationEvaluation(0.0, 0.0, True)

    initial_battery = {uav.uid: uav.battery_pct for uav in preview_uavs}
    targets = {
        uid: np.array([alloc.target_x, alloc.target_y], dtype=float)
        for uid, alloc in assignments.items()
        if uid in initial_battery
    }
    base_vel = np.array([0.0, base_speed_mps], dtype=float)

    elapsed_s = 0.0
    converged = False
    max_steps = max(1, int(max_duration_s / dt_s))
    for _ in range(max_steps):
        if _all_arrived(preview_uavs, targets, arrival_tolerance_m):
            converged = True
            break

        positions = {
            uav.uid: np.array([uav.x_m, uav.y_m], dtype=float)
            for uav in preview_uavs
        }
        next_positions, average_velocities = _rk4_step_preview_positions(
            positions,
            targets,
            velocity_command,
            base_vel,
            dt_s,
        )

        for uav in preview_uavs:
            uav.x_m = float(next_positions[uav.uid][0])
            uav.y_m = float(next_positions[uav.uid][1])
            vel = average_velocities[uav.uid]
            speed_mps = float(np.linalg.norm(vel))
            battery_state = battery_model.calculate_next_state(
                discharge_progress=uav.discharge_progress,
                dt_s=dt_s,
                speed_mps=speed_mps,
                role=uav.role,
                battery_variation_factor=uav.battery_variation_factor,
            )
            uav.discharge_progress = battery_state.discharge_progress
            uav.battery_pct = battery_state.battery_pct

        for target in targets.values():
            target[1] += base_speed_mps * dt_s

        elapsed_s += dt_s

    else:
        converged = _all_arrived(preview_uavs, targets, arrival_tolerance_m)

    drains = [
        max(0.0, initial_battery[uav.uid] - uav.battery_pct)
        for uav in preview_uavs
    ]
    average_drain = sum(drains) / len(drains)
    return ReconfigurationEvaluation(
        average_battery_drain_pct=average_drain,
        transition_time_s=elapsed_s,
        converged=converged,
    )


def evaluate_reconfiguration_plan_3d(
    uavs: list[UavState],
    assignments: dict[str, AllocatedSlot],
    battery_model: BatteryModel,
    tracking_command: TrackingCommand,
    heading_controller: HeadingController,
    collision_avoidance: PotentialField3DAvoidance,
    cfg: SimConfig,
    base_speed_mps: float,
    target_z_m: float = 80.0,
    max_duration_s: float = 60.0,
    arrival_tolerance_m: float = 2.0,
) -> ReconfigurationEvaluation:
    preview_uavs = [
        replace(uav, history=[])
        for uav in uavs
        if uav.uid in assignments and uav.available and uav.vehicle_health != "KILLED"
    ]
    if not preview_uavs or cfg.dt <= 0.0:
        return ReconfigurationEvaluation(0.0, 0.0, True)

    initial_battery = {uav.uid: uav.battery_pct for uav in preview_uavs}
    preview_assignments = {
        uid: alloc
        for uid, alloc in assignments.items()
        if uid in initial_battery
    }

    elapsed_s = 0.0
    converged = False
    max_steps = max(1, int(max_duration_s / cfg.dt))
    for _ in range(max_steps):
        if _all_arrived_3d(preview_uavs, preview_assignments, target_z_m, arrival_tolerance_m):
            converged = True
            break

        for uav in preview_uavs:
            alloc = preview_assignments[uav.uid]
            avoidance_vector = collision_avoidance.compute_avoidance_vector(uav, preview_uavs)
            desired_heading_rad, desired_flight_path_rad, speed_cmd_mps = tracking_command(
                uav,
                alloc.target_x,
                alloc.target_y,
                target_z_m,
                base_speed_mps,
                avoidance_vector.x_m,
                avoidance_vector.y_m,
                avoidance_vector.z_m,
            )
            roll_cmd_rad = heading_controller.compute_roll_command(
                current_heading_rad=uav.heading_rad,
                desired_heading_rad=desired_heading_rad,
                speed_mps=uav.speed_mps,
            )
            step_uav(
                uav,
                roll_cmd_rad=roll_cmd_rad,
                cfg=cfg,
                speed_cmd_mps=speed_cmd_mps,
                flight_path_cmd_rad=desired_flight_path_rad,
            )

            battery_state = battery_model.calculate_next_state(
                discharge_progress=uav.battery_discharge_progress,
                dt_s=cfg.dt,
                speed_mps=uav.speed_mps,
                role=uav.role,
                battery_variation_factor=uav.battery_variation_factor,
            )
            uav.battery_discharge_progress = battery_state.discharge_progress
            uav.battery_pct = battery_state.battery_pct

        preview_assignments = {
            uid: AllocatedSlot(
                uid=alloc.uid,
                form_id=alloc.form_id,
                slot_index=alloc.slot_index,
                target_x=alloc.target_x,
                target_y=alloc.target_y + base_speed_mps * cfg.dt,
                dx=alloc.dx,
                dy=alloc.dy,
            )
            for uid, alloc in preview_assignments.items()
        }
        elapsed_s += cfg.dt

    else:
        converged = _all_arrived_3d(preview_uavs, preview_assignments, target_z_m, arrival_tolerance_m)

    drains = [
        max(0.0, initial_battery[uav.uid] - uav.battery_pct)
        for uav in preview_uavs
    ]
    average_drain = sum(drains) / len(drains)
    return ReconfigurationEvaluation(
        average_battery_drain_pct=average_drain,
        transition_time_s=elapsed_s,
        converged=converged,
    )


def _all_arrived(
    uavs: list[_PreviewUav],
    targets: dict[str, np.ndarray],
    arrival_tolerance_m: float,
) -> bool:
    for uav in uavs:
        target = targets[uav.uid]
        current = np.array([uav.x_m, uav.y_m], dtype=float)
        if float(np.linalg.norm(target - current)) > arrival_tolerance_m:
            return False
    return True


def _rk4_step_preview_positions(
    positions: dict[str, np.ndarray],
    targets: dict[str, np.ndarray],
    velocity_command: VelocityCommand,
    base_vel: np.ndarray,
    dt_s: float,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    k1 = _preview_velocity_field(positions, targets, velocity_command, base_vel)
    k2 = _preview_velocity_field(
        _add_scaled_positions(positions, k1, dt_s * 0.5),
        targets,
        velocity_command,
        base_vel,
    )
    k3 = _preview_velocity_field(
        _add_scaled_positions(positions, k2, dt_s * 0.5),
        targets,
        velocity_command,
        base_vel,
    )
    k4 = _preview_velocity_field(
        _add_scaled_positions(positions, k3, dt_s),
        targets,
        velocity_command,
        base_vel,
    )

    next_positions: dict[str, np.ndarray] = {}
    average_velocities: dict[str, np.ndarray] = {}
    for uid, pos in positions.items():
        average_velocity = (k1[uid] + 2.0 * k2[uid] + 2.0 * k3[uid] + k4[uid]) / 6.0
        next_positions[uid] = pos + average_velocity * dt_s
        average_velocities[uid] = average_velocity
    return next_positions, average_velocities


def _preview_velocity_field(
    positions: dict[str, np.ndarray],
    targets: dict[str, np.ndarray],
    velocity_command: VelocityCommand,
    base_vel: np.ndarray,
) -> dict[str, np.ndarray]:
    velocities: dict[str, np.ndarray] = {}
    for uid, current_pos in positions.items():
        target_pos = targets[uid]
        neighbors = [pos for other_uid, pos in positions.items() if other_uid != uid]
        velocities[uid] = base_vel + velocity_command(current_pos, target_pos, neighbors)
    return velocities


def _add_scaled_positions(
    positions: dict[str, np.ndarray],
    velocities: dict[str, np.ndarray],
    scale: float,
) -> dict[str, np.ndarray]:
    return {
        uid: pos + velocities[uid] * scale
        for uid, pos in positions.items()
    }


def _all_arrived_3d(
    uavs: list[UavState],
    targets: dict[str, AllocatedSlot],
    target_z_m: float,
    arrival_tolerance_m: float,
) -> bool:
    for uav in uavs:
        target = targets[uav.uid]
        distance_m = (
            (target.target_x - uav.x_m) ** 2
            + (target.target_y - uav.y_m) ** 2
            + (target_z_m - uav.z_m) ** 2
        ) ** 0.5
        if distance_m > arrival_tolerance_m:
            return False
    return True
