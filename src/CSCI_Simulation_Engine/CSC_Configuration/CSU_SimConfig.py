from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


AEROSONDE_MASS_KG = 13.5
AEROSONDE_WING_AREA_M2 = 0.55
AEROSONDE_WINGSPAN_M = 2.8956
AEROSONDE_MEAN_CHORD_M = 0.18994
AEROSONDE_AIR_DENSITY_KG_M3 = 1.225
AEROSONDE_GRAVITY_MPS2 = 9.81
AEROSONDE_CRUISE_SPEED_MPS = 25.0
AEROSONDE_CL0 = 0.28
AEROSONDE_CL_ALPHA = 3.45
AEROSONDE_CD0 = 0.03
AEROSONDE_CD_ALPHA = 0.30


def _compute_aerosonde_trim_coefficients() -> tuple[float, float]:
    cl_trim = (
        2.0
        * AEROSONDE_MASS_KG
        * AEROSONDE_GRAVITY_MPS2
        / (
            AEROSONDE_AIR_DENSITY_KG_M3
            * AEROSONDE_CRUISE_SPEED_MPS
            * AEROSONDE_CRUISE_SPEED_MPS
            * AEROSONDE_WING_AREA_M2
        )
    )
    alpha_trim_rad = (cl_trim - AEROSONDE_CL0) / AEROSONDE_CL_ALPHA
    cd_trim = AEROSONDE_CD0 + AEROSONDE_CD_ALPHA * alpha_trim_rad * alpha_trim_rad
    return cl_trim, cd_trim


AEROSONDE_CL_TRIM, AEROSONDE_CD_TRIM = _compute_aerosonde_trim_coefficients()


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.1
    duration: float = 80.0
    # Aerosonde public reference model for reduced pseudo-dynamics validation.
    speed_mps: float = AEROSONDE_CRUISE_SPEED_MPS
    roll_time_constant_s: float = 1.2
    roll_pd_kp: float = 6.25
    roll_pd_kd: float = 5.0
    speed_control_kp: float = 1.4
    flight_path_kp: float = 1.8
    flight_path_kd: float = 1.2
    max_roll_deg: float = 30.0
    max_roll_rate_deg_s: float = 90.0
    max_roll_accel_deg_s2: float = 180.0
    max_gamma_deg: float = 15.0
    max_flight_path_deg: float = 15.0
    max_flight_path_rate_deg_s: float = 20.0
    max_flight_path_accel_deg_s2: float = 35.0
    min_speed_mps: float = 15.0
    max_speed_mps: float = 35.0
    cruise_speed_mps: float = AEROSONDE_CRUISE_SPEED_MPS
    max_climb_rate_mps: float = 8.0
    max_descent_rate_mps: float = 8.0
    max_vertical_accel_mps2: float = 5.0
    max_accel_mps2: float = 3.0
    max_decel_mps2: float = 4.0
    aircraft_mass_kg: float = AEROSONDE_MASS_KG
    wing_area_m2: float = AEROSONDE_WING_AREA_M2
    wingspan_m: float = AEROSONDE_WINGSPAN_M
    mean_chord_m: float = AEROSONDE_MEAN_CHORD_M
    aspect_ratio: float = AEROSONDE_WINGSPAN_M * AEROSONDE_WINGSPAN_M / AEROSONDE_WING_AREA_M2
    oswald_efficiency: float = 0.9
    air_density_kg_m3: float = AEROSONDE_AIR_DENSITY_KG_M3
    gravity_mps2: float = AEROSONDE_GRAVITY_MPS2
    ixx_kgm2: float = 0.8244
    iyy_kgm2: float = 1.135
    izz_kgm2: float = 1.759
    ixz_kgm2: float = 0.1204
    cl0: float = AEROSONDE_CL0
    cl_alpha: float = AEROSONDE_CL_ALPHA
    cl_q: float = 0.0
    cl_delta_e: float = -0.36
    cd0: float = AEROSONDE_CD0
    cd_alpha: float = AEROSONDE_CD_ALPHA
    cd_q: float = 0.0
    cd_delta_e: float = 0.0
    cm0: float = -0.02338
    cm_alpha: float = -0.38
    cm_q: float = -3.6
    cm_delta_e: float = -0.5
    cy0: float = 0.0
    cy_beta: float = -0.98
    cy_p: float = 0.0
    cy_r: float = 0.0
    cy_delta_a: float = 0.0
    cy_delta_r: float = -0.17
    roll_moment_0: float = 0.0
    roll_moment_beta: float = -0.12
    roll_moment_p: float = -0.26
    roll_moment_r: float = 0.14
    roll_moment_delta_a: float = 0.08
    roll_moment_delta_r: float = 0.105
    cn0: float = 0.0
    cn_beta: float = 0.25
    cn_p: float = 0.022
    cn_r: float = -0.35
    cn_delta_a: float = 0.06
    cn_delta_r: float = -0.032
    # Trim-consistent representative coefficients used by the current reduced
    # pseudo-dynamics. Full CL/CD/Cm/CY/Cl/Cn derivatives are kept for future
    # longitudinal 5DOF/6DOF expansion, not forced into the present model.
    lift_coefficient: float = AEROSONDE_CL_TRIM
    drag_coefficient: float = AEROSONDE_CD_TRIM
    thrust_coefficient: float = AEROSONDE_CD_TRIM / 2.0
    output_dir: Path = Path("outputs")


def compute_trim_metrics(config: SimConfig, V: float | None = None) -> dict[str, float]:
    speed_mps = config.cruise_speed_mps if V is None else V
    qbar_pa = 0.5 * config.air_density_kg_m3 * speed_mps * speed_mps
    cl_trim = (
        2.0
        * config.aircraft_mass_kg
        * config.gravity_mps2
        / (
            config.air_density_kg_m3
            * speed_mps
            * speed_mps
            * config.wing_area_m2
        )
    )
    alpha_trim_rad = (cl_trim - config.cl0) / config.cl_alpha
    cd_trim = config.cd0 + config.cd_alpha * alpha_trim_rad * alpha_trim_rad
    drag_trim_n = qbar_pa * config.wing_area_m2 * cd_trim
    thrust_required_trim_n = drag_trim_n
    lift_over_weight = (
        qbar_pa
        * config.wing_area_m2
        * cl_trim
        / (config.aircraft_mass_kg * config.gravity_mps2)
    )
    return {
        "speed_mps": speed_mps,
        "qbar_pa": qbar_pa,
        "cl_trim": cl_trim,
        "alpha_trim_rad": alpha_trim_rad,
        "alpha_trim_deg": alpha_trim_rad * 180.0 / 3.141592653589793,
        "cd_trim": cd_trim,
        "drag_trim_n": drag_trim_n,
        "thrust_required_trim_n": thrust_required_trim_n,
        "lift_over_weight": lift_over_weight,
    }
