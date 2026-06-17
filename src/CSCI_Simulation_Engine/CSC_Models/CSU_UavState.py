from __future__ import annotations

from dataclasses import dataclass, field


Role = str
FormationId = int
FormalStateVector = tuple[float, float, float, float, float, float, float, float]
AuxiliaryVariables = tuple[float, float, float]
HistorySample = tuple[float, float, float, float, float, float, float, float, float, float, float, float]


@dataclass
class UavState:
    uid: str
    formation_id: FormationId
    role: Role
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    z_m: float = 80.0
    flight_path_rad: float = 0.0
    flight_path_rate_rad_s: float = 0.0
    battery_pct: float = 100.0
    battery_discharge_progress: float = 0.0
    cell_voltage_v: float = 4.283333333333333
    battery_variation_factor: float = 1.0
    available: bool = True
    link_ok: bool = True
    vehicle_health: str = "OK"
    payload_ok: bool = True
    roll_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    longitudinal_accel_mps2: float = 0.0
    vertical_accel_mps2: float = 0.0
    history: list[HistorySample] = field(default_factory=list)

    def formal_state_vector(self) -> FormalStateVector:
        """Return the reduced-order state vector used for model description.

        x = [x, y, z, V, psi, gamma, phi, phi_dot]^T
        """
        return (
            self.x_m,
            self.y_m,
            self.z_m,
            self.speed_mps,
            self.heading_rad,
            self.flight_path_rad,
            self.roll_rad,
            self.roll_rate_rad_s,
        )

    def auxiliary_variables(self) -> AuxiliaryVariables:
        """Return calculated variables kept for logging and analysis."""
        return (
            self.flight_path_rate_rad_s,
            self.longitudinal_accel_mps2,
            self.vertical_accel_mps2,
        )

    def record(self, t_s: float) -> None:
        self.history.append(
            (
                t_s,
                self.x_m,
                self.y_m,
                self.z_m,
                self.heading_rad,
                self.flight_path_rad,
                self.speed_mps,
                self.roll_rad,
                self.roll_rate_rad_s,
                self.flight_path_rate_rad_s,
                self.longitudinal_accel_mps2,
                self.vertical_accel_mps2,
            )
        )
