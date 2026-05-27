from __future__ import annotations

import csv
import math
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import ttk
import numpy as np

from src.CSCI_Reconfiguration_Decision.CSC_RolePriority import CandidatePriorityEvaluator
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import (
    update_formation_assignments,
    calculate_formation_render_data,
    AllocatedSlot,
)
from src.CSCI_Simulation_Engine.CSC_Visualization.CSU_FormationRenderer import render_formation_to_canvas
from src.CSCI_Simulation_Engine.CSC_Visualization.CSU_FormationPanel import FormationPanel
from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_StateBus import StateBus
from src.CSCI_Simulation_Engine.CSC_Battery import BatteryModel
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Failure import KillEvent, RandomKillEventModel
from src.CSCI_Simulation_Engine.CSC_Interface.CSU_SimulationPort import (
    NullSimulationPort,
    SimulationPort,
    SimulationSnapshot,
    build_snapshot,
)
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import Role, UavState
from src.CSCI_Simulation_Engine.CSC_Output.CSU_SvgWriter import write_svg
from src.CSCI_Simulation_Engine.CSC_Scenario.CSU_InitialScenario import build_initial_uavs
from src.CSCI_Guidance_Control.CSU_WaypointGuidance import WaypointGuidance


def role_color(role: Role) -> str:
    return {
        "recon": "#2563eb",
        "strike": "#dc2626",
        "decoy": "#16a34a",
    }.get(role, "#111827")


class RealtimeTkViewer:
    def __init__(self, root: tk.Tk, simulation_port: SimulationPort | None = None) -> None:
        self.root = root
        self.root.title("Formation Flight 2D Realtime Simulation")
        self.closed = False
        self.simulation_port = simulation_port or NullSimulationPort()
        self.state_bus = StateBus()
        self.priority_evaluator = CandidatePriorityEvaluator()
        self.battery_model = BatteryModel()
        self.kill_event_model = RandomKillEventModel()
        self.recent_kill_events: list[KillEvent] = []
        self.failure_reconfiguration_applied = False

        self.cfg = SimConfig(dt=0.05, duration=10_000.0, speed_mps=15.0)

        # Guidance Control 초기화
        self.guidance = WaypointGuidance(
            max_speed=self.cfg.speed_mps * 0.8,
            avoidance_radius=8.0,
            repulsive_gain=0.0
        )

        self.uavs = build_initial_uavs(self.cfg.speed_mps)
        self.t_s = 0.0
        self.running = True
        self.scale_px_per_m = 6.2
        self.tail_seconds = 9.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = True
        self.drag_start: tuple[int, int] | None = None
        self.output_dir: Path
        self.csv_path: Path
        self.svg_path: Path
        self.csv_file = None
        self.csv_writer: csv.writer | None = None

        self.canvas_w = 1180
        self.canvas_h = 720
        self.canvas = tk.Canvas(root, width=self.canvas_w, height=self.canvas_h, bg="#f8fafc", highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=8, sticky="nsew")
        self.canvas.bind("<Configure>", self.resize_canvas)
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag_camera)
        self.canvas.bind("<MouseWheel>", self.zoom_with_wheel)
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)
        self.root.bind("<KeyPress>", self.handle_key)

        self.battery_frame = ttk.Frame(root, padding=(12, 10))
        self.battery_frame.grid(row=0, column=8, rowspan=2, sticky="nsew")
        ttk.Label(self.battery_frame, text="UAV Status", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w")
        self.battery_table = ttk.Frame(self.battery_frame)
        self.battery_table.grid(row=1, column=0, pady=(8, 0), sticky="nsew")
        self.battery_rows: dict[str, tuple[ttk.Label, tk.Label, ttk.Label, ttk.Label, tk.Label]] = {}
        self.build_battery_table_header()
        self.battery_frame.grid_columnconfigure(0, weight=1)
        ttk.Label(self.battery_frame, text="Reconfig Priority", font=("Arial", 12, "bold")).grid(
            row=2,
            column=0,
            pady=(14, 0),
            sticky="w",
        )
        self.priority_frame = ttk.Frame(self.battery_frame)
        self.priority_frame.grid(row=3, column=0, pady=(6, 0), sticky="ew")
        self.priority_frame.grid_columnconfigure(0, weight=1)
        self.priority_text = tk.Text(
            self.priority_frame,
            width=43,
            height=16,
            bg="#f8fafc",
            fg="#0f172a",
            font=("Consolas", 9),
            relief="flat",
            padx=0,
            pady=0,
            wrap="none",
        )
        self.priority_scrollbar = ttk.Scrollbar(
            self.priority_frame,
            orient="vertical",
            command=self.priority_text.yview,
        )
        self.priority_text.configure(yscrollcommand=self.priority_scrollbar.set)
        self.priority_text.tag_configure("section", foreground="#0f172a", font=("Consolas", 9, "bold"))
        self.priority_text.tag_configure("available", foreground="#0f172a")
        self.priority_text.tag_configure("killed", foreground="#dc2626", font=("Consolas", 9, "bold"))
        self.priority_text.tag_configure("muted", foreground="#64748b")
        self.priority_text.configure(state="disabled")
        self.priority_text.grid(row=0, column=0, sticky="ew")
        self.priority_scrollbar.grid(row=0, column=1, sticky="ns")
        self.assignments: dict[str, AllocatedSlot] = {}

        self.formation_panel = FormationPanel(self.battery_frame, on_shape_change=self.apply_formation_shape)
        self.formation_panel.frame.grid(row=4, column=0, pady=(14, 0), sticky="new")
        self.battery_frame.grid_rowconfigure(5, weight=1)

        self.start_button = ttk.Button(root, text="Pause", command=self.toggle_running)
        self.start_button.grid(row=1, column=0, padx=6, pady=8, sticky="ew")

        self.reset_button = ttk.Button(root, text="Reset", command=self.reset)
        self.reset_button.grid(row=1, column=1, padx=6, pady=8, sticky="ew")

        self.follow_var = tk.BooleanVar(value=True)
        self.follow_check = ttk.Checkbutton(root, text="Follow", variable=self.follow_var, command=self.update_follow)
        self.follow_check.grid(row=1, column=3, padx=6, pady=8, sticky="ew")

        ttk.Label(root, text="UAV Speed").grid(row=1, column=4, padx=6, pady=8, sticky="e")
        self.speed_var = tk.DoubleVar(value=self.cfg.speed_mps)
        self.speed_scale = ttk.Scale(root, from_=5.0, to=35.0, variable=self.speed_var, command=self.update_speed)
        self.speed_scale.grid(row=1, column=5, padx=6, pady=8, sticky="ew")

        ttk.Label(root, text="Tail Length").grid(row=1, column=6, padx=6, pady=8, sticky="e")
        self.tail_var = tk.DoubleVar(value=self.tail_seconds)
        self.tail_scale = ttk.Scale(root, from_=2.0, to=20.0, variable=self.tail_var, command=self.update_tail)
        self.tail_scale.grid(row=1, column=7, padx=6, pady=8, sticky="ew")

        self.status_var = tk.StringVar(value="")
        ttk.Label(root, textvariable=self.status_var).grid(row=1, column=2, padx=10, pady=8, sticky="w")

        root.grid_columnconfigure(5, weight=1)
        root.grid_columnconfigure(7, weight=1)
        root.grid_rowconfigure(0, weight=1)
        self.initialize_battery_table()
        self.start_output_session()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.tick()

    def apply_formation_shape(self, shape_type: str | None = None) -> None:
        if shape_type is None:
            shape_type = self.formation_panel.get_current_shape()
        if not shape_type:
            self.assignments = {}
            return

        # 통합 매니저를 통해 알고리즘 수행
        active_uavs = [uav for uav in self.uavs if not self.is_killed_uav(uav)]
        self.assignments = update_formation_assignments(active_uavs, shape_type, spacing_m=10.0)

        for uid, alloc in self.assignments.items():
            # 시뮬레이터에서 기체 순간이동 반영
            uav = next((u for u in self.uavs if u.uid == uid), None)
            if uav:
                pass  # 순간이동(Teleport)을 제거하고 advance_simulation에서 Guidance로 이동 처리

    def reset(self) -> None:
        self.finish_output_session()
        self.uavs = build_initial_uavs(self.speed_var.get())
        self.kill_event_model.reset()
        self.recent_kill_events.clear()
        self.t_s = 0.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = self.follow_var.get()
        self.assignments = {}
        self.failure_reconfiguration_applied = False
        self.formation_panel.clear_selection()
        self.initialize_battery_table()
        self.start_output_session()

    def toggle_running(self) -> None:
        self.running = not self.running
        self.start_button.configure(text="Pause" if self.running else "Start")

    def update_speed(self, _: str) -> None:
        speed = self.speed_var.get()
        for uav in self.uavs:
            uav.speed_mps = speed

    def update_tail(self, _: str) -> None:
        self.tail_seconds = self.tail_var.get()

    def update_follow(self) -> None:
        self.follow_camera = self.follow_var.get()

    def start_drag(self, event: tk.Event) -> None:
        self.drag_start = (event.x, event.y)
        self.follow_var.set(False)
        self.follow_camera = False

    def drag_camera(self, event: tk.Event) -> None:
        if self.drag_start is None:
            return
        last_x, last_y = self.drag_start
        dx_px = event.x - last_x
        dy_px = event.y - last_y
        self.camera_x_m -= dx_px / self.scale_px_per_m
        self.camera_y_m += dy_px / self.scale_px_per_m
        self.drag_start = (event.x, event.y)

    def zoom_with_wheel(self, event: tk.Event) -> None:
        if event.delta > 0:
            self.zoom_in(event)
        else:
            self.zoom_out(event)

    def zoom_in(self, _: tk.Event | None = None) -> None:
        self.scale_px_per_m = min(self.scale_px_per_m * 1.12, 18.0)

    def zoom_out(self, _: tk.Event | None = None) -> None:
        self.scale_px_per_m = max(self.scale_px_per_m / 1.12, 2.0)

    def handle_key(self, event: tk.Event) -> None:
        step_m = 6.0
        key = event.keysym.lower()
        if key in {"left", "a"}:
            self.camera_x_m -= step_m
        elif key in {"right", "d"}:
            self.camera_x_m += step_m
        elif key in {"up", "w"}:
            self.camera_y_m += step_m
        elif key in {"down", "s"}:
            self.camera_y_m -= step_m
        elif key in {"plus", "equal"}:
            self.zoom_in()
        elif key in {"minus", "underscore"}:
            self.zoom_out()
        elif key == "space":
            self.toggle_running()
            return
        else:
            return
        self.follow_var.set(False)
        self.follow_camera = False

    def tick(self) -> None:
        if self.closed:
            return
        if self.running:
            self.advance_simulation()
        self.draw()
        self.root.after(33, self.tick)

    def advance_simulation(self) -> None:
        current_formation_speed = self.speed_var.get()
        # 편대의 Y축 직진 기본 속도 벡터 (Feedforward Velocity)
        base_vel = np.array([0.0, current_formation_speed])

        # 기체들이 멈칫하지 않고 부드럽게 대형을 맞추도록 상대 속도의 최대치를 제한합니다.
        self.guidance.max_speed = current_formation_speed * 0.8

        # 편대 전체가 직진 비행(위쪽 방향, Y축)을 유지할 수 있도록 목표 슬롯(target)을 전방으로 이동시킵니다.
        if self.assignments:
            updated_assignments = {}
            for uid, alloc in self.assignments.items():
                updated_assignments[uid] = AllocatedSlot(
                    uid=alloc.uid,
                    form_id=alloc.form_id,
                    slot_index=alloc.slot_index,
                    target_x=alloc.target_x,
                    target_y=alloc.target_y + current_formation_speed * self.cfg.dt,
                    dx=alloc.dx,
                    dy=alloc.dy
                )
            self.assignments = updated_assignments

        # 1. 모든 기체의 다음 속도(명령)를 먼저 계산 (동기적 업데이트)
        vel_commands = []
        for uav in self.uavs:
            current_pos = np.array([uav.x_m, uav.y_m])

            if uav.uid in self.assignments:
                alloc = self.assignments[uav.uid]
                target_pos = np.array([alloc.target_x, alloc.target_y])
            else:
                # 할당되지 않은 기체는 현재 위치를 유지하되 기본 속도로 전진
                target_pos = current_pos

            # 자신을 제외한 다른 기체들을 장애물(이웃)로 등록하여 충돌 회피 처리
            neighbors = [np.array([other.x_m, other.y_m]) for other in self.uavs if other.uid != uav.uid]

            # guidance는 절대 좌표 기반이 아닌 '슬롯까지의 상대적인 속도 보정값' 역할 수행
            rel_vel = self.guidance.compute_velocity_command(current_pos, target_pos, neighbors)

            # 최종 속도 = 편대 기본 직진 속도 + 슬롯 추적 및 회피를 위한 상대 속도
            vel = base_vel + rel_vel
            vel_commands.append(vel)

        if not self.assignments:
            vel_commands = [base_vel for _ in self.uavs]

        # 2. 계산된 속도를 바탕으로 실제 기체의 위치와 기수(Heading) 업데이트
        for i, uav in enumerate(self.uavs):
            uav.record(self.t_s)
            if self.is_killed_uav(uav):
                continue

            vel = vel_commands[i]
            uav.x_m += vel[0] * self.cfg.dt
            uav.y_m += vel[1] * self.cfg.dt
            uav.speed_mps = float(np.linalg.norm(vel))

            if uav.speed_mps > 0.1:
                uav.heading_rad = math.atan2(vel[1], vel[0])

            battery_state = self.battery_model.calculate_next_state(
                discharge_progress=uav.battery_discharge_progress,
                dt_s=self.cfg.dt,
                speed_mps=uav.speed_mps,
                role=uav.role,
                battery_variation_factor=uav.battery_variation_factor,
            )
            uav.battery_discharge_progress = battery_state.discharge_progress
            uav.cell_voltage_v = battery_state.cell_voltage_v
            uav.battery_pct = battery_state.battery_pct

        self.recent_kill_events = self.kill_event_model.apply_due_events(self.t_s, self.uavs)
        if (
            not self.failure_reconfiguration_applied
            and len(self.kill_event_model.kill_events) >= self.kill_event_model.config.total_kills
        ):
            for uav in self.uavs:
                if not self.is_killed_uav(uav):
                    uav.formation_id = 1
            self.failure_reconfiguration_applied = True
            self.formation_panel.set_selection("line")
            self.apply_formation_shape("line")
        snapshot = build_snapshot(self.t_s, self.uavs)
        self.state_bus.update_from_simulation_snapshot(snapshot)
        self.simulation_port.publish(snapshot)
        self.write_snapshot_csv(snapshot)
        self.t_s += self.cfg.dt

    @staticmethod
    def is_killed_uav(uav: UavState) -> bool:
        return uav.vehicle_health == "KILLED"

    def start_output_session(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.output_dir = Path("outputs") / f"realtime_{timestamp}"
        self.csv_path = self.output_dir / "trajectory.csv"
        self.svg_path = self.output_dir / "trajectory.svg"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "time_s",
                "uav_id",
                "formation_id",
                "role",
                "available",
                "availability_reason",
                "battery_pct",
                "cell_voltage_v",
                "battery_discharge_progress",
                "link_ok",
                "vehicle_health",
                "payload_ok",
                "x_m",
                "y_m",
                "heading_deg",
                "roll_deg",
            ]
        )

    def write_snapshot_csv(self, snapshot: SimulationSnapshot) -> None:
        if self.csv_writer is None or self.csv_file is None:
            return

        operational_states = {
            state.telemetry.uid: state for state in self.state_bus.operational_states(snapshot.time_s)
        }
        for uav in snapshot.uavs:
            operational_state = operational_states.get(uav.uid)
            available = operational_state.available if operational_state is not None else uav.available
            unavailable_reason = operational_state.unavailable_reason if operational_state is not None else ""
            self.csv_writer.writerow(
                [
                    f"{snapshot.time_s:.2f}",
                    uav.uid,
                    uav.formation_id,
                    uav.role,
                    int(available),
                    unavailable_reason,
                    f"{uav.battery_pct:.1f}",
                    f"{uav.cell_voltage_v:.4f}",
                    f"{uav.battery_discharge_progress:.6f}",
                    int(uav.link_ok),
                    uav.vehicle_health,
                    int(uav.payload_ok),
                    f"{uav.x_m:.3f}",
                    f"{uav.y_m:.3f}",
                    f"{math.degrees(uav.heading_rad):.3f}",
                    f"{math.degrees(uav.roll_rad):.3f}",
                ]
            )
        self.csv_file.flush()

    def finish_output_session(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        if any(uav.history for uav in self.uavs):
            write_svg(self.uavs, self.svg_path)

    def close(self) -> None:
        self.closed = True
        self.finish_output_session()
        self.root.destroy()

    def resize_canvas(self, event: tk.Event) -> None:
        self.canvas_w = max(event.width, 1)
        self.canvas_h = max(event.height, 1)

    def world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        cx = self.canvas_w * 0.48
        cy = self.canvas_h * 0.52
        sx = cx + (x_m - self.camera_x_m) * self.scale_px_per_m
        sy = cy - (y_m - self.camera_y_m) * self.scale_px_per_m
        return sx, sy

    def update_camera(self) -> None:
        if not self.uavs or not self.follow_camera:
            return
        tracked_uavs = [uav for uav in self.uavs if not self.is_killed_uav(uav)]
        if not tracked_uavs:
            tracked_uavs = self.uavs

        lead_x = sum(uav.x_m for uav in tracked_uavs) / len(tracked_uavs)
        lead_y = sum(uav.y_m for uav in tracked_uavs) / len(tracked_uavs)
        self.camera_x_m += (lead_x - self.camera_x_m) * 0.04
        self.camera_y_m += (lead_y - self.camera_y_m) * 0.04

    def draw(self) -> None:
        self.update_camera()
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_slots()
        self.draw_uavs()
        self.draw_hud()
        self.update_battery_table()
        self.update_priority_panel()

    def initialize_battery_table(self) -> None:
        for widget in self.battery_table.winfo_children():
            widget.destroy()

        self.battery_rows.clear()
        self.build_battery_table_header()
        for uav in self.uavs:
            row_idx = len(self.battery_rows) + 1
            form_label = ttk.Label(self.battery_table, text=f"F{uav.formation_id}", width=6, anchor="center")
            role_label = tk.Label(
                self.battery_table,
                text=uav.role,
                width=8,
                anchor="center",
                bg=role_color(uav.role),
                fg="#ffffff",
            )
            battery_label = ttk.Label(self.battery_table, text=f"{uav.battery_pct:5.1f}%", width=9, anchor="e")
            voltage_label = ttk.Label(self.battery_table, text=f"{uav.cell_voltage_v:4.2f}V", width=9, anchor="e")
            status_label = tk.Label(
                self.battery_table,
                text="OK",
                width=8,
                anchor="center",
                bg="#e2e8f0",
                fg="#0f172a",
            )

            form_label.grid(row=row_idx, column=0, padx=(0, 4), pady=2, sticky="ew")
            role_label.grid(row=row_idx, column=1, padx=4, pady=2, sticky="ew")
            battery_label.grid(row=row_idx, column=2, padx=4, pady=2, sticky="ew")
            voltage_label.grid(row=row_idx, column=3, padx=4, pady=2, sticky="ew")
            status_label.grid(row=row_idx, column=4, padx=(4, 0), pady=2, sticky="ew")
            self.battery_rows[uav.uid] = (form_label, role_label, battery_label, voltage_label, status_label)

    def update_battery_table(self) -> None:
        for uav in self.uavs:
            if uav.uid not in self.battery_rows:
                self.initialize_battery_table()
                return

            form_label, role_label, battery_label, voltage_label, status_label = self.battery_rows[uav.uid]
            form_label.configure(text=f"F{uav.formation_id}")
            role_label.configure(text=uav.role)
            battery_label.configure(text=f"{uav.battery_pct:5.1f}%")
            voltage_label.configure(text=f"{uav.cell_voltage_v:4.2f}V")

            if self.is_killed_uav(uav):
                role_label.configure(bg="#000000", fg="#ffffff")
                status_label.configure(text="KILLED", bg="#dc2626", fg="#ffffff")
            else:
                role_label.configure(bg=role_color(uav.role), fg="#ffffff")
                status_label.configure(text="OK", bg="#e2e8f0", fg="#0f172a")

    def update_priority_panel(self) -> None:
        candidates = self.priority_evaluator.rank_candidates(self.state_bus.operational_states(self.t_s))
        if not candidates:
            self.set_priority_text([("Waiting for telemetry", "muted")])
            return

        rows: list[tuple[str, str]] = [("AVAILABLE CANDIDATES", "section"), ("Rank UAV    Role   Score  Batt", "muted")]
        for rank, candidate in enumerate(candidates, start=1):
            rows.append(
                (
                f"{rank:>2}.  {candidate.uid:<6} {candidate.role:<6} "
                    f"{candidate.priority_score:.2f}  {candidate.battery_pct:5.1f}%",
                    "available",
                )
            )

        killed_uavs = [uav for uav in self.uavs if self.is_killed_uav(uav)]
        if killed_uavs:
            rows.append(("", "muted"))
            rows.append(("KILLED / EXCLUDED", "section"))
            for uav in killed_uavs:
                rows.append((f"XX  {uav.uid:<6} {uav.role:<6} KILLED", "killed"))

        self.set_priority_text(rows)

    def set_priority_text(self, rows: list[tuple[str, str]]) -> None:
        self.priority_text.configure(state="normal")
        self.priority_text.delete("1.0", tk.END)
        for text, tag in rows:
            self.priority_text.insert(tk.END, f"{text}\n", tag)
        self.priority_text.configure(state="disabled")

    def build_battery_table_header(self) -> None:
        headers = [("Form", 6), ("Role", 8), ("Battery", 9), ("Voltage", 9), ("Status", 8)]
        for col_idx, (text, width) in enumerate(headers):
            label = ttk.Label(self.battery_table, text=text, width=width, anchor="center", font=("Arial", 9, "bold"))
            label.grid(row=0, column=col_idx, padx=4, pady=(0, 4), sticky="ew")

    def draw_grid(self) -> None:
        grid_m = 10.0
        left_m = self.camera_x_m - self.canvas_w / self.scale_px_per_m / 2.0
        right_m = self.camera_x_m + self.canvas_w / self.scale_px_per_m / 2.0
        bottom_m = self.camera_y_m - self.canvas_h / self.scale_px_per_m / 2.0
        top_m = self.camera_y_m + self.canvas_h / self.scale_px_per_m / 2.0

        x = math.floor(left_m / grid_m) * grid_m
        while x <= right_m:
            sx, _ = self.world_to_screen(x, 0.0)
            color = "#cbd5e1" if abs(x) < 1e-6 else "#e2e8f0"
            self.canvas.create_line(sx, 0, sx, self.canvas_h, fill=color)
            self.canvas.create_text(sx + 3, self.canvas_h - 18, text=f"{x:.0f}", fill="#64748b", anchor="w", font=("Arial", 9))
            x += grid_m

        y = math.floor(bottom_m / grid_m) * grid_m
        while y <= top_m:
            _, sy = self.world_to_screen(0.0, y)
            color = "#cbd5e1" if abs(y) < 1e-6 else "#e2e8f0"
            self.canvas.create_line(0, sy, self.canvas_w, sy, fill=color)
            self.canvas.create_text(8, sy - 3, text=f"{y:.0f}", fill="#64748b", anchor="w", font=("Arial", 9))
            y += grid_m

    def draw_slots(self) -> None:
        """현재 편대들의 중심을 계산하여 각 슬롯의 위치를 화면에 시각적으로 표시합니다."""
        if not hasattr(self, 'assignments') or not self.assignments:
            return

        render_data = calculate_formation_render_data(self.uavs, self.assignments)
        # 독립된 렌더러 모듈에 캔버스와 렌더링 데이터를 넘겨 그리기 위임
        render_formation_to_canvas(self.canvas, render_data, self.world_to_screen)

    def draw_uavs(self) -> None:
        for uav in self.uavs:
            color = role_color(uav.role)
            self.draw_tail(uav, color)
            if self.is_killed_uav(uav):
                self.draw_killed_uav_marker(uav)
            else:
                self.draw_uav_body(uav, color)

    def draw_tail(self, uav: UavState, color: str) -> None:
        cutoff_t = self.t_s - self.tail_seconds
        recent = [sample for sample in uav.history if sample[0] >= cutoff_t]
        if len(recent) < 2:
            return

        max_segments = len(recent) - 1
        for idx in range(max_segments):
            t0, x0, y0, _, _, _ = recent[idx]
            _, x1, y1, _, _, _ = recent[idx + 1]
            sx0, sy0 = self.world_to_screen(x0, y0)
            sx1, sy1 = self.world_to_screen(x1, y1)
            age_ratio = (t0 - cutoff_t) / max(self.tail_seconds, 0.1)
            width = 1 + 3 * age_ratio
            self.canvas.create_line(sx0, sy0, sx1, sy1, fill=color, width=width, stipple="gray50")

    def draw_uav_body(self, uav: UavState, color: str) -> None:
        sx, sy = self.world_to_screen(uav.x_m, uav.y_m)
        body_len = 18
        wing_span = 24
        heading = -uav.heading_rad

        points_body = [
            (body_len, 0),
            (-body_len * 0.65, -wing_span * 0.55),
            (-body_len * 0.35, 0),
            (-body_len * 0.65, wing_span * 0.55),
        ]
        rotated = []
        for px, py in points_body:
            rx = px * math.cos(heading) - py * math.sin(heading)
            ry = px * math.sin(heading) + py * math.cos(heading)
            rotated.extend([sx + rx, sy + ry])

        self.canvas.create_polygon(rotated, fill="#0f172a", outline=color, width=2)
        self.canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=color, outline="#ffffff", width=1)
        self.canvas.create_line(sx, sy, sx, sy - 30, fill="#334155", width=1)

    def draw_killed_uav_marker(self, uav: UavState) -> None:
        sx, sy = self.world_to_screen(uav.x_m, uav.y_m)
        radius = 14
        self.canvas.create_line(sx - radius, sy - radius, sx + radius, sy + radius, fill="#111827", width=3)
        self.canvas.create_line(sx - radius, sy + radius, sx + radius, sy - radius, fill="#111827", width=3)
        self.canvas.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, outline="#dc2626", width=2)
        self.canvas.create_text(
            sx + 22,
            sy - 20,
            text=f"{uav.uid} KILLED",
            fill="#991b1b",
            anchor="w",
            font=("Arial", 10, "bold"),
        )

    def draw_hud(self) -> None:
        self.status_var.set(
            f"t = {self.t_s:5.1f} sec    speed = {self.speed_var.get():4.1f} m/s    tail = {self.tail_seconds:4.1f} sec"
        )


def main() -> None:
    root = tk.Tk()
    RealtimeTkViewer(root)
    root.mainloop()
