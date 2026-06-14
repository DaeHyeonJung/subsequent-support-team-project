from __future__ import annotations

import csv
import math
import tkinter as tk
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from tkinter import ttk
import numpy as np

from src.CSCI_Reconfiguration_Decision.CSC_RolePriority import CandidatePriorityEvaluator, ROLE_PRIORITY_WEIGHT
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import (
    update_formation_assignments,
    calculate_formation_render_data,
    AllocatedSlot,
)
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_ReconfigurationEvaluator import (
    evaluate_reconfiguration_plan,
)
from src.CSCI_Simulation_Engine.CSC_Visualization.CSU_FormationRenderer import render_formation_to_canvas
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
        self.confirmed_role_weights = self.priority_evaluator.get_role_priority_weights()
        self.priority_locked = False
        self.locked_priority_rows: list[tuple[str, str]] = []
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
        self.display_formation_ids = {uav.uid: uav.formation_id for uav in self.uavs}
        self.t_s = 0.0
        self.running = True
        self.scale_px_per_m = 6.2
        self.tail_seconds = 2.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = True
        self.drag_start: tuple[int, int] | None = None
        self.output_dir: Path
        self.csv_path: Path
        self.svg_path: Path
        self.csv_file = None
        self.csv_writer: csv.writer | None = None

        self.expanded_canvas_height = 600
        self.compact_canvas_height = 240
        self.canvas_w = 460
        self.canvas_h = self.expanded_canvas_height
        self.top_view_frame = ttk.LabelFrame(root, text="2D TOP VIEW", padding=(2, 2))
        self.top_view_frame.grid(row=1, column=0, columnspan=4, padx=(6, 3), pady=(0, 0), sticky="nsew")
        self.top_view_frame.grid_rowconfigure(0, weight=1)
        self.top_view_frame.grid_columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            self.top_view_frame,
            width=self.canvas_w,
            height=self.canvas_h,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", self.resize_canvas)
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag_camera)
        self.canvas.bind("<MouseWheel>", self.zoom_with_wheel)
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)
        self.root.bind("<KeyPress>", self.handle_key)

        self.preview_canvas_w = 460
        self.preview_canvas_h = self.expanded_canvas_height
        self.preview_frame = ttk.LabelFrame(root, text="FORMATION PREVIEW", padding=(2, 2))
        self.preview_frame.grid(row=1, column=4, columnspan=4, padx=(3, 6), pady=(0, 0), sticky="nsew")
        self.preview_frame.grid_rowconfigure(0, weight=1)
        self.preview_frame.grid_columnconfigure(0, weight=1)
        self.preview_canvas = tk.Canvas(
            self.preview_frame,
            width=self.preview_canvas_w,
            height=self.preview_canvas_h,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        self.preview_canvas.grid(row=0, column=0, sticky="nsew")
        self.preview_canvas.bind("<Configure>", self.resize_preview_canvas)

        self.sidebar_canvas = tk.Canvas(root, width=360, highlightthickness=0)
        self.sidebar_scrollbar = ttk.Scrollbar(root, orient="vertical", command=self.sidebar_canvas.yview)
        self.sidebar_canvas.configure(yscrollcommand=self.sidebar_scrollbar.set)
        self.sidebar_canvas.grid(row=1, column=8, rowspan=2, sticky="nsew")
        self.sidebar_scrollbar.grid(row=1, column=9, rowspan=2, sticky="ns")

        self.battery_frame = ttk.Frame(self.sidebar_canvas, padding=(12, 10))
        self.sidebar_window = self.sidebar_canvas.create_window((0, 0), window=self.battery_frame, anchor="nw")
        self.battery_frame.bind("<Configure>", self.update_sidebar_scrollregion)
        self.sidebar_canvas.bind("<Configure>", self.resize_sidebar_frame)
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
            height=14,
            bg="#f8fafc",
            fg="#0f172a",
            font=("Consolas", 12),
            relief="flat",
            padx=0,
            pady=0,
            wrap="none",
        )
        self.priority_text.tag_configure("section", foreground="#0f172a", font=("Consolas", 11, "bold"))
        self.priority_text.tag_configure("available", foreground="#0f172a")
        self.priority_text.tag_configure("killed", foreground="#dc2626", font=("Consolas", 11, "bold"))
        self.priority_text.tag_configure("muted", foreground="#64748b")
        self.priority_text.configure(state="disabled")
        self.priority_text.grid(row=0, column=0, sticky="ew")
        self.assignments: dict[str, AllocatedSlot] = {}
        self.current_shape_type: str | None = None

        self.build_role_weight_controls()
        self.battery_frame.grid_rowconfigure(5, weight=1)

        self.start_button = ttk.Button(root, text="Pause", command=self.toggle_running)
        self.start_button.grid(row=0, column=0, padx=6, pady=8, sticky="ew")

        self.reset_button = ttk.Button(root, text="Reset", command=self.reset)
        self.reset_button.grid(row=0, column=1, padx=6, pady=8, sticky="ew")

        self.follow_var = tk.BooleanVar(value=True)
        self.follow_check = ttk.Checkbutton(root, text="Follow", variable=self.follow_var, command=self.update_follow)
        self.follow_check.grid(row=0, column=4, padx=6, pady=8, sticky="ew")

        ttk.Label(root, text="UAV Speed").grid(row=0, column=5, padx=6, pady=8, sticky="e")
        self.speed_var = tk.DoubleVar(value=self.cfg.speed_mps)
        self.speed_scale = ttk.Scale(root, from_=5.0, to=35.0, variable=self.speed_var, command=self.update_speed)
        self.speed_scale.grid(row=0, column=6, columnspan=3, padx=6, pady=8, sticky="ew")

        self.status_var = tk.StringVar(value="")
        ttk.Label(root, textvariable=self.status_var, width=34).grid(
            row=0,
            column=2,
            columnspan=2,
            padx=10,
            pady=8,
            sticky="w",
        )

        for column_idx in range(8):
            root.grid_columnconfigure(column_idx, weight=1, uniform="simulation")
        root.grid_rowconfigure(0, weight=0)
        root.grid_rowconfigure(1, weight=1)
        root.grid_rowconfigure(2, weight=0)
        self.build_formation_shape_selector(root)
        self.initialize_battery_table()
        self.start_output_session()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.tick()

    def update_sidebar_scrollregion(self, _: tk.Event | None = None) -> None:
        self.sidebar_canvas.configure(scrollregion=self.sidebar_canvas.bbox("all"))

    def resize_sidebar_frame(self, event: tk.Event) -> None:
        self.sidebar_canvas.itemconfigure(self.sidebar_window, width=event.width)

    def build_formation_shape_selector(self, parent: tk.Widget) -> None:
        self.shape_selector_visible = False
        self.shape_selector_frame = ttk.Frame(parent, padding=(6, 6))
        self.shape_preview_canvases: dict[str, tk.Canvas] = {}
        for column_idx in range(4):
            self.shape_selector_frame.grid_columnconfigure(column_idx, weight=1, uniform="shape")
        self.shape_selector_frame.grid_rowconfigure(0, weight=1)

        shapes = [
            ("wedge", "Wedge"),
            ("line", "Line"),
            ("column", "Column"),
            ("staggered_column", "Staggered Column"),
        ]
        for col_idx, (shape_type, label) in enumerate(shapes):
            canvas = tk.Canvas(
                self.shape_selector_frame,
                height=280,
                bg="#f8fafc",
                highlightthickness=1,
                highlightbackground="#94a3b8",
                cursor="hand2",
            )
            canvas.grid(row=0, column=col_idx, padx=4, pady=2, sticky="nsew")
            canvas.bind("<Button-1>", lambda _event, selected_shape=shape_type: self.select_formation_shape(selected_shape))
            canvas.bind("<Configure>", lambda _event, selected_shape=shape_type: self.draw_formation_shape_preview(selected_shape))
            self.shape_preview_canvases[shape_type] = canvas

        self.shape_selector_frame.grid(row=2, column=0, columnspan=8, sticky="nsew")
        self.shape_selector_frame.grid_remove()

    def refresh_formation_shape_previews(self) -> None:
        for shape_type in self.shape_preview_canvases:
            self.draw_formation_shape_preview(shape_type)

    def draw_formation_shape_preview(self, shape_type: str) -> None:
        canvas = self.shape_preview_canvases.get(shape_type)
        if canvas is None:
            return

        canvas.delete("all")
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)
        canvas.create_rectangle(0, 0, width, height, fill="#f8fafc", outline="#94a3b8")

        label = {
            "wedge": "Wedge",
            "line": "Line",
            "column": "Column",
            "staggered_column": "Staggered Column",
        }.get(shape_type, shape_type)
        canvas.create_text(width / 2, 15, text=label, fill="#0f172a", font=("Arial", 10, "bold"))

        active_uavs = [
            replace(uav, formation_id=1)
            for uav in self.uavs
            if not self.is_killed_uav(uav)
        ]
        if not active_uavs:
            return

        assignments = update_formation_assignments(
            active_uavs,
            shape_type,
            spacing_m=10.0,
            role_weights=self.confirmed_role_weights,
            current_speed=self.speed_var.get(),
        )
        preview_guidance = WaypointGuidance(
            max_speed=self.speed_var.get() * 0.8,
            avoidance_radius=self.guidance.avoidance_radius,
            repulsive_gain=self.guidance.repulsive_gain,
        )
        evaluation = evaluate_reconfiguration_plan(
            active_uavs,
            assignments,
            self.battery_model,
            preview_guidance.compute_velocity_command,
            base_speed_mps=self.speed_var.get(),
            dt_s=self.cfg.dt,
        )
        ordered_allocs = sorted(assignments.values(), key=lambda alloc: alloc.slot_index)
        if not ordered_allocs:
            return

        role_by_uid = {uav.uid: uav.role for uav in active_uavs}
        min_x = min(alloc.target_x for alloc in ordered_allocs)
        max_x = max(alloc.target_x for alloc in ordered_allocs)
        min_y = min(alloc.target_y for alloc in ordered_allocs)
        max_y = max(alloc.target_y for alloc in ordered_allocs)

        plot_left = 18.0
        plot_right = width - 18.0
        plot_top = 34.0
        metrics_top = height - 45.0
        plot_bottom = metrics_top - 8.0
        plot_w = max(plot_right - plot_left, 1.0)
        plot_h = max(plot_bottom - plot_top, 1.0)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min(plot_w / span_x, plot_h / span_y) * 0.82
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        screen_cx = (plot_left + plot_right) / 2.0
        screen_cy = (plot_top + plot_bottom) / 2.0

        points: list[tuple[float, float, AllocatedSlot]] = []
        for alloc in ordered_allocs:
            sx = screen_cx + (alloc.target_x - center_x) * scale
            sy = screen_cy - (alloc.target_y - center_y) * scale
            points.append((sx, sy, alloc))

        if shape_type == "wedge":
            self.draw_wedge_preview_lines(canvas, points)
        else:
            for idx in range(1, len(points)):
                x1, y1, _ = points[idx - 1]
                x2, y2, _ = points[idx]
                canvas.create_line(x1, y1, x2, y2, fill="#cbd5e1", width=2)

        radius = 8
        for sx, sy, alloc in points:
            role = role_by_uid.get(alloc.uid, "")
            fill = role_color(role)
            canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill=fill, outline="#ffffff", width=2)
            canvas.create_text(sx, sy, text=str(alloc.slot_index), fill="#ffffff", font=("Arial", 8, "bold"))

        canvas.create_text(
            12,
            height - 32,
            text=f"배터리 평균 소모량 : {evaluation.average_battery_drain_pct:.2f}%",
            fill="#475569",
            anchor="w",
            font=("Arial", 12),
        )
        transition_text = f"{evaluation.transition_time_s:.1f}s"
        if not evaluation.converged:
            transition_text = f">{evaluation.transition_time_s:.0f}s"
        canvas.create_text(
            12,
            height - 14,
            text=f"편대 재편성 전환 소요 시간: {transition_text}",
            fill="#475569",
            anchor="w",
            font=("Arial", 12),
        )

    def draw_wedge_preview_lines(
        self,
        canvas: tk.Canvas,
        points: list[tuple[float, float, AllocatedSlot]],
    ) -> None:
        points_by_slot = {alloc.slot_index: (sx, sy) for sx, sy, alloc in points}
        if 1 not in points_by_slot:
            return

        left_branch = [1] + sorted(slot for slot in points_by_slot if slot > 1 and slot % 2 == 0)
        right_branch = [1] + sorted(slot for slot in points_by_slot if slot > 1 and slot % 2 == 1)
        for branch in (left_branch, right_branch):
            for start_slot, end_slot in zip(branch, branch[1:]):
                x1, y1 = points_by_slot[start_slot]
                x2, y2 = points_by_slot[end_slot]
                canvas.create_line(x1, y1, x2, y2, fill="#cbd5e1", width=2)

    def show_formation_shape_selector(self) -> None:
        if self.shape_selector_visible:
            return

        self.shape_selector_visible = True
        self.canvas.configure(height=self.compact_canvas_height)
        self.preview_canvas.configure(height=self.compact_canvas_height)
        self.shape_selector_frame.grid()
        self.root.update_idletasks()
        self.refresh_formation_shape_previews()

    def hide_formation_shape_selector(self) -> None:
        if not self.shape_selector_visible:
            return

        self.shape_selector_visible = False
        self.shape_selector_frame.grid_remove()
        self.canvas.configure(height=self.expanded_canvas_height)
        self.preview_canvas.configure(height=self.expanded_canvas_height)

    def select_formation_shape(self, shape_type: str) -> None:
        for uav in self.uavs:
            if not self.is_killed_uav(uav):
                uav.formation_id = 1
        self.failure_reconfiguration_applied = True
        self.apply_formation_shape(shape_type)
        self.hide_formation_shape_selector()

    def build_role_weight_controls(self) -> None:
        self.role_weight_frame = ttk.Frame(self.battery_frame)
        self.role_weight_frame.grid(row=4, column=0, pady=(14, 0), sticky="new")
        self.role_weight_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(self.role_weight_frame, text="Role Weight", font=("Arial", 12, "bold")).grid(
            row=0,
            column=0,
            sticky="w",
        )
        role_weight_button_frame = ttk.Frame(self.role_weight_frame)
        role_weight_button_frame.grid(row=0, column=1, columnspan=2, sticky="e")
        self.confirm_role_weight_button = ttk.Button(
            role_weight_button_frame,
            text="확인",
            command=self.confirm_role_weights,
        )
        self.confirm_role_weight_button.grid(row=0, column=0, padx=(0, 4), sticky="e")
        self.reset_role_weight_button = ttk.Button(
            role_weight_button_frame,
            text="초기화",
            command=self.reset_role_weights,
        )
        self.reset_role_weight_button.grid(row=0, column=1, sticky="e")

        self.role_weight_vars: dict[str, tk.DoubleVar] = {}
        self.role_weight_value_labels: dict[str, ttk.Label] = {}
        self.role_weight_scales: dict[str, ttk.Scale] = {}
        current_weights = self.priority_evaluator.get_role_priority_weights()
        for row_idx, role in enumerate(ROLE_PRIORITY_WEIGHT, start=1):
            value = current_weights.get(role, ROLE_PRIORITY_WEIGHT[role])
            self.role_weight_vars[role] = tk.DoubleVar(value=value)

            role_label = tk.Label(
                self.role_weight_frame,
                text=role.title(),
                width=8,
                anchor="center",
                bg=role_color(role),
                fg="#ffffff",
            )
            role_label.grid(row=row_idx, column=0, padx=(0, 8), pady=3, sticky="ew")

            scale = ttk.Scale(
                self.role_weight_frame,
                from_=0.0,
                to=1.0,
                variable=self.role_weight_vars[role],
                command=lambda raw_value, selected_role=role: self.update_role_weight(selected_role, raw_value),
            )
            scale.bind(
                "<Button-1>",
                lambda event, selected_role=role, selected_scale=scale: self.set_role_weight_from_pointer(
                    selected_role,
                    selected_scale,
                    event,
                ),
            )
            scale.bind(
                "<B1-Motion>",
                lambda event, selected_role=role, selected_scale=scale: self.set_role_weight_from_pointer(
                    selected_role,
                    selected_scale,
                    event,
                ),
            )
            scale.grid(row=row_idx, column=1, pady=3, sticky="ew")
            self.role_weight_scales[role] = scale

            value_label = ttk.Label(self.role_weight_frame, text=f"{value:.1f}", width=4, anchor="e")
            value_label.grid(row=row_idx, column=2, padx=(8, 0), pady=3, sticky="e")
            self.role_weight_value_labels[role] = value_label

    def set_role_weight_from_pointer(self, role: str, scale: ttk.Scale, event: tk.Event) -> str:
        width = max(scale.winfo_width(), 1)
        ratio = max(0.0, min(1.0, event.x / width))
        self.update_role_weight(role, str(ratio))
        return "break"

    def update_role_weight(self, role: str, raw_value: str) -> None:
        stepped_value = round(float(raw_value) * 10.0) / 10.0
        stepped_value = max(0.0, min(1.0, stepped_value))

        current_value = self.role_weight_vars[role].get()
        if abs(current_value - stepped_value) > 1e-9:
            self.role_weight_vars[role].set(stepped_value)

        self.role_weight_value_labels[role].configure(text=f"{stepped_value:.1f}")
        self.priority_evaluator.set_role_priority_weight(role, stepped_value)

    def confirm_role_weights(self) -> None:
        self.confirmed_role_weights = self.priority_evaluator.get_role_priority_weights()
        self.locked_priority_rows = self.build_priority_rows()
        self.priority_locked = True
        self.set_priority_text(self.locked_priority_rows)
        self.set_role_weight_controls_enabled(False)
        self.show_formation_shape_selector()

    def reset_role_weights(self) -> None:
        self.hide_formation_shape_selector()
        self.priority_locked = False
        self.locked_priority_rows = []
        self.set_role_weight_controls_enabled(True)

        for role, default_weight in ROLE_PRIORITY_WEIGHT.items():
            self.update_role_weight(role, str(default_weight))
        self.confirmed_role_weights = self.priority_evaluator.get_role_priority_weights()
        self.update_priority_panel()

    def set_role_weight_controls_enabled(self, enabled: bool) -> None:
        for scale in self.role_weight_scales.values():
            if enabled:
                scale.state(["!disabled"])
            else:
                scale.state(["disabled"])
        self.confirm_role_weight_button.configure(state="normal" if enabled else "disabled")

    def apply_formation_shape(self, shape_type: str | None = None) -> None:
        if shape_type:
            self.current_shape_type = shape_type
        else:
            shape_type = self.current_shape_type

        if not shape_type:
            self.assignments = {}
            return

        # 통합 매니저를 통해 알고리즘 수행
        active_uavs = [uav for uav in self.uavs if not self.is_killed_uav(uav)]
        self.assignments = update_formation_assignments(
            active_uavs,
            shape_type,
            spacing_m=10.0,
            role_weights=self.confirmed_role_weights,
            current_speed=self.speed_var.get(),
        )

        for uid, alloc in self.assignments.items():
            # 시뮬레이터에서 기체 순간이동 반영
            uav = next((u for u in self.uavs if u.uid == uid), None)
            if uav:
                pass  # 순간이동(Teleport)을 제거하고 advance_simulation에서 Guidance로 이동 처리

    def update_assignment_reference_for_speed(self, speed: float) -> None:
        if not self.assignments:
            return

        assignments_by_form: dict[int, list[AllocatedSlot]] = {}
        for alloc in self.assignments.values():
            assignments_by_form.setdefault(alloc.form_id, []).append(alloc)

        updated_assignments: dict[str, AllocatedSlot] = {}
        for allocs in assignments_by_form.values():
            center_x = sum(alloc.target_x for alloc in allocs) / len(allocs)

            if speed >= 30.0:
                slot_1 = next((alloc for alloc in allocs if alloc.slot_index == 1), allocs[0])
                center_y = slot_1.target_y
            elif speed <= 10.0:
                center_y = min(alloc.target_y for alloc in allocs)
            else:
                center_y = sum(alloc.target_y for alloc in allocs) / len(allocs)

            for alloc in allocs:
                updated_assignments[alloc.uid] = replace(
                    alloc,
                    dx=alloc.target_x - center_x,
                    dy=alloc.target_y - center_y,
                )

        self.assignments = updated_assignments

    def reset(self) -> None:
        self.finish_output_session()
        self.uavs = build_initial_uavs(self.speed_var.get())
        self.display_formation_ids = {uav.uid: uav.formation_id for uav in self.uavs}
        self.kill_event_model.reset()
        self.recent_kill_events.clear()
        self.t_s = 0.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = self.follow_var.get()
        self.assignments = {}
        self.current_shape_type = None
        self.failure_reconfiguration_applied = False
        self.hide_formation_shape_selector()
        self.priority_locked = False
        self.locked_priority_rows = []
        self.set_role_weight_controls_enabled(True)
        self.initialize_battery_table()
        self.start_output_session()

    def toggle_running(self) -> None:
        self.running = not self.running
        self.start_button.configure(text="Pause" if self.running else "Start")

    def update_speed(self, _: str) -> None:
        speed = self.speed_var.get()
        for uav in self.uavs:
            uav.speed_mps = speed
        self.update_assignment_reference_for_speed(speed)

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

        # 2. 타겟(슬롯)을 전방으로 이동 (계산 오차 방지를 위해 기체 속도 계산 후 이동시킵니다)
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

        # 3. 계산된 속도를 바탕으로 실제 기체의 위치와 기수(Heading) 업데이트
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

    def resize_preview_canvas(self, event: tk.Event) -> None:
        self.preview_canvas_w = max(event.width, 1)
        self.preview_canvas_h = max(event.height, 1)

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
        self.preview_canvas.delete("all")
        self.draw_grid()
        self.draw_preview_grid()
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
            form_label = ttk.Label(
                self.battery_table,
                text=f"F{self.display_formation_ids.get(uav.uid, uav.formation_id)}",
                width=6,
                anchor="center",
            )
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
            form_label.configure(text=f"F{self.display_formation_ids.get(uav.uid, uav.formation_id)}")
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
        if self.priority_locked:
            self.set_priority_text(self.locked_priority_rows)
            return

        self.set_priority_text(self.build_priority_rows())

    def build_priority_rows(self) -> list[tuple[str, str]]:
        candidates = self.priority_evaluator.rank_candidates(self.state_bus.operational_states(self.t_s))
        if not candidates:
            return [("Waiting for telemetry", "muted")]

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

        return rows

    def set_priority_text(self, rows: list[tuple[str, str]]) -> None:
        self.priority_text.configure(state="normal")
        self.priority_text.delete("1.0", tk.END)
        for row_idx, (text, tag) in enumerate(rows):
            line_end = "\n" if row_idx < len(rows) - 1 else ""
            self.priority_text.insert(tk.END, f"{text}{line_end}", tag)
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

    def preview_world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        cx = self.preview_canvas_w * 0.48
        cy = self.preview_canvas_h * 0.52
        sx = cx + (x_m - self.camera_x_m) * self.scale_px_per_m
        sy = cy - (y_m - self.camera_y_m) * self.scale_px_per_m
        return sx, sy

    def draw_preview_grid(self) -> None:
        grid_m = 10.0
        left_m = self.camera_x_m - self.preview_canvas_w / self.scale_px_per_m / 2.0
        right_m = self.camera_x_m + self.preview_canvas_w / self.scale_px_per_m / 2.0
        bottom_m = self.camera_y_m - self.preview_canvas_h / self.scale_px_per_m / 2.0
        top_m = self.camera_y_m + self.preview_canvas_h / self.scale_px_per_m / 2.0

        x = math.floor(left_m / grid_m) * grid_m
        while x <= right_m:
            sx, _ = self.preview_world_to_screen(x, 0.0)
            color = "#cbd5e1" if abs(x) < 1e-6 else "#e2e8f0"
            self.preview_canvas.create_line(sx, 0, sx, self.preview_canvas_h, fill=color)
            self.preview_canvas.create_text(
                sx + 3,
                self.preview_canvas_h - 18,
                text=f"{x:.0f}",
                fill="#64748b",
                anchor="w",
                font=("Arial", 9),
            )
            x += grid_m

        y = math.floor(bottom_m / grid_m) * grid_m
        while y <= top_m:
            _, sy = self.preview_world_to_screen(0.0, y)
            color = "#cbd5e1" if abs(y) < 1e-6 else "#e2e8f0"
            self.preview_canvas.create_line(0, sy, self.preview_canvas_w, sy, fill=color)
            self.preview_canvas.create_text(8, sy - 3, text=f"{y:.0f}", fill="#64748b", anchor="w", font=("Arial", 9))
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
            f"t = {self.t_s:5.1f} sec    speed = {self.speed_var.get():4.1f} m/s"
        )


def main() -> None:
    root = tk.Tk()
    RealtimeTkViewer(root)
    root.mainloop()
