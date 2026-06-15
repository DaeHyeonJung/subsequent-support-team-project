from __future__ import annotations

import csv
import math
import tkinter as tk
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from tkinter import ttk

from src.CSCI_Guidance_Control.CSC_CollisionAvoidance import PotentialField3DAvoidance
from src.CSCI_Guidance_Control.CSC_Controller.CSU_HeadingController import HeadingController
from src.CSCI_Guidance_Control.CSC_Controller.CSU_LateralSlotLQRController import (
    LateralSlotLQRController,
)
from src.CSCI_Guidance_Control.CSC_Controller.CSU_VerticalLQRController import VerticalLQRController
from src.CSCI_Guidance_Control.CSC_Guidance.CSU_BasicLOS3DGuidance import BasicLOS3DGuidance
from src.CSCI_Guidance_Control.CSC_Guidance.CSU_SlotReferenceGenerator import SlotReferenceGenerator
from src.CSCI_Reconfiguration_Decision.CSC_RolePriority import CandidatePriorityEvaluator, ROLE_PRIORITY_WEIGHT
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import (
    update_formation_assignments,
    calculate_formation_render_data,
    AllocatedSlot,
)
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_ReconfigurationEvaluator import (
    evaluate_reconfiguration_plan_3d,
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
from src.CSCI_Simulation_Engine.CSC_Dynamics.CSU_PointMassPseudoDynamics import step_uav
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
        self.heading_controller = HeadingController(
            heading_gain=0.65,
            max_yaw_rate_rad_s=math.radians(22.0),
        )
        self.basic_los_3d_guidance = BasicLOS3DGuidance(arrival_radius_m=4.0)
        self.collision_avoidance = PotentialField3DAvoidance()
        self.lateral_slot_lqr_controller = LateralSlotLQRController()
        self.slot_reference_generator = SlotReferenceGenerator()
        self.vertical_lqr_controller = VerticalLQRController()
        self.slot_along_velocity_gain = 0.35
        self.max_slot_along_velocity_correction_mps = 12.0
        self.max_slot_speed_boost_mps = 12.0
        self.max_slot_speed_reduction_mps = 8.0
        self.formation_slot_spacing_m = 15.0

        self.cfg = SimConfig(
            dt=0.05,
            duration=10_000.0,
        )

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
        self.preview_perspective_scale_px_per_m = 4.2
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
        self.preview_canvas_h = int(self.expanded_canvas_height * 0.7)
        self.side_canvas_w = 460
        self.side_canvas_h = self.expanded_canvas_height - self.preview_canvas_h
        self.view_stack_frame = ttk.Frame(root)
        self.view_stack_frame.grid(row=1, column=4, columnspan=4, padx=(3, 6), pady=(0, 0), sticky="nsew")
        self.view_stack_frame.grid_rowconfigure(0, weight=7)
        self.view_stack_frame.grid_rowconfigure(1, weight=3)
        self.view_stack_frame.grid_columnconfigure(0, weight=1)

        self.preview_frame = ttk.LabelFrame(self.view_stack_frame, text="3D ISO VIEW", padding=(2, 2))
        self.preview_frame.grid(row=0, column=0, padx=0, pady=(0, 3), sticky="nsew")
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

        self.side_frame = ttk.LabelFrame(self.view_stack_frame, text="3D SIDE VIEW", padding=(2, 2))
        self.side_frame.grid(row=1, column=0, padx=0, pady=(3, 0), sticky="nsew")
        self.side_frame.grid_rowconfigure(0, weight=1)
        self.side_frame.grid_columnconfigure(0, weight=1)
        self.side_canvas = tk.Canvas(
            self.side_frame,
            width=self.side_canvas_w,
            height=self.side_canvas_h,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        self.side_canvas.grid(row=0, column=0, sticky="nsew")
        self.side_canvas.bind("<Configure>", self.resize_side_canvas)

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

        ttk.Label(root, text="Cruise Speed").grid(row=0, column=5, padx=6, pady=8, sticky="e")
        self.speed_var = tk.DoubleVar(value=self.cfg.speed_mps)
        self.speed_label = ttk.Label(
            root,
            text=f"{self.cfg.cruise_speed_mps:.1f} m/s",
            width=10,
        )
        self.speed_label.grid(row=0, column=6, columnspan=3, padx=6, pady=8, sticky="ew")

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
            spacing_m=self.formation_slot_spacing_m,
            role_weights=self.confirmed_role_weights,
            current_speed=self.speed_var.get(),
        )
        evaluation = evaluate_reconfiguration_plan_3d(
            active_uavs,
            assignments,
            self.battery_model,
            self.compute_preview_virtual_structure_tracking_command,
            self.heading_controller,
            self.collision_avoidance,
            self.cfg,
            base_speed_mps=self.speed_var.get(),
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
        preview_height = int(self.compact_canvas_height * 0.7)
        self.preview_canvas.configure(height=preview_height)
        self.side_canvas.configure(height=self.compact_canvas_height - preview_height)
        self.shape_selector_frame.grid()
        self.root.update_idletasks()
        self.refresh_formation_shape_previews()

    def hide_formation_shape_selector(self) -> None:
        if not self.shape_selector_visible:
            return

        self.shape_selector_visible = False
        self.shape_selector_frame.grid_remove()
        self.canvas.configure(height=self.expanded_canvas_height)
        preview_height = int(self.expanded_canvas_height * 0.7)
        self.preview_canvas.configure(height=preview_height)
        self.side_canvas.configure(height=self.expanded_canvas_height - preview_height)

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
            self.slot_reference_generator.reset()
            return

        # 통합 매니저를 통해 알고리즘 수행
        active_uavs = [uav for uav in self.uavs if not self.is_killed_uav(uav)]
        self.slot_reference_generator.reset()
        self.assignments = update_formation_assignments(
            active_uavs,
            shape_type,
            spacing_m=self.formation_slot_spacing_m,
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
        self.vertical_lqr_controller.reset()
        self.lateral_slot_lqr_controller.reset()
        self.slot_reference_generator.reset()
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
        if self.shape_selector_visible:
            self.refresh_formation_shape_previews()

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

        for uav in self.uavs:
            uav.record(self.t_s)
            if self.is_killed_uav(uav):
                continue

            desired_heading_rad = math.radians(90.0)
            desired_flight_path_rad = 0.0
            speed_cmd_mps = current_formation_speed
            roll_cmd_rad: float | None = None

            if uav.uid in self.assignments:
                alloc = self.assignments[uav.uid]
                avoidance_vector = self.collision_avoidance.compute_avoidance_vector(uav, self.uavs)
                roll_cmd_rad, desired_flight_path_rad, speed_cmd_mps = (
                    self.compute_virtual_structure_tracking_command(
                        uav=uav,
                        target_x_m=alloc.target_x,
                        target_y_m=alloc.target_y,
                        target_z_m=80.0,
                        cruise_speed_mps=current_formation_speed,
                        avoidance_x_m=avoidance_vector.x_m,
                        avoidance_y_m=avoidance_vector.y_m,
                        avoidance_z_m=avoidance_vector.z_m,
                    )
                )

            if roll_cmd_rad is None:
                roll_cmd_rad = self.heading_controller.compute_roll_command(
                    current_heading_rad=uav.heading_rad,
                    desired_heading_rad=desired_heading_rad,
                    speed_mps=uav.speed_mps,
                )
            step_uav(
                uav,
                roll_cmd_rad=roll_cmd_rad,
                cfg=self.cfg,
                speed_cmd_mps=speed_cmd_mps,
                flight_path_cmd_rad=desired_flight_path_rad,
            )

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

        self.recent_kill_events = self.kill_event_model.apply_due_events(self.t_s, self.uavs)
        snapshot = build_snapshot(self.t_s, self.uavs)
        self.state_bus.update_from_simulation_snapshot(snapshot)
        self.simulation_port.publish(snapshot)
        self.write_snapshot_csv(snapshot)
        self.t_s += self.cfg.dt

    def compute_virtual_structure_tracking_command(
        self,
        uav: UavState,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        cruise_speed_mps: float,
        avoidance_x_m: float = 0.0,
        avoidance_y_m: float = 0.0,
        avoidance_z_m: float = 0.0,
        update_vertical_lqr_state: bool = True,
        update_lateral_lqr_state: bool = True,
        update_slot_reference_state: bool = True,
    ) -> tuple[float, float, float]:
        commanded_altitude_m = target_z_m + avoidance_z_m
        los_result = self.basic_los_3d_guidance.compute_desired_command(
            current_x_m=uav.x_m,
            current_y_m=uav.y_m,
            current_z_m=uav.z_m,
            target_x_m=target_x_m + avoidance_x_m,
            target_y_m=target_y_m + avoidance_y_m,
            target_z_m=commanded_altitude_m,
            fallback_heading_rad=uav.heading_rad,
            fallback_flight_path_rad=uav.flight_path_rad,
        )

        cross_track_error_m = target_x_m - uav.x_m + avoidance_x_m
        along_track_error_m = target_y_m - uav.y_m + avoidance_y_m

        if update_slot_reference_state:
            smoothed_target_x_m = self.slot_reference_generator.update_lateral_reference(
                uid=uav.uid,
                current_x_m=uav.x_m,
                final_target_x_m=target_x_m,
                dt_s=self.cfg.dt,
                update_state=True,
            )
        else:
            smoothed_target_x_m = target_x_m
        commanded_lateral_position_m = smoothed_target_x_m + avoidance_x_m
        roll_cmd_rad = self.lateral_slot_lqr_controller.compute_roll_command(
            lateral_position_m=uav.x_m,
            commanded_lateral_position_m=commanded_lateral_position_m,
            speed_mps=uav.speed_mps,
            heading_rad=uav.heading_rad,
            roll_rad=uav.roll_rad,
            roll_rate_rad_s=uav.roll_rate_rad_s,
            gravity_mps2=self.cfg.gravity_mps2,
            roll_pd_kp=self.cfg.roll_pd_kp,
            roll_pd_kd=self.cfg.roll_pd_kd,
            dt_s=self.cfg.dt,
            command_id=uav.uid,
            update_state=update_lateral_lqr_state,
            apply_rate_limit=update_lateral_lqr_state,
        )

        desired_flight_path_rad = self.vertical_lqr_controller.compute_flight_path_command(
            altitude_m=uav.z_m,
            commanded_altitude_m=commanded_altitude_m,
            speed_mps=uav.speed_mps,
            flight_path_rad=uav.flight_path_rad,
            flight_path_rate_rad_s=uav.flight_path_rate_rad_s,
            flight_path_kp=self.cfg.flight_path_kp,
            flight_path_kd=self.cfg.flight_path_kd,
            dt_s=self.cfg.dt,
            command_id=uav.uid,
            update_state=update_vertical_lqr_state,
            apply_rate_limit=update_vertical_lqr_state,
        )

        along_velocity_correction_mps = self.slot_along_velocity_gain * along_track_error_m
        along_velocity_correction_mps = max(
            -self.max_slot_along_velocity_correction_mps,
            min(along_velocity_correction_mps, self.max_slot_along_velocity_correction_mps),
        )
        desired_y_velocity_mps = cruise_speed_mps + along_velocity_correction_mps
        forward_projection = max(math.sin(uav.heading_rad), 0.25)
        speed_cmd_mps = desired_y_velocity_mps / forward_projection

        min_slot_speed_mps = max(self.cfg.min_speed_mps, cruise_speed_mps - self.max_slot_speed_reduction_mps)
        max_slot_speed_mps = min(self.cfg.max_speed_mps, cruise_speed_mps + self.max_slot_speed_boost_mps)
        speed_cmd_mps = max(min_slot_speed_mps, min(speed_cmd_mps, max_slot_speed_mps))

        return roll_cmd_rad, desired_flight_path_rad, speed_cmd_mps

    def compute_preview_virtual_structure_tracking_command(
        self,
        uav: UavState,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        cruise_speed_mps: float,
        avoidance_x_m: float = 0.0,
        avoidance_y_m: float = 0.0,
        avoidance_z_m: float = 0.0,
    ) -> tuple[float, float, float]:
        return self.compute_virtual_structure_tracking_command(
            uav=uav,
            target_x_m=target_x_m,
            target_y_m=target_y_m,
            target_z_m=target_z_m,
            cruise_speed_mps=cruise_speed_mps,
            avoidance_x_m=avoidance_x_m,
            avoidance_y_m=avoidance_y_m,
            avoidance_z_m=avoidance_z_m,
            update_vertical_lqr_state=False,
            update_lateral_lqr_state=False,
            update_slot_reference_state=False,
        )

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
                    f"{uav.z_m:.3f}",
                    f"{uav.speed_mps:.3f}",
                    f"{math.degrees(uav.heading_rad):.3f}",
                    f"{math.degrees(uav.flight_path_rad):.3f}",
                    f"{math.degrees(uav.roll_rad):.3f}",
                    f"{math.degrees(uav.roll_rate_rad_s):.3f}",
                    f"{math.degrees(uav.flight_path_rate_rad_s):.3f}",
                    f"{uav.longitudinal_accel_mps2:.3f}",
                    f"{uav.vertical_accel_mps2:.3f}",
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

    def resize_side_canvas(self, event: tk.Event) -> None:
        self.side_canvas_w = max(event.width, 1)
        self.side_canvas_h = max(event.height, 1)

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
        self.side_canvas.delete("all")
        self.draw_grid()
        self.draw_3d_preview_view()
        self.draw_3d_side_view()
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

    def preview_world_to_perspective(self, x_m: float, y_m: float, z_m: float) -> tuple[float, float]:
        origin_x = self.preview_canvas_w * 0.52
        origin_y = self.preview_canvas_h * 0.68
        dx = x_m - self.camera_x_m
        dy = y_m - self.camera_y_m
        dz = z_m - 80.0
        scale = self.preview_perspective_scale_px_per_m
        sx = origin_x + (dx - dy) * 0.62 * scale
        sy = origin_y + (dx + dy) * 0.23 * scale - dz * 2.6
        return sx, sy

    def point_in_preview_view(self, sx: float, sy: float) -> bool:
        return 0.0 <= sx <= self.preview_canvas_w and 0.0 <= sy <= self.preview_canvas_h

    def draw_3d_preview_line(self, sx0: float, sy0: float, sx1: float, sy1: float, **kwargs: object) -> None:
        if self.point_in_preview_view(sx0, sy0) and self.point_in_preview_view(sx1, sy1):
            self.preview_canvas.create_line(sx0, sy0, sx1, sy1, **kwargs)

    def draw_3d_preview_view(self) -> None:
        self.preview_canvas.create_rectangle(
            0,
            0,
            self.preview_canvas_w,
            self.preview_canvas_h,
            fill="#f8fafc",
            outline="#94a3b8",
        )
        self.draw_3d_preview_grid()

        for alloc in self.assignments.values():
            tx, ty = self.preview_world_to_perspective(alloc.target_x, alloc.target_y, 80.0)
            if not self.point_in_preview_view(tx, ty):
                continue
            self.preview_canvas.create_line(tx - 9, ty, tx + 9, ty, fill="#7c3aed", width=2)
            self.preview_canvas.create_line(tx, ty - 9, tx, ty + 9, fill="#7c3aed", width=2)
            self.preview_canvas.create_text(
                tx + 8,
                ty - 9,
                text=f"s{alloc.form_id}-{alloc.slot_index}",
                fill="#4c1d95",
                anchor="w",
                font=("Arial", 8, "bold"),
            )

        for uav in sorted(self.uavs, key=lambda item: item.y_m):
            color = "#111827" if self.is_killed_uav(uav) else role_color(uav.role)
            self.draw_3d_preview_tail(uav, color)
            sx, sy = self.preview_world_to_perspective(uav.x_m, uav.y_m, uav.z_m)
            gx, gy = self.preview_world_to_perspective(uav.x_m, uav.y_m, 0.0)
            self.draw_3d_preview_line(sx, sy, gx, gy, fill="#cbd5e1", width=1)
            if self.is_killed_uav(uav):
                self.draw_3d_preview_killed_marker(uav)
            else:
                self.draw_3d_preview_uav_symbol(uav, color)

    def draw_3d_preview_grid(self) -> None:
        step = 30.0
        grid_min_x, grid_max_x, grid_min_y, grid_max_y = self.compute_3d_preview_grid_bounds(step)
        self.draw_3d_preview_floor_grid(grid_min_x, grid_max_x, grid_min_y, grid_max_y, step)
        self.draw_3d_preview_wall_grid(grid_min_x, grid_max_x, grid_min_y, grid_max_y, step)
        self.draw_3d_preview_floor_ticks(grid_min_x, grid_max_x, grid_min_y, grid_max_y, step)
        self.draw_3d_preview_altitude_ticks(grid_max_x, grid_min_y)

    def compute_3d_preview_grid_bounds(self, step: float) -> tuple[float, float, float, float]:
        x_values = [self.camera_x_m - 120.0, self.camera_x_m + 120.0]
        y_values = [self.camera_y_m - 80.0, self.camera_y_m + 220.0]

        for uav in self.uavs:
            x_values.append(uav.x_m)
            y_values.append(uav.y_m)
        for alloc in self.assignments.values():
            x_values.append(alloc.target_x)
            y_values.append(alloc.target_y)

        margin_m = 60.0
        grid_min_x = math.floor((min(x_values) - margin_m) / step) * step
        grid_max_x = math.ceil((max(x_values) + margin_m) / step) * step
        grid_min_y = math.floor((min(y_values) - margin_m) / step) * step
        grid_max_y = math.ceil((max(y_values) + margin_m) / step) * step
        return grid_min_x, grid_max_x, grid_min_y, grid_max_y

    def draw_3d_preview_floor_grid(
        self,
        grid_min_x: float,
        grid_max_x: float,
        grid_min_y: float,
        grid_max_y: float,
        step: float,
    ) -> None:
        value = grid_min_x
        while value <= grid_max_x:
            color = "#cbd5e1" if abs(value % (step * 2.0)) < 1e-6 else "#e2e8f0"
            sx0, sy0 = self.preview_world_to_perspective(value, grid_min_y, 0.0)
            sx1, sy1 = self.preview_world_to_perspective(value, grid_max_y, 0.0)
            self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill=color, width=1)
            value += step

        value = grid_min_y
        while value <= grid_max_y:
            color = "#cbd5e1" if abs(value % (step * 2.0)) < 1e-6 else "#e2e8f0"
            sx0, sy0 = self.preview_world_to_perspective(grid_min_x, value, 0.0)
            sx1, sy1 = self.preview_world_to_perspective(grid_max_x, value, 0.0)
            self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill=color, width=1)
            value += step

        corners = [
            self.preview_world_to_perspective(grid_min_x, grid_min_y, 0.0),
            self.preview_world_to_perspective(grid_max_x, grid_min_y, 0.0),
            self.preview_world_to_perspective(grid_max_x, grid_max_y, 0.0),
            self.preview_world_to_perspective(grid_min_x, grid_max_y, 0.0),
        ]
        for idx in range(len(corners)):
            x0, y0 = corners[idx]
            x1, y1 = corners[(idx + 1) % len(corners)]
            self.draw_3d_preview_line(x0, y0, x1, y1, fill="#94a3b8", width=1)

    def draw_3d_preview_wall_grid(
        self,
        grid_min_x: float,
        grid_max_x: float,
        grid_min_y: float,
        grid_max_y: float,
        step: float,
    ) -> None:
        altitude_values = range(0, 141, 20)
        value = grid_min_x
        while value <= grid_max_x:
            for altitude_m in altitude_values:
                sx0, sy0 = self.preview_world_to_perspective(value, grid_min_y, float(altitude_m))
                sx1, sy1 = self.preview_world_to_perspective(value + step, grid_min_y, float(altitude_m))
                self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill="#edf2f7")
            sx0, sy0 = self.preview_world_to_perspective(value, grid_min_y, 0.0)
            sx1, sy1 = self.preview_world_to_perspective(value, grid_min_y, 140.0)
            self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill="#edf2f7")
            value += step

        value = grid_min_y
        while value <= grid_max_y:
            for altitude_m in altitude_values:
                sx0, sy0 = self.preview_world_to_perspective(grid_min_x, value, float(altitude_m))
                sx1, sy1 = self.preview_world_to_perspective(grid_min_x, value + step, float(altitude_m))
                self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill="#edf2f7")
            sx0, sy0 = self.preview_world_to_perspective(grid_min_x, value, 0.0)
            sx1, sy1 = self.preview_world_to_perspective(grid_min_x, value, 140.0)
            self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill="#edf2f7")
            value += step

    def draw_3d_preview_floor_ticks(
        self,
        grid_min_x: float,
        grid_max_x: float,
        grid_min_y: float,
        grid_max_y: float,
        step: float,
    ) -> None:
        tick = grid_min_x
        while tick <= grid_max_x:
            sx, sy = self.preview_world_to_perspective(tick, grid_min_y, 0.0)
            if self.point_in_preview_view(sx, sy):
                self.preview_canvas.create_text(sx, sy + 14, text=f"{tick:.0f}", fill="#94a3b8", anchor="n", font=("Arial", 8))
            tick += step

        tick = grid_min_y
        while tick <= grid_max_y:
            sx, sy = self.preview_world_to_perspective(grid_min_x, tick, 0.0)
            if self.point_in_preview_view(sx, sy):
                self.preview_canvas.create_text(sx - 6, sy, text=f"{tick:.0f}", fill="#94a3b8", anchor="e", font=("Arial", 8))
            tick += step

    def draw_3d_preview_altitude_ticks(self, base_x: float, base_y: float) -> None:
        for altitude_m in range(0, 141, 20):
            sx, sy = self.preview_world_to_perspective(base_x, base_y, float(altitude_m))
            if not self.point_in_preview_view(sx, sy):
                continue
            self.preview_canvas.create_line(sx - 5, sy, sx + 5, sy, fill="#cbd5e1", width=1)
            self.preview_canvas.create_text(sx + 8, sy, text=f"{altitude_m}", fill="#94a3b8", anchor="w", font=("Arial", 8))
        sx0, sy0 = self.preview_world_to_perspective(base_x, base_y, 0.0)
        sx1, sy1 = self.preview_world_to_perspective(base_x, base_y, 140.0)
        self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill="#cbd5e1", width=1)

    def draw_3d_preview_tail(self, uav: UavState, color: str) -> None:
        cutoff_t = self.t_s - self.tail_seconds
        recent = [sample for sample in uav.history if sample[0] >= cutoff_t]
        for idx in range(max(0, len(recent) - 1)):
            t0, x0, y0, z0, _, _, _, _, _, _, _, _ = recent[idx]
            _, x1, y1, z1, _, _, _, _, _, _, _, _ = recent[idx + 1]
            sx0, sy0 = self.preview_world_to_perspective(x0, y0, z0)
            sx1, sy1 = self.preview_world_to_perspective(x1, y1, z1)
            age_ratio = (t0 - cutoff_t) / max(self.tail_seconds, 0.1)
            width = 1 + 2 * age_ratio
            self.draw_3d_preview_line(sx0, sy0, sx1, sy1, fill=color, width=width)

    def draw_3d_preview_uav_symbol(self, uav: UavState, color: str) -> None:
        body_len_m = 9.0
        wing_span_m = 7.2
        local_points = [
            (body_len_m, 0.0),
            (-body_len_m * 0.65, -wing_span_m * 0.55),
            (-body_len_m * 0.35, 0.0),
            (-body_len_m * 0.65, wing_span_m * 0.55),
        ]
        world_points = []
        for px, py in local_points:
            world_x = uav.x_m + px * math.cos(uav.heading_rad) - py * math.sin(uav.heading_rad)
            world_y = uav.y_m + px * math.sin(uav.heading_rad) + py * math.cos(uav.heading_rad)
            world_points.append((world_x, world_y, uav.z_m))

        body_points = [self.preview_world_to_perspective(*point) for point in world_points]
        if all(self.point_in_preview_view(px, py) for px, py in body_points):
            flat = [coord for point in body_points for coord in point]
            self.preview_canvas.create_polygon(flat, fill="#0f172a", outline=color, width=2)
            cx, cy = self.preview_world_to_perspective(uav.x_m, uav.y_m, uav.z_m)
            self.preview_canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=color, outline="#ffffff", width=1)

    def draw_3d_preview_killed_marker(self, uav: UavState) -> None:
        sx, sy = self.preview_world_to_perspective(uav.x_m, uav.y_m, uav.z_m)
        if not self.point_in_preview_view(sx, sy):
            return
        radius = 10
        self.preview_canvas.create_line(sx - radius, sy - radius, sx + radius, sy + radius, fill="#111827", width=2)
        self.preview_canvas.create_line(sx - radius, sy + radius, sx + radius, sy - radius, fill="#111827", width=2)
        self.preview_canvas.create_oval(sx - 13, sy - 13, sx + 13, sy + 13, outline="#dc2626", width=2)

    def compute_side_view_bounds(self) -> tuple[float, float, float, float]:
        y_min = self.camera_y_m - 35.0
        y_max = self.camera_y_m + 85.0
        z_values = [80.0]

        for uav in self.uavs:
            z_values.append(uav.z_m)
        for alloc in self.assignments.values():
            z_values.append(80.0)

        z_min_raw = min(z_values)
        z_max_raw = max(z_values)
        z_span = max(z_max_raw - z_min_raw, 28.0)
        z_center = (z_min_raw + z_max_raw) * 0.5
        z_margin = max(z_span * 0.12, 4.0)
        z_min = max(0.0, z_center - z_span * 0.5 - z_margin)
        z_max = z_center + z_span * 0.5 + z_margin
        return y_min, y_max, z_min, z_max

    def side_world_to_screen(
        self,
        y_m: float,
        z_m: float,
        bounds: tuple[float, float, float, float],
    ) -> tuple[float, float]:
        y_min, y_max, z_min, z_max = bounds
        left_pad = 38.0
        right_pad = 14.0
        top_pad = 16.0
        bottom_pad = 32.0
        plot_w = max(self.side_canvas_w - left_pad - right_pad, 1.0)
        plot_h = max(self.side_canvas_h - top_pad - bottom_pad, 1.0)
        sx = left_pad + (y_m - y_min) / max(y_max - y_min, 1.0) * plot_w
        sy = self.side_canvas_h - bottom_pad - (z_m - z_min) / max(z_max - z_min, 1.0) * plot_h
        return sx, sy

    def draw_3d_side_view(self) -> None:
        self.side_canvas.create_rectangle(
            0,
            0,
            self.side_canvas_w,
            self.side_canvas_h,
            fill="#f8fafc",
            outline="#94a3b8",
        )
        bounds = self.compute_side_view_bounds()
        y_min, y_max, z_min, z_max = bounds
        left_pad = 38.0
        right_pad = 14.0
        top_pad = 16.0
        bottom_pad = 32.0
        x0 = left_pad
        y0 = top_pad
        x1 = self.side_canvas_w - right_pad
        y1 = self.side_canvas_h - bottom_pad

        self.side_canvas.create_rectangle(x0, y0, x1, y1, outline="#cbd5e1", fill="")

        for idx in range(5):
            ratio = idx / 4.0
            tick_y = y_min + (y_max - y_min) * ratio
            sx, _ = self.side_world_to_screen(tick_y, z_min, bounds)
            self.side_canvas.create_line(sx, y0, sx, y1, fill="#e2e8f0")
            self.side_canvas.create_text(sx, y1 + 14, text=f"{tick_y:.0f}", fill="#94a3b8", font=("Arial", 8))

        for idx in range(5):
            ratio = idx / 4.0
            tick_z = z_min + (z_max - z_min) * ratio
            _, sy = self.side_world_to_screen(y_min, tick_z, bounds)
            self.side_canvas.create_line(x0, sy, x1, sy, fill="#e2e8f0")
            self.side_canvas.create_text(x0 - 6, sy, text=f"{tick_z:.0f}", fill="#94a3b8", anchor="e", font=("Arial", 8))

        if z_min <= 80.0 <= z_max:
            _, ref_y = self.side_world_to_screen(y_min, 80.0, bounds)
            self.side_canvas.create_line(x0, ref_y, x1, ref_y, fill="#7c3aed", dash=(4, 3), width=1)
            self.side_canvas.create_text(x1 - 4, ref_y - 4, text="80 m", fill="#4c1d95", anchor="se", font=("Arial", 8, "bold"))

        for alloc in self.assignments.values():
            sx, sy = self.side_world_to_screen(alloc.target_y, 80.0, bounds)
            self.side_canvas.create_line(sx - 7, sy, sx + 7, sy, fill="#7c3aed", width=2)
            self.side_canvas.create_line(sx, sy - 7, sx, sy + 7, fill="#7c3aed", width=2)

        for uav in sorted(self.uavs, key=lambda item: item.y_m):
            color = "#111827" if self.is_killed_uav(uav) else role_color(uav.role)
            cutoff_t = self.t_s - self.tail_seconds
            recent = [sample for sample in uav.history if sample[0] >= cutoff_t]
            for idx in range(max(0, len(recent) - 1)):
                t0, _, y_start, z_start, _, _, _, _, _, _, _, _ = recent[idx]
                _, _, y_end, z_end, _, _, _, _, _, _, _, _ = recent[idx + 1]
                sx0, sy0 = self.side_world_to_screen(y_start, z_start, bounds)
                sx1, sy1 = self.side_world_to_screen(y_end, z_end, bounds)
                age_ratio = (t0 - cutoff_t) / max(self.tail_seconds, 0.1)
                width = 1 + 2 * age_ratio
                self.side_canvas.create_line(sx0, sy0, sx1, sy1, fill=color, width=width)

            sx, sy = self.side_world_to_screen(uav.y_m, uav.z_m, bounds)
            if self.is_killed_uav(uav):
                self.side_canvas.create_line(sx - 7, sy - 7, sx + 7, sy + 7, fill="#111827", width=2)
                self.side_canvas.create_line(sx - 7, sy + 7, sx + 7, sy - 7, fill="#111827", width=2)
                self.side_canvas.create_oval(sx - 10, sy - 10, sx + 10, sy + 10, outline="#dc2626", width=2)
            else:
                self.side_canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=color, outline="#ffffff", width=1)
                self.side_canvas.create_line(sx, sy, sx + 12, sy, fill=color, width=2)

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
            t0, x0, y0, _, _, _, _, _, _, _, _, _ = recent[idx]
            _, x1, y1, _, _, _, _, _, _, _, _, _ = recent[idx + 1]
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
