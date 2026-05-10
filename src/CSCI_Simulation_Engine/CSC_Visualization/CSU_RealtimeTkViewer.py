from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from src.CSCI_Reconfiguration_Decision.CSC_StateBus.CSU_StateBus import StateBus
from src.CSCI_Simulation_Engine.CSC_Command.CSU_RollCommand import straight_roll_command
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Dynamics.CSU_PointMassPseudoDynamics import step_uav
from src.CSCI_Simulation_Engine.CSC_Interface.CSU_SimulationPort import (
    NullSimulationPort,
    SimulationPort,
    build_snapshot,
)
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import Role, UavState
from src.CSCI_Simulation_Engine.CSC_Scenario.CSU_InitialScenario import build_initial_uavs


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
        self.simulation_port = simulation_port or NullSimulationPort()
        self.state_bus = StateBus()

        self.cfg = SimConfig(dt=0.05, duration=10_000.0, speed_mps=15.0)
        self.uavs = build_initial_uavs(self.cfg.speed_mps)
        self.t_s = 0.0
        self.running = True
        self.scale_px_per_m = 6.2
        self.tail_seconds = 9.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = True
        self.drag_start: tuple[int, int] | None = None

        self.canvas_w = 1180
        self.canvas_h = 720
        self.canvas = tk.Canvas(root, width=self.canvas_w, height=self.canvas_h, bg="#f8fafc", highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=8, sticky="nsew")
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag_camera)
        self.canvas.bind("<MouseWheel>", self.zoom_with_wheel)
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)
        self.root.bind("<KeyPress>", self.handle_key)

        self.start_button = ttk.Button(root, text="Pause", command=self.toggle_running)
        self.start_button.grid(row=1, column=0, padx=6, pady=8, sticky="ew")

        self.reset_button = ttk.Button(root, text="Reset", command=self.reset)
        self.reset_button.grid(row=1, column=1, padx=6, pady=8, sticky="ew")

        self.follow_var = tk.BooleanVar(value=True)
        self.follow_check = ttk.Checkbutton(root, text="Follow", variable=self.follow_var, command=self.update_follow)
        self.follow_check.grid(row=1, column=2, padx=6, pady=8, sticky="ew")

        ttk.Label(root, text="UAV Speed").grid(row=1, column=3, padx=6, pady=8, sticky="e")
        self.speed_var = tk.DoubleVar(value=self.cfg.speed_mps)
        self.speed_scale = ttk.Scale(root, from_=5.0, to=35.0, variable=self.speed_var, command=self.update_speed)
        self.speed_scale.grid(row=1, column=4, padx=6, pady=8, sticky="ew")

        ttk.Label(root, text="Tail Length").grid(row=1, column=5, padx=6, pady=8, sticky="e")
        self.tail_var = tk.DoubleVar(value=self.tail_seconds)
        self.tail_scale = ttk.Scale(root, from_=2.0, to=20.0, variable=self.tail_var, command=self.update_tail)
        self.tail_scale.grid(row=1, column=6, padx=6, pady=8, sticky="ew")

        self.status_var = tk.StringVar(value="")
        ttk.Label(root, textvariable=self.status_var).grid(row=1, column=7, padx=10, pady=8, sticky="w")

        root.grid_columnconfigure(4, weight=1)
        root.grid_columnconfigure(6, weight=1)
        root.grid_rowconfigure(0, weight=1)

        self.tick()

    def reset(self) -> None:
        self.uavs = build_initial_uavs(self.speed_var.get())
        self.t_s = 0.0
        self.camera_x_m = 0.0
        self.camera_y_m = -25.0
        self.follow_camera = self.follow_var.get()

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
        if self.running:
            self.advance_simulation()
        self.draw()
        self.root.after(33, self.tick)

    def advance_simulation(self) -> None:
        for uav in self.uavs:
            uav.record(self.t_s)
            step_uav(uav, straight_roll_command(uav, self.t_s), self.cfg)
        snapshot = build_snapshot(self.t_s, self.uavs)
        self.state_bus.update_from_simulation_snapshot(snapshot)
        self.simulation_port.publish(snapshot)
        self.t_s += self.cfg.dt

    def world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        cx = self.canvas_w * 0.48
        cy = self.canvas_h * 0.52
        sx = cx + (x_m - self.camera_x_m) * self.scale_px_per_m
        sy = cy - (y_m - self.camera_y_m) * self.scale_px_per_m
        return sx, sy

    def update_camera(self) -> None:
        if not self.uavs or not self.follow_camera:
            return
        lead_x = sum(uav.x_m for uav in self.uavs) / len(self.uavs)
        lead_y = sum(uav.y_m for uav in self.uavs) / len(self.uavs)
        self.camera_x_m += (lead_x - self.camera_x_m) * 0.04
        self.camera_y_m += (lead_y - self.camera_y_m) * 0.04

    def draw(self) -> None:
        self.update_camera()
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_uavs()
        self.draw_hud()

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

    def draw_uavs(self) -> None:
        for uav in self.uavs:
            color = role_color(uav.role)
            self.draw_tail(uav, color)
            self.draw_uav_body(uav, color)

    def draw_tail(self, uav: UavState, color: str) -> None:
        cutoff_t = self.t_s - self.tail_seconds
        recent = [sample for sample in uav.history if sample[0] >= cutoff_t]
        if len(recent) < 2:
            return

        max_segments = len(recent) - 1
        for idx in range(max_segments):
            t0, x0, y0, _, _ = recent[idx]
            _, x1, y1, _, _ = recent[idx + 1]
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

    def draw_hud(self) -> None:
        self.canvas.create_text(16, 16, text="Dynamic 2D Plot", fill="#0f172a", anchor="nw", font=("Arial", 16, "bold"))
        self.canvas.create_text(16, 42, text="blue: recon   red: strike   green: decoy", fill="#475569", anchor="nw", font=("Arial", 10))
        self.canvas.create_text(
            16,
            62,
            text="drag: pan   mouse wheel: zoom   arrow/WASD: move view   space: pause",
            fill="#64748b",
            anchor="nw",
            font=("Arial", 10),
        )
        available_count = len(self.state_bus.available_uavs(self.t_s))
        total_count = len(self.state_bus.latest_telemetry())
        recon_count = sum(1 for message in self.state_bus.latest_telemetry() if message.role == "recon")
        self.canvas.create_text(
            16,
            82,
            text=f"StateBus: telemetry {total_count} UAVs   available {available_count} UAVs   recon {recon_count} UAVs",
            fill="#475569",
            anchor="nw",
            font=("Arial", 10),
        )
        self.status_var.set(
            f"t = {self.t_s:5.1f} sec    speed = {self.speed_var.get():4.1f} m/s    tail = {self.tail_seconds:4.1f} sec"
        )


def main() -> None:
    root = tk.Tk()
    RealtimeTkViewer(root)
    root.mainloop()
