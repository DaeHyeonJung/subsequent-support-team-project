import re
import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, List

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState
from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import AllocatedSlot

def role_color(role: str) -> str:
    return {
        "recon": "#2563eb",
        "strike": "#dc2626",
        "decoy": "#16a34a",
    }.get(role, "#111827")

# 기체의 최초 표시 이름을 기억하기 위한 캐시 딕셔너리입니다.
# 편대 재편성 시 formation_id가 변경되어도 기존 이름을 유지하게 해줍니다.
_UID_DISPLAY_CACHE: Dict[str, str] = {}

def format_uav_uid(uid: str, form_id: int) -> str:
    """기체의 UID에서 숫자만 추출해 일관된 F1-U1 형식으로 출력하도록 보정합니다."""
    if uid in _UID_DISPLAY_CACHE:
        return _UID_DISPLAY_CACHE[uid]

    nums = re.findall(r'\d+', uid)
    if not nums:
        result = f"F{form_id}-U1"
    else:
        val = int(nums[-1])
        # 초기 기체 이름이 UAV_0 형태이거나 0번일 경우, 1부터 시작하도록 보정
        if "uav" in uid.lower() or val == 0:
            val += 1
        result = f"F{form_id}-U{val}"
        
    _UID_DISPLAY_CACHE[uid] = result
    return result

def resolve_uid(input_name: str) -> str:
    """입력된 표시 이름(F1-U1 등)을 검색하여 실제 기체의 UID(uav-0 등)로 반환합니다."""
    for uid, display_name in _UID_DISPLAY_CACHE.items():
        if display_name == input_name:
            return uid
    return input_name

class FormationPanel:
    """
    시뮬레이션 우측에 표시되는 상태 표(UAV/Slot)와 편대 제어 UI를 전담하는 모듈입니다.
    TkViewer에서 복잡한 테이블 갱신 코드를 분리하기 위해 설계되었습니다.
    """
    def __init__(self, parent: tk.Widget, on_shape_change: Callable[[str], None]):
        self.parent = parent
        self.on_shape_change = on_shape_change
        
        self.frame = ttk.Frame(parent, padding=(12, 10))
        self.frame.grid_rowconfigure(0, weight=1)
        
        # UAV Status Frame
        self.battery_frame = ttk.Frame(self.frame)
        self.battery_frame.grid(row=0, column=0, sticky="nsew")
        ttk.Label(self.battery_frame, text="UAV Status", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w")
        self.battery_table = ttk.Frame(self.battery_frame)
        self.battery_table.grid(row=1, column=0, pady=(8, 0), sticky="nsew")
        self.battery_rows: dict[str, tuple[ttk.Label, ttk.Label, ttk.Label, tk.Label, ttk.Label]] = {}
        
        # Slot Status Frame
        self.slot_frame = ttk.Frame(self.frame)
        self.slot_frame.grid(row=1, column=0, pady=(20, 0), sticky="nsew")
        ttk.Label(self.slot_frame, text="Slot Status", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w")
        self.slot_table = ttk.Frame(self.slot_frame)
        self.slot_table.grid(row=1, column=0, pady=(8, 0), sticky="nsew")
        self.slot_rows: dict[str, tuple[ttk.Label, ttk.Label, ttk.Label, ttk.Label]] = {}
        
        # Formation Shape Frame
        self.formation_frame = ttk.Frame(self.frame)
        self.formation_frame.grid(row=2, column=0, pady=(20, 0), sticky="nsew")
        ttk.Label(self.formation_frame, text="Formation Shape", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w")
        
        self.shape_var = tk.StringVar(value="wedge")
        for i, shape in enumerate(["wedge", "line", "column", "staggered_column"]):
            ttk.Radiobutton(
                self.formation_frame, 
                text=shape.replace("_", " ").title(),
                variable=self.shape_var, 
                value=shape,
                command=self._on_radio_change
            ).grid(row=i+1, column=0, sticky="w", pady=2)
            
        self._build_battery_table_header()
        self._build_slot_table_header()

    def _on_radio_change(self) -> None:
        self.on_shape_change(self.shape_var.get())

    def get_current_shape(self) -> str:
        return self.shape_var.get()

    def _build_battery_table_header(self) -> None:
        headers = [("UAV", 7), ("Form", 6), ("Slot", 6), ("Role", 8), ("Battery", 9)]
        for col_idx, (text, width) in enumerate(headers):
            label = ttk.Label(self.battery_table, text=text, width=width, anchor="center", font=("Arial", 9, "bold"))
            label.grid(row=0, column=col_idx, padx=4, pady=(0, 4), sticky="ew")

    def _build_slot_table_header(self) -> None:
        headers = [("Slot", 6), ("dx(m)", 6), ("dy(m)", 6), ("UAV", 7)]
        for col_idx, (text, width) in enumerate(headers):
            label = ttk.Label(self.slot_table, text=text, width=width, anchor="center", font=("Arial", 9, "bold"))
            label.grid(row=0, column=col_idx, padx=2, pady=(0, 4), sticky="ew")

    def initialize_battery_table(self, uavs: List[UavState], assignments: Dict[str, AllocatedSlot]) -> None:
        for widget in self.battery_table.winfo_children():
            widget.destroy()
        self.battery_rows.clear()
        self._build_battery_table_header()
        
        for uav in uavs:
            row_idx = len(self.battery_rows) + 1
            
            uid_label = ttk.Label(self.battery_table, text=format_uav_uid(uav.uid, uav.formation_id), width=7, anchor="center")
            form_label = ttk.Label(self.battery_table, text=f"F{uav.formation_id}", width=6, anchor="center")
            slot_idx = assignments[uav.uid].slot_index if uav.uid in assignments else "-"
            slot_str = f"s{uav.formation_id}-{slot_idx}" if slot_idx != "-" else "-"
            slot_label = ttk.Label(self.battery_table, text=slot_str, width=6, anchor="center")
            role_label = tk.Label(self.battery_table, text=uav.role, width=8, anchor="center", bg=role_color(uav.role), fg="#ffffff")
            battery_label = ttk.Label(self.battery_table, text=f"{uav.battery_pct:5.1f}%", width=9, anchor="e")

            uid_label.grid(row=row_idx, column=0, padx=(0, 4), pady=2, sticky="ew")
            form_label.grid(row=row_idx, column=1, padx=4, pady=2, sticky="ew")
            slot_label.grid(row=row_idx, column=2, padx=4, pady=2, sticky="ew")
            role_label.grid(row=row_idx, column=3, padx=4, pady=2, sticky="ew")
            battery_label.grid(row=row_idx, column=4, padx=(4, 0), pady=2, sticky="ew")
            self.battery_rows[uav.uid] = (uid_label, form_label, slot_label, role_label, battery_label)

    def update_battery_table(self, uavs: List[UavState], assignments: Dict[str, AllocatedSlot]) -> None:
        for uav in uavs:
            if uav.uid not in self.battery_rows:
                self.initialize_battery_table(uavs, assignments)
                return
            uid_label, form_label, slot_label, role_label, battery_label = self.battery_rows[uav.uid]
            
            uid_label.configure(text=format_uav_uid(uav.uid, uav.formation_id))
            form_label.configure(text=f"F{uav.formation_id}")
            slot_idx = assignments[uav.uid].slot_index if uav.uid in assignments else "-"
            slot_label.configure(text=f"s{uav.formation_id}-{slot_idx}" if slot_idx != "-" else "-")
            role_label.configure(text=uav.role, bg=role_color(uav.role))
            battery_label.configure(text=f"{uav.battery_pct:5.1f}%")

    def refresh_slot_table(self, assignments: Dict[str, AllocatedSlot]) -> None:
        for widget in self.slot_table.winfo_children():
            widget.destroy()
        self.slot_rows.clear()
        self._build_slot_table_header()
        for idx, alloc in enumerate(assignments.values()):
            row_idx = idx + 1
            slot_str = f"s{alloc.form_id}-{alloc.slot_index}"
            slot_label = ttk.Label(self.slot_table, text=slot_str, width=6, anchor="center")
            dx_label = ttk.Label(self.slot_table, text=f"{alloc.dx:.1f}", width=6, anchor="center")
            dy_label = ttk.Label(self.slot_table, text=f"{alloc.dy:.1f}", width=6, anchor="center")
            
            uid_label = ttk.Label(self.slot_table, text=format_uav_uid(alloc.uid, alloc.form_id), width=7, anchor="center")

            slot_label.grid(row=row_idx, column=0, padx=2, pady=2, sticky="ew")
            dx_label.grid(row=row_idx, column=1, padx=2, pady=2, sticky="ew")
            dy_label.grid(row=row_idx, column=2, padx=2, pady=2, sticky="ew")
            uid_label.grid(row=row_idx, column=3, padx=2, pady=2, sticky="ew")
            self.slot_rows[slot_str] = (slot_label, dx_label, dy_label, uid_label)