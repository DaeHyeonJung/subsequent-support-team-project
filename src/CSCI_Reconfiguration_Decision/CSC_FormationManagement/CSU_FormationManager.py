from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState
from .CSU_FormationSlots import get_formation_slots
from .CSU_SlotAllocation import allocate_slots_with_constraints

@dataclass(frozen=True)
class AllocatedSlot:
    uid: str
    form_id: int
    slot_index: int
    target_x: float
    target_y: float
    dx: float
    dy: float

@dataclass(frozen=True)
class RenderLine:
    x1: float
    y1: float
    x2: float
    y2: float

@dataclass(frozen=True)
class RenderSlot:
    x: float
    y: float
    label: str

@dataclass(frozen=True)
class RenderCenter:
    x: float
    y: float
    label: str

@dataclass(frozen=True)
class FormationRenderData:
    lines: List[RenderLine]
    slots: List[RenderSlot]
    centers: List[RenderCenter]

def update_formation_assignments(uavs: List[UavState], shape_type: str, spacing_m: float = 10.0) -> Dict[str, AllocatedSlot]:
    """
    UAV 상태와 원하는 편대 형상을 받아, 각 기체의 절대 목적지와 슬롯 정보를 매핑해 반환합니다.
    시뮬레이션 UI와 알고리즘의 완전한 분리를 위한 통합 인터페이스 모듈입니다.
    """
    formations: dict[int, list[UavState]] = {}
    for uav in uavs:
        formations.setdefault(uav.formation_id, []).append(uav)

    assignments = {}

    for form_id, uav_list in formations.items():
        if not uav_list:
            continue

        center_x = sum(u.x_m for u in uav_list) / len(uav_list)
        center_y = sum(u.y_m for u in uav_list) / len(uav_list)

        slots = get_formation_slots(shape_type, len(uav_list), spacing_m)

        mean_dx = sum(s.dx_m for s in slots) / len(slots)
        mean_dy = sum(s.dy_m for s in slots) / len(slots)

        allocation_map = allocate_slots_with_constraints(
            uav_list, slots, center_x, center_y, mean_dx, mean_dy
        )

        for uav in uav_list:
            slot = allocation_map[uav.uid]
            adjusted_dx = slot.dx_m - mean_dx
            adjusted_dy = slot.dy_m - mean_dy

            assignments[uav.uid] = AllocatedSlot(
                uid=uav.uid,
                form_id=form_id,
                slot_index=slot.slot_index,
                target_x=center_x + adjusted_dx,
                target_y=center_y + adjusted_dy,
                dx=adjusted_dx,
                dy=adjusted_dy
            )

    return assignments

def calculate_formation_render_data(uavs: List[UavState], assignments: Dict[str, AllocatedSlot]) -> FormationRenderData:
    """
    현재 UAV들의 위치(무게중심)와 할당된 슬롯 오프셋을 바탕으로,
    화면에 그려야 할 편대 뼈대(점선), 슬롯 마커, 중심점의 절대 좌표를 계산하여 반환합니다.
    UI와 알고리즘을 분리하기 위한 시각화 데이터 생성 함수입니다.
    """
    assigned_uids = set(assignments)
    centers: dict[int, tuple[float, float]] = {}
    formations: dict[int, list[UavState]] = {}
    for uav in uavs:
        if uav.uid in assigned_uids:
            formations.setdefault(uav.formation_id, []).append(uav)

    for form_id, uav_list in formations.items():
        if not uav_list:
            continue
        cx = sum(u.x_m for u in uav_list) / len(uav_list)
        cy = sum(u.y_m for u in uav_list) / len(uav_list)
        centers[form_id] = (cx, cy)

    form_slots_world: dict[int, list[tuple[float, float, int]]] = {}
    slots_out = []

    # 슬롯 중복 방지 (기체 기준이 아닌 형상 슬롯 기준으로 수집)
    unique_slots = {}
    for alloc in assignments.values():
        unique_slots[(alloc.form_id, alloc.slot_index)] = alloc

    for alloc in unique_slots.values():
        form_id = alloc.form_id
        tx = alloc.target_x
        ty = alloc.target_y

        slots_out.append(RenderSlot(x=tx, y=ty, label=f"s{form_id}-{alloc.slot_index}"))
        form_slots_world.setdefault(form_id, []).append((tx, ty, alloc.slot_index))

    lines_out = []
    for form_id, pts in form_slots_world.items():
        pts.sort(key=lambda x: x[2])
        for i in range(1, len(pts)):
            p1 = pts[i]
            closest_p = None
            min_dist = float('inf')
            for j in range(i):
                p2 = pts[j]
                dist = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
                if dist < min_dist:
                    min_dist = dist
                    closest_p = p2
            if closest_p:
                lines_out.append(RenderLine(x1=p1[0], y1=p1[1], x2=closest_p[0], y2=closest_p[1]))

    centers_out = []
    for form_id, (cx, cy) in centers.items():
        centers_out.append(RenderCenter(x=cx, y=cy, label=f"F{form_id}"))

    return FormationRenderData(lines=lines_out, slots=slots_out, centers=centers_out)
