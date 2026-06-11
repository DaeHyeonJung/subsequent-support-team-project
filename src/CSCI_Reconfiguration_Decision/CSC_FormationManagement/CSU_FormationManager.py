from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState
from .CSU_FormationSlots import get_formation_slots
from .CSU_SlotAllocation import allocate_slots_with_constraints, allocate_slots_by_role

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

def update_formation_assignments(
    uavs: List[UavState], 
    shape_type: str, 
    spacing_m: float = 10.0,
    role_weights: Dict[str, float] | None = None,
    current_speed: float = 15.0
) -> Dict[str, AllocatedSlot]:
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
            
        slots = get_formation_slots(shape_type, len(uav_list), spacing_m)
        
        # X축은 무조건 기체들의 중간(평균)을 사용하여 좌우 쏠림을 방지합니다.
        center_x = sum(u.x_m for u in uav_list) / len(uav_list)
        mean_dx = sum(s.dx_m for s in slots) / len(slots)
        
        # 속도 구간에 따른 Y축 적응형 기준점(Adaptive Reference Point) 설정
        if current_speed >= 30.0:
            # 30 이상 (고속): 1번 슬롯(최전방)의 Y를 리더 기체의 Y에 맞춤
            if role_weights is not None:
                ref_uav = sorted(uav_list, key=lambda u: (role_weights.get(u.role, 0.0), u.battery_pct), reverse=True)[0]
            else:
                ref_uav = sorted(uav_list, key=lambda u: u.battery_pct, reverse=True)[0]
            center_y = ref_uav.y_m
            
            # 1번 슬롯의 Y 오프셋을 빼주어, 리더 기체 Y 위치에 1번 슬롯을 완벽히 고정
            slot_1 = next((s for s in slots if s.slot_index == 1), slots[0])
            mean_dy = slot_1.dy_m
        elif current_speed <= 10.0:
            # 10 이하 (저속): 형상의 '최후방' Y를 제일 뒤에 있는 기체의 Y에 맞춤
            ref_uav = min(uav_list, key=lambda u: u.y_m)
            center_y = ref_uav.y_m
            mean_dy = min(s.dy_m for s in slots)
        else:
            # 10 초과 ~ 30 미만 (중속): Y축도 기체 전체의 평균(무게 중심) 기준
            center_y = sum(u.y_m for u in uav_list) / len(uav_list)
            mean_dy = sum(s.dy_m for s in slots) / len(slots)
        
        if role_weights is not None:
            allocation_map = allocate_slots_by_role(
                uav_list, slots, center_x, center_y, mean_dx, mean_dy, role_weights
            )
        else:
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
    centers: dict[int, tuple[float, float]] = {}
        
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
        
        # 타겟 절대 좌표에서 오프셋을 역으로 빼서 가상의 중심점(Virtual Center)을 산출
        if form_id not in centers:
            centers[form_id] = (tx - alloc.dx, ty - alloc.dy)
        
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