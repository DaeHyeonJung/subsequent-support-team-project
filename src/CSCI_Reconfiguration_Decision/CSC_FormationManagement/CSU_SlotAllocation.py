from __future__ import annotations
import numpy as np
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState
from .CSU_FormationSlots import SlotOffset
from .CSU_FairHungarian import fair_hungarian_assignment


def allocate_slots_with_constraints(
    uavs: list[UavState], 
    slots: list[SlotOffset],
    center_x: float,
    center_y: float,
    mean_dx: float,
    mean_dy: float
) -> dict[str, SlotOffset]:
    """
    기체들을 슬롯에 배정합니다.
    
    :return: { "UAV uid": SlotOffset } 형태의 최종 할당 딕셔너리
    """
    allocation_map: dict[str, SlotOffset] = {}
    
    # 1단계: 전체 기체들 할당 (Fair 헝가리안 알고리즘 적용)
    if uavs and slots:
        N, M = len(uavs), len(slots)
        cost_matrix = np.zeros((N, M))
        
        for i, uav in enumerate(uavs):
            for j, slot in enumerate(slots):
                target_x = center_x + slot.dx_m - mean_dx
                target_y = center_y + slot.dy_m - mean_dy
                cost_matrix[i, j] = np.hypot(uav.x_m - target_x, uav.y_m - target_y)
                
        matching = fair_hungarian_assignment(cost_matrix)
        
        if matching is not None:
            for i, slot_idx in enumerate(matching):
                if slot_idx >= 0:
                    allocation_map[uavs[i].uid] = slots[slot_idx]
                    
    # 2단계: Fallback (만약 매칭되지 않은 기체가 있다면 남은 자리에 강제 배정)
    unallocated_uavs = [u for u in uavs if u.uid not in allocation_map]
    used_slots = set(allocation_map.values())
    unallocated_slots = [s for s in slots if s not in used_slots]
    
    for uav, slot in zip(unallocated_uavs, unallocated_slots):
        allocation_map[uav.uid] = slot
        
    return allocation_map