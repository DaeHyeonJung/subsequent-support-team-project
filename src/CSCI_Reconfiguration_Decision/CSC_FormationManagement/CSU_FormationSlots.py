from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class SlotOffset:
    """
    편대 중심 또는 리더를 기준(0,0)으로 한 상대적인 슬롯 좌표(m)를 정의합니다.
    """
    slot_index: int  # 슬롯 번호 (1번이 보통 최전방/리더)
    dx_m: float      # 좌우 상대 거리 (오른쪽이 양수)
    dy_m: float      # 전후 상대 거리 (앞쪽이 양수)


def generate_wedge_shape(num_uavs: int, spacing_m: float = 10.0) -> list[SlotOffset]:
    """
    V자(Wedge) 형태의 편대 슬롯을 생성합니다.
    """
    slots = []
    for i in range(num_uavs):
        if i == 0:
            slots.append(SlotOffset(slot_index=i+1, dx_m=0.0, dy_m=0.0))
        else:
            # 짝수는 우측, 홀수는 좌측으로 배치하며 뒤로 물러납니다.
            side = -1 if i % 2 != 0 else 1
            row = (i + 1) // 2
            slots.append(SlotOffset(slot_index=i+1, dx_m=side * row * spacing_m, dy_m=-row * spacing_m))
    return slots


def generate_line_shape(num_uavs: int, spacing_m: float = 10.0) -> list[SlotOffset]:
    """
    횡렬(Line) 형태의 편대 슬롯을 생성합니다. 리더를 중심으로 좌우로 나란히 배치됩니다.
    """
    slots = []
    for i in range(num_uavs):
        if i == 0:
            slots.append(SlotOffset(slot_index=i+1, dx_m=0.0, dy_m=0.0))
        else:
            side = -1 if i % 2 != 0 else 1
            col = (i + 1) // 2
            slots.append(SlotOffset(slot_index=i+1, dx_m=side * col * spacing_m, dy_m=0.0))
    return slots


def generate_column_shape(num_uavs: int, spacing_m: float = 10.0) -> list[SlotOffset]:
    """
    종렬(Column) 형태의 편대 슬롯을 생성합니다. 리더 뒤로 일렬로 배치됩니다.
    """
    slots = []
    for i in range(num_uavs):
        slots.append(SlotOffset(slot_index=i+1, dx_m=0.0, dy_m=-i * spacing_m))
    return slots


def generate_staggered_column_shape(num_uavs: int, spacing_m: float = 10.0) -> list[SlotOffset]:
    """
    지그재그 종렬(Staggered Column) 형태의 편대 슬롯을 생성합니다.
    """
    slots = []
    for i in range(num_uavs):
        if i == 0:
            slots.append(SlotOffset(slot_index=i+1, dx_m=0.0, dy_m=0.0))
        else:
            side = -1 if i % 2 != 0 else 1
            dx = side * (spacing_m * 0.5)
            dy = -i * spacing_m
            slots.append(SlotOffset(slot_index=i+1, dx_m=dx, dy_m=dy))
    return slots


def get_formation_slots(shape_type: str, num_uavs: int, spacing_m: float = 10.0) -> list[SlotOffset]:
    """
    요청된 형상(shape_type)에 맞는 슬롯 배치를 반환하는 팩토리 함수입니다.
    지원 형상: wedge, line, column, staggered_column
    """
    shape = shape_type.lower().replace(" ", "_")
    if shape == "wedge":
        return generate_wedge_shape(num_uavs, spacing_m)
    elif shape == "line":
        return generate_line_shape(num_uavs, spacing_m)
    elif shape == "column":
        return generate_column_shape(num_uavs, spacing_m)
    elif shape == "staggered_column":
        return generate_staggered_column_shape(num_uavs, spacing_m)
        
    return generate_wedge_shape(num_uavs, spacing_m)