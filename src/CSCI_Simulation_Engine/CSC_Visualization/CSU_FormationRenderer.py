import tkinter as tk
from typing import Callable

from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FormationManager import FormationRenderData


def render_formation_to_canvas(
    canvas: tk.Canvas,
    render_data: FormationRenderData,
    world_to_screen: Callable[[float, float], tuple[float, float]]
) -> None:
    """
    계산된 시각화 데이터(FormationRenderData)를 전달받아 Tkinter Canvas에 편대 형상을 렌더링합니다.
    UI 그리기(Drawing) 역할만 전담하는 모듈입니다.
    """
    # 1. 편대 형상의 뼈대(모양) 점선 연결
    for line in render_data.lines:
        sx1, sy1 = world_to_screen(line.x1, line.y1)
        sx2, sy2 = world_to_screen(line.x2, line.y2)
        canvas.create_line(sx1, sy1, sx2, sy2, fill="#cbd5e1", dash=(4, 4), width=2)
        
    # 2. 각 슬롯 마커와 텍스트 그리기
    for slot in render_data.slots:
        sx, sy = world_to_screen(slot.x, slot.y)
        size = 4
        canvas.create_line(sx - size, sy, sx + size, sy, fill="#94a3b8", width=1)
        canvas.create_line(sx, sy - size, sx, sy + size, fill="#94a3b8", width=1)
        canvas.create_oval(sx - size, sy - size, sx + size, sy + size, outline="#94a3b8", width=1, fill="#f8fafc")
        canvas.create_text(sx + 6, sy - 6, text=slot.label, fill="#64748b", anchor="sw", font=("Arial", 8, "bold"))
        
    # 3. 가상구조의 중심점을 파란색 점으로 찍어주고 라벨링
    for center in render_data.centers:
        sx, sy = world_to_screen(center.x, center.y)
        canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill="#3b82f6", outline="#2563eb", width=1)
        canvas.create_text(sx, sy - 10, text=center.label, fill="#1d4ed8", font=("Arial", 10, "bold"))