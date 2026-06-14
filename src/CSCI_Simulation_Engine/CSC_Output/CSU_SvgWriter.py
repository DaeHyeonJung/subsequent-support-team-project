from __future__ import annotations

from pathlib import Path

from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import Role, UavState


def role_color(role: Role) -> str:
    return {
        "recon": "#2563eb",
        "strike": "#dc2626",
        "decoy": "#16a34a",
    }.get(role, "#111827")


def write_svg(uavs: list[UavState], path: Path) -> None:
    points = [(x, y) for uav in uavs for _, x, y, _, _, _, _, _, _, _, _, _ in uav.history]
    if not points:
        return

    min_x = min(x for x, _ in points)
    max_x = max(x for x, _ in points)
    min_y = min(y for _, y in points)
    max_y = max(y for _, y in points)

    width = 1200
    height = 700
    margin = 70
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def sx(x_m: float) -> float:
        return margin + (x_m - min_x) * scale

    def sy(y_m: float) -> float:
        return height - margin - (y_m - min_y) * scale

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="70" y="38" font-family="Arial" font-size="22" fill="#0f172a">Formation Flight 2D Trajectories</text>',
        '<text x="70" y="62" font-family="Arial" font-size="13" fill="#475569">blue: recon, red: strike, green: decoy</text>',
        f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" stroke="#94a3b8" stroke-width="1"/>',
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height - margin}" stroke="#94a3b8" stroke-width="1"/>',
    ]

    for uav in uavs:
        if not uav.history:
            continue
        color = role_color(uav.role)
        coords = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for _, x, y, _, _, _, _, _, _, _, _, _ in uav.history)
        start = uav.history[0]
        end = uav.history[-1]
        lines.extend(
            [
                f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2" opacity="0.82"/>',
                f'<circle cx="{sx(start[1]):.2f}" cy="{sy(start[2]):.2f}" r="4" fill="#ffffff" stroke="{color}" stroke-width="2"/>',
                f'<circle cx="{sx(end[1]):.2f}" cy="{sy(end[2]):.2f}" r="5" fill="{color}"/>',
                f'<text x="{sx(end[1]) + 7:.2f}" y="{sy(end[2]) - 7:.2f}" font-family="Arial" font-size="12" fill="#0f172a">{uav.uid}</text>',
            ]
        )

    lines.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
