from __future__ import annotations

from html import escape
from pathlib import Path

from src.CSCI_Simulation_Engine.CSC_Battery.CSU_BatteryConfig import BatteryDrainConfig
from src.CSCI_Simulation_Engine.CSC_Battery.CSU_BatteryModel import BatteryModel


def write_lipo_standard_curve_svg(
    path: Path,
    config: BatteryDrainConfig | None = None,
    sample_count: int = 401,
) -> None:
    """Write an SVG graph of the configured LiHV standard discharge curve."""

    model = BatteryModel(config)
    path.parent.mkdir(parents=True, exist_ok=True)

    width = 1000
    height = 620
    margin_left = 86
    margin_right = 34
    margin_top = 46
    margin_bottom = 82
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    x_min = 0.0
    x_max = 100.0
    y_min = 3.0
    y_max = 4.35

    def sx(x_value: float) -> float:
        return margin_left + (x_value - x_min) / (x_max - x_min) * plot_w

    def sy(y_value: float) -> float:
        return margin_top + (y_max - y_value) / (y_max - y_min) * plot_h

    samples = []
    for idx in range(sample_count):
        discharge_percent = x_min + (x_max - x_min) * idx / max(sample_count - 1, 1)
        voltage = model.cell_voltage_at_discharge_percent(discharge_percent)
        samples.append((discharge_percent, voltage))

    curve_points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in samples)
    anchors = sorted(model.config.voltage_curve_points, key=lambda point: point[0])

    parts: list[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        _text(width / 2, 28, "LiHV 1C LCO Standard Discharge Curve", 22, "#111827", "middle", "bold"),
        _text(width / 2, 54, "WebPlotDigitizer anchor points with linear interpolation", 13, "#475569", "middle"),
    ]

    for tick in range(0, 101, 10):
        x = sx(float(tick))
        grid_color = "#cbd5e1" if tick in {0, 100} else "#e2e8f0"
        parts.append(_line(x, margin_top, x, margin_top + plot_h, grid_color, 1))
        parts.append(_text(x, height - 50, str(tick), 12, "#475569", "middle"))

    y_ticks = [3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2, 4.35]
    for tick in y_ticks:
        y = sy(tick)
        grid_color = "#cbd5e1" if tick in {3.0, 4.35} else "#e2e8f0"
        parts.append(_line(margin_left, y, margin_left + plot_w, y, grid_color, 1))
        parts.append(_text(margin_left - 12, y + 4, f"{tick:.2f}", 12, "#475569", "end"))

    parts.extend(
        [
            _line(margin_left, margin_top, margin_left, margin_top + plot_h, "#0f172a", 1.5),
            _line(margin_left, margin_top + plot_h, margin_left + plot_w, margin_top + plot_h, "#0f172a", 1.5),
            f'<polyline points="{curve_points}" fill="none" stroke="#2563eb" stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round"/>',
        ]
    )

    for discharge_percent, voltage in anchors:
        parts.append(
            f'<circle cx="{sx(discharge_percent):.2f}" cy="{sy(voltage):.2f}" r="4.2" fill="#ffffff" stroke="#dc2626" stroke-width="2"/>'
        )

    parts.extend(
        [
            _text(width / 2, height - 16, "Discharge progress (%)", 15, "#111827", "middle", "bold"),
            f'<g transform="translate(22 {height / 2}) rotate(-90)">'
            + _text(0, 0, "Cell voltage (V)", 15, "#111827", "middle", "bold")
            + "</g>",
            _legend(margin_left + plot_w - 230, margin_top + 18),
            "</svg>",
        ]
    )

    path.write_text("\n".join(parts), encoding="utf-8")


def _line(x1: float, y1: float, x2: float, y2: float, color: str, width: float) -> str:
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{width}"/>'


def _text(
    x: float,
    y: float,
    text: str,
    size: int,
    color: str,
    anchor: str,
    weight: str = "normal",
) -> str:
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" '
        f'font-family="Arial, sans-serif" font-size="{size}" font-weight="{weight}" fill="{color}">'
        f"{escape(text)}</text>"
    )


def _legend(x: float, y: float) -> str:
    return "\n".join(
        [
            f'<rect x="{x:.2f}" y="{y:.2f}" width="204" height="58" fill="#ffffff" stroke="#cbd5e1" rx="6"/>',
            f'<line x1="{x + 16:.2f}" y1="{y + 21:.2f}" x2="{x + 58:.2f}" y2="{y + 21:.2f}" stroke="#2563eb" stroke-width="3.2"/>',
            _text(x + 68, y + 25, "Interpolated curve", 12, "#334155", "start"),
            f'<circle cx="{x + 37:.2f}" cy="{y + 43:.2f}" r="4.2" fill="#ffffff" stroke="#dc2626" stroke-width="2"/>',
            _text(x + 68, y + 47, "Anchor points", 12, "#334155", "start"),
        ]
    )
