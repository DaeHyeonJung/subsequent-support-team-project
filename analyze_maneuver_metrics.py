from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path


def find_latest_trajectory() -> Path:
    candidates = sorted(Path("outputs").glob("realtime_*/trajectory.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError("No realtime trajectory.csv found under outputs/realtime_*")
    return candidates[-1]


def read_trajectory(path: Path) -> dict[str, list[dict[str, float]]]:
    series: dict[str, list[dict[str, float]]] = defaultdict(list)
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row.get("vehicle_health") == "KILLED":
                continue
            uid = row["uav_id"]
            series[uid].append(
                {
                    "time_s": float(row["time_s"]),
                    "z_m": float(row["z_m"]),
                    "speed_mps": float(row["speed_mps"]),
                    "flight_path_deg": float(row["flight_path_deg"]),
                    "roll_deg": float(row["roll_deg"]),
                    "longitudinal_accel_mps2": float(row["longitudinal_accel_mps2"]),
                    "vertical_accel_mps2": float(row["vertical_accel_mps2"]),
                }
            )
    return dict(sorted(series.items()))


def write_summary_csv(series: dict[str, list[dict[str, float]]], path: Path, nominal_altitude_m: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "uav_id",
                "min_altitude_m",
                "max_altitude_m",
                "max_altitude_error_m",
                "max_abs_flight_path_deg",
                "max_abs_roll_deg",
                "max_speed_mps",
                "max_abs_longitudinal_accel_mps2",
                "max_abs_vertical_accel_mps2",
            ],
        )
        writer.writeheader()
        for uid, rows in series.items():
            altitudes = [row["z_m"] for row in rows]
            flight_paths = [abs(row["flight_path_deg"]) for row in rows]
            rolls = [abs(row["roll_deg"]) for row in rows]
            speeds = [row["speed_mps"] for row in rows]
            lon_accels = [abs(row["longitudinal_accel_mps2"]) for row in rows]
            vert_accels = [abs(row["vertical_accel_mps2"]) for row in rows]
            writer.writerow(
                {
                    "uav_id": uid,
                    "min_altitude_m": min(altitudes),
                    "max_altitude_m": max(altitudes),
                    "max_altitude_error_m": max(abs(z - nominal_altitude_m) for z in altitudes),
                    "max_abs_flight_path_deg": max(flight_paths),
                    "max_abs_roll_deg": max(rolls),
                    "max_speed_mps": max(speeds),
                    "max_abs_longitudinal_accel_mps2": max(lon_accels),
                    "max_abs_vertical_accel_mps2": max(vert_accels),
                }
            )


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if abs(src_max - src_min) < 1e-9:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) / (src_max - src_min) * (dst_max - dst_min)


def polyline(points: list[tuple[float, float]], color: str, width: int = 2) -> str:
    coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}" opacity="0.92"/>'


def draw_panel(
    lines: list[str],
    series: dict[str, list[dict[str, float]]],
    key: str,
    title: str,
    unit: str,
    x0: int,
    y0: int,
    width: int,
    height: int,
    symmetric: bool = False,
    reference: float | None = None,
) -> None:
    margin_l = 55
    margin_r = 20
    margin_t = 36
    margin_b = 34
    plot_x0 = x0 + margin_l
    plot_y0 = y0 + margin_t
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b

    all_rows = [row for rows in series.values() for row in rows]
    t_min = min(row["time_s"] for row in all_rows)
    t_max = max(row["time_s"] for row in all_rows)
    values = [row[key] for row in all_rows]
    if reference is not None:
        values.append(reference)
    y_min = min(values)
    y_max = max(values)
    if symmetric:
        bound = max(abs(y_min), abs(y_max), 1.0)
        y_min, y_max = -bound, bound
    else:
        pad = max((y_max - y_min) * 0.12, 0.5)
        y_min -= pad
        y_max += pad

    def sx(t_s: float) -> float:
        return scale(t_s, t_min, t_max, plot_x0, plot_x0 + plot_w)

    def sy(value: float) -> float:
        return scale(value, y_min, y_max, plot_y0 + plot_h, plot_y0)

    lines.append(f'<rect x="{x0}" y="{y0}" width="{width}" height="{height}" rx="0" fill="#f8fafc" stroke="#cbd5e1"/>')
    lines.append(f'<text x="{x0 + 16}" y="{y0 + 24}" font-family="Arial" font-size="15" font-weight="bold" fill="#0f172a">{title}</text>')
    lines.append(f'<text x="{x0 + width - 20}" y="{y0 + 24}" font-family="Arial" font-size="11" fill="#64748b" text-anchor="end">{unit}</text>')

    for tick in range(0, 5):
        value = y_min + (y_max - y_min) * tick / 4.0
        y = sy(value)
        lines.append(f'<line x1="{plot_x0}" y1="{y:.2f}" x2="{plot_x0 + plot_w}" y2="{y:.2f}" stroke="#e2e8f0"/>')
        lines.append(f'<text x="{plot_x0 - 8}" y="{y + 4:.2f}" font-family="Arial" font-size="10" fill="#64748b" text-anchor="end">{value:.1f}</text>')

    lines.append(f'<line x1="{plot_x0}" y1="{plot_y0 + plot_h}" x2="{plot_x0 + plot_w}" y2="{plot_y0 + plot_h}" stroke="#94a3b8"/>')
    lines.append(f'<line x1="{plot_x0}" y1="{plot_y0}" x2="{plot_x0}" y2="{plot_y0 + plot_h}" stroke="#94a3b8"/>')
    if reference is not None:
        y = sy(reference)
        lines.append(f'<line x1="{plot_x0}" y1="{y:.2f}" x2="{plot_x0 + plot_w}" y2="{y:.2f}" stroke="#dc2626" stroke-dasharray="6,5" stroke-width="2"/>')

    palette = ["#2563eb", "#dc2626", "#16a34a", "#7c3aed", "#f97316", "#0891b2", "#be123c", "#4d7c0f", "#4338ca", "#a16207"]
    for idx, rows in enumerate(series.values()):
        color = palette[idx % len(palette)]
        points = [(sx(row["time_s"]), sy(row[key])) for row in rows]
        lines.append(polyline(points, color, 2))


def write_maneuver_svg(series: dict[str, list[dict[str, float]]], path: Path, nominal_altitude_m: float) -> None:
    width = 1320
    height = 900
    panel_w = 620
    panel_h = 330
    gap_x = 40
    gap_y = 40
    start_x = 50
    start_y = 85

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f1f5f9"/>',
        '<text x="50" y="40" font-family="Arial" font-size="25" font-weight="bold" fill="#0f172a">Maneuver Metrics</text>',
        '<text x="50" y="62" font-family="Arial" font-size="13" fill="#475569">Altitude, flight path angle, roll angle, and acceleration response</text>',
    ]

    draw_panel(lines, series, "z_m", "Altitude Response", "m", start_x, start_y, panel_w, panel_h, reference=nominal_altitude_m)
    draw_panel(lines, series, "flight_path_deg", "Flight Path Angle", "deg", start_x + panel_w + gap_x, start_y, panel_w, panel_h, symmetric=True)
    draw_panel(lines, series, "roll_deg", "Roll Angle", "deg", start_x, start_y + panel_h + gap_y, panel_w, panel_h, symmetric=True)
    draw_panel(
        lines,
        series,
        "vertical_accel_mps2",
        "Vertical Acceleration",
        "m/s^2",
        start_x + panel_w + gap_x,
        start_y + panel_h + gap_y,
        panel_w,
        panel_h,
        symmetric=True,
    )

    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze UAV maneuver severity from trajectory CSV.")
    parser.add_argument("--input", type=Path, default=None, help="trajectory.csv path. Defaults to latest outputs/realtime_* file.")
    parser.add_argument("--nominal-altitude", type=float, default=80.0)
    args = parser.parse_args()

    trajectory_path = args.input or find_latest_trajectory()
    series = read_trajectory(trajectory_path)
    if not series:
        raise RuntimeError(f"No maneuver data could be computed from {trajectory_path}")

    output_dir = trajectory_path.parent
    summary_path = output_dir / "maneuver_metrics_summary.csv"
    svg_path = output_dir / "maneuver_metrics.svg"
    write_summary_csv(series, summary_path, args.nominal_altitude)
    write_maneuver_svg(series, svg_path, args.nominal_altitude)

    all_rows = [row for rows in series.values() for row in rows]
    max_altitude_error = max(abs(row["z_m"] - args.nominal_altitude) for row in all_rows)
    max_flight_path = max(abs(row["flight_path_deg"]) for row in all_rows)
    max_roll = max(abs(row["roll_deg"]) for row in all_rows)
    max_vertical_accel = max(abs(row["vertical_accel_mps2"]) for row in all_rows)

    print(f"Analyzed: {trajectory_path}")
    print(f"Wrote: {summary_path}")
    print(f"Wrote: {svg_path}")
    print(f"Max altitude error: {max_altitude_error:.2f} m")
    print(f"Max |flight path angle|: {max_flight_path:.2f} deg")
    print(f"Max |roll angle|: {max_roll:.2f} deg")
    print(f"Max |vertical acceleration|: {max_vertical_accel:.2f} m/s^2")


if __name__ == "__main__":
    main()
