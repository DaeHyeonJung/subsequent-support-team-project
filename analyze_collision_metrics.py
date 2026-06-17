from __future__ import annotations

import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path


def find_latest_trajectory() -> Path:
    candidates = sorted(Path("outputs").glob("realtime_*/trajectory.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError("No realtime trajectory.csv found under outputs/realtime_*")
    return candidates[-1]


def read_trajectory(path: Path) -> dict[float, list[dict[str, float | str]]]:
    frames: dict[float, list[dict[str, float | str]]] = defaultdict(list)
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row.get("vehicle_health") == "KILLED":
                continue
            t_s = float(row["time_s"])
            frames[t_s].append(
                {
                    "uid": row["uav_id"],
                    "x_m": float(row["x_m"]),
                    "y_m": float(row["y_m"]),
                    "z_m": float(row.get("z_m", 80.0)),
                }
            )
    return dict(sorted(frames.items()))


def compute_metrics(frames: dict[float, list[dict[str, float | str]]]) -> list[dict[str, float | str]]:
    metrics = []
    for t_s, states in frames.items():
        min_3d = float("inf")
        min_horizontal = float("inf")
        min_vertical = 0.0
        closest_pair = ""

        for idx, first in enumerate(states):
            for second in states[idx + 1 :]:
                dx = float(first["x_m"]) - float(second["x_m"])
                dy = float(first["y_m"]) - float(second["y_m"])
                dz = float(first["z_m"]) - float(second["z_m"])
                horizontal = math.hypot(dx, dy)
                distance_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
                if distance_3d < min_3d:
                    min_3d = distance_3d
                    min_horizontal = horizontal
                    min_vertical = abs(dz)
                    closest_pair = f"{first['uid']}-{second['uid']}"

        if math.isfinite(min_3d):
            metrics.append(
                {
                    "time_s": t_s,
                    "min_distance_3d_m": min_3d,
                    "min_horizontal_distance_m": min_horizontal,
                    "vertical_separation_at_min_m": min_vertical,
                    "closest_pair": closest_pair,
                }
            )
    return metrics


def write_metrics_csv(metrics: list[dict[str, float | str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "time_s",
                "min_distance_3d_m",
                "min_horizontal_distance_m",
                "vertical_separation_at_min_m",
                "closest_pair",
            ],
        )
        writer.writeheader()
        writer.writerows(metrics)


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if abs(src_max - src_min) < 1e-9:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) / (src_max - src_min) * (dst_max - dst_min)


def polyline(points: list[tuple[float, float]], color: str, width: int = 2) -> str:
    coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"/>'


def write_metrics_svg(metrics: list[dict[str, float | str]], path: Path, protected_radius_m: float) -> None:
    width = 1200
    height = 640
    margin_l = 80
    margin_r = 40
    margin_t = 60
    margin_b = 70
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b

    times = [float(row["time_s"]) for row in metrics]
    min_3d = [float(row["min_distance_3d_m"]) for row in metrics]
    min_h = [float(row["min_horizontal_distance_m"]) for row in metrics]
    vertical = [float(row["vertical_separation_at_min_m"]) for row in metrics]
    y_max = max(max(min_3d), max(min_h), max(vertical), protected_radius_m) * 1.15
    y_min = 0.0
    t_min = min(times)
    t_max = max(times)

    def sx(t_s: float) -> float:
        return scale(t_s, t_min, t_max, margin_l, margin_l + plot_w)

    def sy(value: float) -> float:
        return scale(value, y_min, y_max, margin_t + plot_h, margin_t)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="80" y="35" font-family="Arial" font-size="22" font-weight="bold" fill="#0f172a">Collision Avoidance Metrics</text>',
        '<text x="80" y="55" font-family="Arial" font-size="13" fill="#475569">Minimum pairwise distance over time</text>',
        f'<line x1="{margin_l}" y1="{margin_t + plot_h}" x2="{margin_l + plot_w}" y2="{margin_t + plot_h}" stroke="#94a3b8"/>',
        f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{margin_t + plot_h}" stroke="#94a3b8"/>',
    ]

    for tick in range(0, 6):
        value = y_max * tick / 5.0
        y = sy(value)
        lines.append(f'<line x1="{margin_l}" y1="{y:.2f}" x2="{margin_l + plot_w}" y2="{y:.2f}" stroke="#e2e8f0"/>')
        lines.append(f'<text x="{margin_l - 10}" y="{y + 4:.2f}" font-family="Arial" font-size="11" fill="#64748b" text-anchor="end">{value:.1f}</text>')

    safety_y = sy(protected_radius_m)
    lines.append(f'<line x1="{margin_l}" y1="{safety_y:.2f}" x2="{margin_l + plot_w}" y2="{safety_y:.2f}" stroke="#dc2626" stroke-width="2" stroke-dasharray="7,5"/>')
    lines.append(f'<text x="{margin_l + plot_w - 4}" y="{safety_y - 7:.2f}" font-family="Arial" font-size="12" fill="#dc2626" text-anchor="end">protected radius {protected_radius_m:.1f} m</text>')

    lines.append(polyline([(sx(t), sy(v)) for t, v in zip(times, min_3d)], "#2563eb", 3))
    lines.append(polyline([(sx(t), sy(v)) for t, v in zip(times, min_h)], "#16a34a", 2))
    lines.append(polyline([(sx(t), sy(v)) for t, v in zip(times, vertical)], "#7c3aed", 2))

    legend_y = height - 30
    legend = [
        ("#2563eb", "min 3D distance"),
        ("#16a34a", "min horizontal distance"),
        ("#7c3aed", "vertical separation at closest pair"),
        ("#dc2626", "protected radius"),
    ]
    x = margin_l
    for color, label in legend:
        lines.append(f'<line x1="{x}" y1="{legend_y}" x2="{x + 28}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>')
        lines.append(f'<text x="{x + 35}" y="{legend_y + 4}" font-family="Arial" font-size="12" fill="#334155">{label}</text>')
        x += 245

    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze UAV collision avoidance metrics from trajectory CSV.")
    parser.add_argument("--input", type=Path, default=None, help="trajectory.csv path. Defaults to latest outputs/realtime_* file.")
    parser.add_argument("--protected-radius", "--safety-radius", type=float, default=7.0)
    args = parser.parse_args()

    trajectory_path = args.input or find_latest_trajectory()
    metrics = compute_metrics(read_trajectory(trajectory_path))
    if not metrics:
        raise RuntimeError(f"No metrics could be computed from {trajectory_path}")

    output_dir = trajectory_path.parent
    csv_path = output_dir / "collision_metrics.csv"
    svg_path = output_dir / "collision_metrics.svg"
    write_metrics_csv(metrics, csv_path)
    write_metrics_svg(metrics, svg_path, args.protected_radius)

    min_row = min(metrics, key=lambda row: float(row["min_distance_3d_m"]))
    violations = sum(1 for row in metrics if float(row["min_distance_3d_m"]) < args.protected_radius)
    violating_pairs = Counter(
        str(row["closest_pair"])
        for row in metrics
        if float(row["min_distance_3d_m"]) < args.protected_radius
    )
    print(f"Analyzed: {trajectory_path}")
    print(f"Wrote: {csv_path}")
    print(f"Wrote: {svg_path}")
    print(
        "Minimum 3D distance: "
        f"{float(min_row['min_distance_3d_m']):.2f} m at t={float(min_row['time_s']):.2f}s "
        f"({min_row['closest_pair']})"
    )
    print(f"Protected-radius violations: {violations}")
    if violating_pairs:
        print("Top violating pairs:")
        for pair, count in violating_pairs.most_common(5):
            print(f"  {pair}: {count}")


if __name__ == "__main__":
    main()
