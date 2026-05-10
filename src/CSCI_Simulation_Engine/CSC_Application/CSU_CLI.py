from __future__ import annotations

import argparse
from pathlib import Path

from src.CSCI_Simulation_Engine.CSC_Application.CSU_SimulationRunner import run_simulation
from src.CSCI_Simulation_Engine.CSC_Command.CSU_RollCommand import (
    straight_roll_command,
    turn_demo_roll_command,
)
from src.CSCI_Simulation_Engine.CSC_Configuration.CSU_SimConfig import SimConfig
from src.CSCI_Simulation_Engine.CSC_Output.CSU_CsvWriter import write_csv
from src.CSCI_Simulation_Engine.CSC_Output.CSU_SvgWriter import write_svg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D formation flight pseudo dynamics simulator")
    parser.add_argument("--duration", type=float, default=80.0, help="simulation duration in seconds")
    parser.add_argument("--dt", type=float, default=0.1, help="integration time step in seconds")
    parser.add_argument("--speed", type=float, default=15.0, help="UAV speed in m/s")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"), help="directory for CSV and SVG outputs")
    parser.add_argument("--turn-demo", action="store_true", help="apply a simple roll command turn maneuver")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = SimConfig(dt=args.dt, duration=args.duration, speed_mps=args.speed, output_dir=args.output_dir)
    command = turn_demo_roll_command if args.turn_demo else straight_roll_command
    uavs = run_simulation(cfg, command)

    csv_path = cfg.output_dir / "trajectory.csv"
    svg_path = cfg.output_dir / "trajectory.svg"
    write_csv(uavs, csv_path)
    write_svg(uavs, svg_path)

    print(f"Wrote {csv_path}")
    print(f"Wrote {svg_path}")
