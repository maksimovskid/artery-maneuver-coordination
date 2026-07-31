#!/usr/bin/env python3
"""Capture deterministic SUMO/OMNeT++ data for animation metric analysis."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = REPO_ROOT / "scenarios" / "artery-maneuver-coordination"
RUN_ARTERY = REPO_ROOT / "tools" / "run_artery.py"

RUNS = {
    ("merging", "coordinated"): {
        "config": "envmod-19CAVs-merging-animation-data",
        "output": SCENARIO_DIR / "results_animation" / "merging" / "coordinated",
    },
    ("merging", "baseline"): {
        "config": "envmod-19CAVs-merging-baseline-animation-data",
        "output": SCENARIO_DIR / "results_animation" / "merging" / "baseline",
    },
    ("lane-change", "coordinated"): {
        "config": "envmod-19CAVs-emergency-lane-change-animation-data",
        "output": SCENARIO_DIR / "results_animation" / "lane_change" / "coordinated",
    },
    ("lane-change", "baseline"): {
        "config": "envmod-19CAVs-emergency-lane-change-baseline-animation-data",
        "output": SCENARIO_DIR / "results_animation" / "lane_change" / "baseline",
    },
}

EXPECTED_OUTPUTS = ("fcd.xml", "lanechange.xml", "tripinfo.xml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one maneuver-coordination animation data capture.")
    parser.add_argument(
        "scenario",
        choices=("merging", "lane-change"),
        help="Logical scenario to capture.")
    parser.add_argument(
        "variant",
        choices=("coordinated", "baseline"),
        help="Coordinated or baseline run.")
    parser.add_argument(
        "--run",
        default="0",
        help="OMNeT++ run number. Default: 0.")
    parser.add_argument(
        "--time-limit",
        default="30s",
        help="OMNeT++ simulation time limit. Default: 30s.")
    parser.add_argument(
        "--keep",
        action="store_true",
        help="Keep an existing output directory instead of cleaning it first.")
    return parser.parse_args()


def clean_output_dir(output_dir: Path, keep: bool) -> None:
    if output_dir.exists() and not keep:
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)


def run_capture(config: str, run: str, time_limit: str, output_dir: Path) -> int:
    log_path = output_dir / "simulation.log"
    cmd = [
        str(RUN_ARTERY),
        "-l",
        "build",
        "-s",
        str(SCENARIO_DIR),
        "--",
        "omnetpp.ini",
        "-u",
        "Cmdenv",
        "-c",
        config,
        "-r",
        run,
        f"--sim-time-limit={time_limit}",
        "--cmdenv-express-mode=false",
    ]

    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.run(
            cmd,
            cwd=REPO_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )

    return process.returncode


def validate_capture(output_dir: Path) -> list[str]:
    errors: list[str] = []
    log_path = output_dir / "simulation.log"

    if not log_path.exists():
        errors.append(f"missing log file: {log_path}")
    else:
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        if "Error:" in log_text:
            errors.append(f"simulation log contains 'Error:': {log_path}")
        if "Simulation time limit reached" not in log_text:
            errors.append(f"simulation did not report the time limit: {log_path}")

    for filename in EXPECTED_OUTPUTS:
        path = output_dir / filename
        if not path.exists():
            errors.append(f"missing expected SUMO output: {path}")
        elif path.stat().st_size == 0:
            errors.append(f"empty SUMO output: {path}")

    return errors


def main() -> int:
    args = parse_args()
    run = RUNS[(args.scenario, args.variant)]
    output_dir = run["output"]

    clean_output_dir(output_dir, args.keep)
    print(f"capturing {args.scenario} {args.variant}")
    print(f"config: {run['config']}")
    print(f"output: {output_dir.relative_to(REPO_ROOT)}")

    returncode = run_capture(run["config"], args.run, args.time_limit, output_dir)
    if returncode != 0:
        print(
            f"capture failed with exit status {returncode}; "
            f"see {output_dir / 'simulation.log'}",
            file=sys.stderr,
        )
        return returncode

    errors = validate_capture(output_dir)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    print("capture complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
