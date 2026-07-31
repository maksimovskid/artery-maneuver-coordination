"""Headless Artery scenario runner for maneuver-coordination regression tests."""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from .log_assertions import ParsedLog
except ImportError:
    from log_assertions import ParsedLog


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = REPO_ROOT / "scenarios" / "artery-maneuver-coordination"
RUN_ARTERY = REPO_ROOT / "tools" / "run_artery.py"


@dataclass
class ScenarioRun:
    config: str
    output_dir: Path
    log_path: Path
    parsed_log: ParsedLog
    returncode: int
    duration_s: float | None = None

    def __enter__(self) -> "ScenarioRun":
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if exc_type is None:
            self.cleanup()
        else:
            print(f"\nscenario log preserved at {self.log_path}", file=sys.stderr)
        return False

    def cleanup(self) -> None:
        shutil.rmtree(self.output_dir, ignore_errors=True)


def run_scenario(config: str, run: str = "0", time_limit: str = "30s") -> ScenarioRun:
    output_dir = Path(tempfile.mkdtemp(prefix=f"mcm-regression-{config}-"))
    log_path = output_dir / "simulation.log"
    result_dir = output_dir / "results"
    result_dir.mkdir(parents=True, exist_ok=True)

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
        f"--result-dir={result_dir}",
    ]

    started = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as handle:
        process = subprocess.run(
            cmd,
            cwd=REPO_ROOT,
            stdout=handle,
            stderr=subprocess.STDOUT,
            check=False,
        )
    duration_s = time.perf_counter() - started

    parsed_log = ParsedLog.from_path(log_path)
    scenario_run = ScenarioRun(
        config=config,
        output_dir=output_dir,
        log_path=log_path,
        parsed_log=parsed_log,
        returncode=process.returncode,
        duration_s=duration_s,
    )

    errors = validate_run(scenario_run, time_limit)
    if errors:
        joined = "\n".join(errors)
        raise AssertionError(
            f"scenario {config} failed validation; log preserved at "
            f"{log_path}\n{joined}"
        )

    return scenario_run


def validate_run(scenario_run: ScenarioRun, time_limit: str) -> list[str]:
    text = scenario_run.parsed_log.text
    errors: list[str] = []
    if scenario_run.returncode != 0:
        errors.append(f"non-zero exit status: {scenario_run.returncode}")
    if "Error:" in text:
        errors.append("simulation log contains 'Error:'")
    expected_limit = time_limit[:-1] if time_limit.endswith("s") else time_limit
    if "Simulation time limit reached" not in text or f"t={expected_limit}s" not in text:
        errors.append(f"simulation did not report reaching {time_limit}")
    return errors
