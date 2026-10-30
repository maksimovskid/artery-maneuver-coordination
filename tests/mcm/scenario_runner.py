"""Headless Artery scenario runner for maneuver-coordination regression tests."""

from __future__ import annotations

import os
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
# Also support direct invocation of test scripts from outside the repository root.
sys.path.insert(0, str(REPO_ROOT))
from tools.run_artery import positive_timeout, run_simulation

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


def run_scenario(config: str, run: str = "0", time_limit: str = "30s",
                 extra_args: tuple[str, ...] = (), *,
                 build_dir: str | Path | None = None,
                 wall_clock_timeout: float | None = None) -> ScenarioRun:
    build_dir = Path(build_dir or os.environ.get("ARTERY_BUILD_DIR", "build"))
    if not build_dir.is_absolute():
        build_dir = REPO_ROOT / build_dir
    wall_clock_timeout = positive_timeout(str(
        wall_clock_timeout if wall_clock_timeout is not None
        else os.environ.get("ARTERY_TEST_TIMEOUT", "300")
    ))
    output_dir = Path(tempfile.mkdtemp(prefix=f"mcm-regression-{config}-"))
    log_path = output_dir / "simulation.log"
    result_dir = output_dir / "results"
    sumo_result_dir = output_dir / "sumo"
    result_dir.mkdir(parents=True, exist_ok=True)
    sumo_result_dir.mkdir(parents=True, exist_ok=True)

    sumo_output_options = (
        f"--tripinfo-output {sumo_result_dir / 'simulation.tripinfo.xml'} "
        f"--statistic-output {sumo_result_dir / 'simulation.statistic.xml'}"
    )

    cmd = [
        str(RUN_ARTERY),
        "-l",
        str(build_dir),
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
        f'--*.traci.launcher.extraOptions="{sumo_output_options}"',
    ]

    cmd.extend(extra_args)
    started = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as handle:
        try:
            returncode = run_simulation(
                cmd, cwd=REPO_ROOT, stdout=handle, stderr=subprocess.STDOUT,
                timeout=wall_clock_timeout,
            )
        except subprocess.TimeoutExpired as error:
            raise AssertionError(
                f"scenario {config} exceeded wall-clock timeout of {wall_clock_timeout:g}s; "
                f"log preserved at {log_path}; override ARTERY_TEST_TIMEOUT (seconds)"
            ) from error
    duration_s = time.perf_counter() - started

    parsed_log = ParsedLog.from_path(log_path)
    scenario_run = ScenarioRun(
        config=config,
        output_dir=output_dir,
        log_path=log_path,
        parsed_log=parsed_log,
        returncode=returncode,
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
