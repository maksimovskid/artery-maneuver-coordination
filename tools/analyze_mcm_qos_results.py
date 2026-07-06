#!/usr/bin/env python3
"""Summarize MCM/QoS OMNeT++ scalar statistics into CSV."""

import argparse
import csv
import math
import re
from pathlib import Path


DEFAULT_INPUT = Path("scenarios/artery-maneuver-coordination/results")
DEFAULT_OUTPUT = DEFAULT_INPUT / "mcm_qos_summary.csv"

METRICS = {
    "McmSentCounter",
    "McmReceivedCounter",
    "McmIntentionSentCounter",
    "McmIntentionReceivedCounter",
    "McmNegotiationSentCounter",
    "McmNegotiationReceivedCounter",
    "McmExecutionSentCounter",
    "McmExecutionReceivedCounter",
    "McmExecutionEmergencySentCounter",
    "McmExecutionEmergencyReceivedCounter",
    "msgsizeReceived",
    "EteDelayMcm",
    "EteDelayMcmNegotiation",
    "EteDelayMcmExecution",
    "EteDelayMcmEmergency",
    "dccTimeWaitNextMcm",
    "coopCBR",
    "NegotiationStartedCounter",
    "NegotiationCompletedCounter",
    "ExecutionStartedCounter",
    "ExecutionCompletedCounter",
    "currentMCSoperatingMode",
}

FIELDNAMES = [
    "config",
    "run",
    "file",
    "metric",
    "module",
    "count",
    "mean",
    "min",
    "max",
    "stddev",
    "sum",
    "value",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize selected MCM/QoS OMNeT++ .sca statistics into CSV."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"Directory to scan recursively for .sca files (default: {DEFAULT_INPUT})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"CSV output path (default: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args()


def metric_base(name):
    return name.split(":", 1)[0]


def clean_number(value):
    if value in {"nan", "-nan", "NaN", "-NaN"}:
        return ""
    try:
        number = float(value)
    except ValueError:
        return value
    if math.isnan(number):
        return ""
    return value


def filename_defaults(path):
    match = re.match(r"(?P<config>.+?)(?:-seed=(?P<seed>[^-]+))?(?:-#(?P<run>\d+))?$", path.stem)
    if not match:
        return path.stem, ""
    run = f"#{match.group('run')}" if match.group("run") is not None else ""
    seed = match.group("seed")
    if seed:
        run = f"seed={seed} {run}".strip()
    return match.group("config"), run


def empty_row(config, run, path, metric, module):
    return {
        "config": config,
        "run": run,
        "file": str(path),
        "metric": metric,
        "module": module,
        "count": "",
        "mean": "",
        "min": "",
        "max": "",
        "stddev": "",
        "sum": "",
        "value": "",
    }


def parse_sca(path):
    config, run = filename_defaults(path)
    rows = []
    current_stat = None

    def finish_stat():
        nonlocal current_stat
        if current_stat is not None:
            rows.append(current_stat)
            current_stat = None

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue

            if line.startswith("attr configname "):
                config = line.split(None, 2)[2]
                continue
            if line.startswith("attr runnumber "):
                run = line.split(None, 2)[2]
                continue
            if line.startswith("attr replication ") and not run:
                run = line.split(None, 2)[2]
                continue

            if line.startswith("scalar "):
                finish_stat()
                parts = line.split(None, 3)
                if len(parts) != 4:
                    continue
                _, module, name, value = parts
                metric = metric_base(name)
                if metric not in METRICS:
                    continue

                row = empty_row(config, run, path, metric, module)
                value = clean_number(value)
                if name.endswith(":count"):
                    row["count"] = value
                row["value"] = value
                rows.append(row)
                continue

            if line.startswith("statistic "):
                finish_stat()
                parts = line.split(None, 2)
                if len(parts) != 3:
                    continue
                _, module, name = parts
                metric = metric_base(name)
                if metric in METRICS:
                    current_stat = empty_row(config, run, path, metric, module)
                continue

            if current_stat is not None and line.startswith("field "):
                parts = line.split(None, 2)
                if len(parts) != 3:
                    continue
                _, field_name, value = parts
                if field_name in {"count", "mean", "min", "max", "stddev", "sum"}:
                    current_stat[field_name] = clean_number(value)

    finish_stat()
    return rows


def main():
    args = parse_args()
    sca_files = sorted(args.input.rglob("*.sca"))
    rows = []
    for path in sca_files:
        rows.extend(parse_sca(path))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Scanned {len(sca_files)} .sca file(s); wrote {len(rows)} row(s) to {args.output}")


if __name__ == "__main__":
    main()
