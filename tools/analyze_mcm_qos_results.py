#!/usr/bin/env python3
"""Summarize MCM/QoS OMNeT++ scalar statistics into CSV."""

import argparse
import csv
import math
import re
import statistics
from collections import defaultdict
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
    "negotiationTime",
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

AGGREGATE_FIELDNAMES = [
    "config",
    "metric",
    "module",
    "rows",
    "runs",
    "seeds",
    "value_mean",
    "value_stddev",
    "value_min",
    "value_max",
    "count_sum",
    "mean_mean",
    "mean_stddev",
    "mean_min",
    "mean_max",
    "sum_mean",
    "sum_stddev",
    "sum_min",
    "sum_max",
    "negotiation_mcms_per_completed_negotiation",
    "negotiation_mcms_sent_per_started_negotiation",
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
    parser.add_argument(
        "--aggregate-output",
        type=Path,
        help="Optional CSV output path for aggregate summaries across runs/seeds.",
    )
    parser.add_argument(
        "--group-without-module",
        action="store_true",
        help="Aggregate by config and metric only instead of config, metric, and module.",
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
        return path.stem, "", ""
    run = f"#{match.group('run')}" if match.group("run") is not None else ""
    seed = match.group("seed")
    if seed:
        run = f"seed={seed} {run}".strip()
    return match.group("config"), run, seed or ""


def empty_row(config, run, seed, path, metric, module):
    return {
        "config": config,
        "run": run,
        "_seed": seed,
        "_run_key": run or path.stem,
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
    config, run, seed = filename_defaults(path)
    rows = []
    current_stat = None
    runnumber = ""
    replication = ""

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
                runnumber = line.split(None, 2)[2]
                continue
            if line.startswith("attr replication "):
                replication = line.split(None, 2)[2]
                continue
            if line.startswith("itervar seed "):
                seed = line.split(None, 2)[2]
                run_parts = []
                if seed:
                    run_parts.append(f"seed={seed}")
                if replication:
                    run_parts.append(replication)
                elif runnumber:
                    run_parts.append(runnumber)
                run = " ".join(run_parts)
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

                row = empty_row(config, run, seed, path, metric, module)
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
                    current_stat = empty_row(config, run, seed, path, metric, module)
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


def numeric_value(value):
    if value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if math.isnan(number):
        return None
    return number


def format_number(value):
    if value is None:
        return ""
    return f"{value:.12g}"


def value_stats(values):
    if not values:
        return "", "", "", ""
    if len(values) > 1:
        stddev = statistics.stdev(values)
    else:
        stddev = None
    return (
        format_number(statistics.mean(values)),
        format_number(stddev),
        format_number(min(values)),
        format_number(max(values)),
    )


def aggregate_rows(rows, group_without_module=False):
    groups = defaultdict(list)
    for row in rows:
        module = "" if group_without_module else row["module"]
        groups[(row["config"], row["metric"], module)].append(row)

    count_sums_by_context = defaultdict(dict)
    for (config, metric, module), group in groups.items():
        count_values = [numeric_value(row["count"]) for row in group]
        count_values = [value for value in count_values if value is not None]
        if count_values:
            count_sums_by_context[(config, module)][metric] = sum(count_values)

    aggregate = []
    for (config, metric, module), group in sorted(groups.items()):
        value_values = [numeric_value(row["value"]) for row in group]
        value_values = [value for value in value_values if value is not None]
        mean_values = [numeric_value(row["mean"]) for row in group]
        mean_values = [value for value in mean_values if value is not None]
        sum_values = [numeric_value(row["sum"]) for row in group]
        sum_values = [value for value in sum_values if value is not None]
        count_values = [numeric_value(row["count"]) for row in group]
        count_values = [value for value in count_values if value is not None]

        value_mean, value_stddev, value_min, value_max = value_stats(value_values)
        mean_mean, mean_stddev, mean_min, mean_max = value_stats(mean_values)
        sum_mean, sum_stddev, sum_min, sum_max = value_stats(sum_values)

        seeds = {row["_seed"] for row in group if row["_seed"]}
        runs = {row["_run_key"] or row["file"] for row in group}
        counts = count_sums_by_context.get((config, module), {})
        negotiation_sent = counts.get("McmNegotiationSentCounter")
        negotiation_completed = counts.get("NegotiationCompletedCounter")
        negotiation_started = counts.get("NegotiationStartedCounter")

        # Derived aggregate ratios. These are totals within the active grouping
        # context, so --group-without-module gives config-level ratios across
        # all participating modules/runs. Missing or zero denominators are left
        # blank rather than guessed.
        mcms_per_completed = ""
        if negotiation_sent is not None and negotiation_completed:
            mcms_per_completed = format_number(negotiation_sent / negotiation_completed)

        mcms_per_started = ""
        if negotiation_sent is not None and negotiation_started:
            mcms_per_started = format_number(negotiation_sent / negotiation_started)

        aggregate.append({
            "config": config,
            "metric": metric,
            "module": module,
            "rows": len(group),
            "runs": len(runs),
            "seeds": len(seeds),
            "value_mean": value_mean,
            "value_stddev": value_stddev,
            "value_min": value_min,
            "value_max": value_max,
            "count_sum": format_number(sum(count_values)) if count_values else "",
            "mean_mean": mean_mean,
            "mean_stddev": mean_stddev,
            "mean_min": mean_min,
            "mean_max": mean_max,
            "sum_mean": sum_mean,
            "sum_stddev": sum_stddev,
            "sum_min": sum_min,
            "sum_max": sum_max,
            "negotiation_mcms_per_completed_negotiation": mcms_per_completed,
            "negotiation_mcms_sent_per_started_negotiation": mcms_per_started,
        })
    return aggregate


def write_csv(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    sca_files = sorted(args.input.rglob("*.sca"))
    rows = []
    for path in sca_files:
        rows.extend(parse_sca(path))

    write_csv(args.output, FIELDNAMES, rows)

    print(f"Scanned {len(sca_files)} .sca file(s); wrote {len(rows)} row(s) to {args.output}")
    if args.aggregate_output:
        aggregate = aggregate_rows(rows, group_without_module=args.group_without_module)
        write_csv(args.aggregate_output, AGGREGATE_FIELDNAMES, aggregate)
        print(f"Wrote {len(aggregate)} aggregate row(s) to {args.aggregate_output}")


if __name__ == "__main__":
    main()
