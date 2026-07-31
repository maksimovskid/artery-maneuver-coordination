#!/usr/bin/env python3
"""Analyze 200-CAV high-load diagnostic runs."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "scenarios" / "artery-maneuver-coordination"
DEFAULT_INPUT = SCENARIO_DIR / "results_diagnostic" / "200cav_qos"
DEFAULT_OUTPUT = DEFAULT_INPUT
DEFAULT_CONFIG = "envmod-200CAVs-qos-diagnostic"
DEFAULT_RUN = "0"
DEFAULT_SEED = "10"
DEFAULT_LIMIT = "10s"
DEFAULT_SUMOCFG = SCENARIO_DIR / "routes" / "test_200CAVs.sumocfg"
TAG_RE = re.compile(r"\[(MCM-[^\]]+)\]")
KEY_VALUE_RE = re.compile(r'([A-Za-z0-9_]+)=("[^"]*"|\S+)')

MCM_METRICS = {
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
    "executionTime",
    "SecondRequestStartedCounter",
    "SecondRequestCompletedCounter",
    "SecondRequestRejectedCounter",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create CSV summaries for the 200-CAV high-load diagnostic run."
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--run", default=DEFAULT_RUN)
    parser.add_argument("--seed", default=DEFAULT_SEED)
    parser.add_argument("--time-limit", default=DEFAULT_LIMIT)
    parser.add_argument("--sumocfg", type=Path, default=DEFAULT_SUMOCFG)
    parser.add_argument("--runtime-log", type=Path)
    parser.add_argument("--log", type=Path, help="Optional simulation log with MCM event markers.")
    parser.add_argument("--compare-with", type=Path, help="Optional diagnostic_summary.csv to compare against.")
    parser.add_argument("--comparison-output", type=Path, help="Optional output CSV for the comparison.")
    return parser.parse_args()


def format_value(value: float | int | str | None) -> str:
    if value is None:
        return "not_available"
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if math.isnan(value):
        return "not_available"
    return f"{value:.12g}"


def parse_route_files(sumocfg: Path) -> tuple[list[Path], int]:
    tree = ET.parse(sumocfg)
    root = tree.getroot()
    route_files_value = (root.findtext("./input/route-files") or "").strip()
    if not route_files_value:
        route_files = root.find("./input/route-files")
        route_files_value = route_files.get("value", "") if route_files is not None else ""
    route_files = [
        (sumocfg.parent / item.strip()).resolve()
        for item in route_files_value.split(",")
        if item.strip()
    ]
    scheduled = 0
    for route_file in route_files:
        route_tree = ET.parse(route_file)
        route_root = route_tree.getroot()
        scheduled += len(route_root.findall(".//vehicle"))
        for flow in route_root.findall(".//flow"):
            number = flow.get("number")
            if number is not None:
                scheduled += int(float(number))
    return route_files, scheduled


def parse_fcd(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"FCD output not found: {path}")

    rows: list[dict[str, str]] = []
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag != "timestep":
            continue
        time_s = float(element.get("time", "nan"))
        ids = [
            vehicle.get("id", "")
            for vehicle in element.findall("vehicle")
            if vehicle.get("id")
        ]
        count = len(ids)
        rows.append({
            "time_s": format_value(time_s),
            "active_cav_count": str(count),
            "vehicle_ids": " ".join(ids),
        })
        element.clear()
    return rows


def time_to_count(active_rows: list[dict[str, str]]) -> dict[float, int]:
    return {
        float(row["time_s"]): int(row["active_cav_count"])
        for row in active_rows
    }


def nearest_active_count(active_rows: list[dict[str, str]], time_s: float | None) -> str:
    if time_s is None or not active_rows:
        return "not_available"
    counts = time_to_count(active_rows)
    nearest = min(counts, key=lambda candidate: abs(candidate - time_s))
    return str(counts[nearest])


def active_vehicle_seconds(active_rows: list[dict[str, str]]) -> float | None:
    if len(active_rows) < 2:
        return None

    samples = [
        (float(row["time_s"]), int(row["active_cav_count"]))
        for row in active_rows
    ]
    vehicle_seconds = 0.0
    for (time_s, count), (next_time_s, _) in zip(samples, samples[1:]):
        delta = next_time_s - time_s
        if delta > 0.0:
            vehicle_seconds += count * delta
    return vehicle_seconds


def rate_per_vehicle_second(count: str, vehicle_seconds: float | None) -> str:
    if vehicle_seconds is None or vehicle_seconds <= 0.0 or count == "not_available":
        return "not_available"
    try:
        return format_value(float(count) / vehicle_seconds)
    except ValueError:
        return "not_available"


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (percent / 100.0)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[int(rank)]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def parse_vec_channel_load(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"OMNeT++ vector file not found: {path}")

    channel_vectors: set[str] = set()
    values_by_time: dict[float, list[float]] = defaultdict(list)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("vector "):
                parts = line.split()
                if len(parts) >= 4 and parts[3].startswith("ChannelLoad"):
                    channel_vectors.add(parts[1])
                continue
            if not line[0].isdigit():
                continue
            parts = line.split()
            if len(parts) < 4 or parts[0] not in channel_vectors:
                continue
            try:
                time_s = round(float(parts[2]), 6)
                value = float(parts[3])
            except ValueError:
                continue
            values_by_time[time_s].append(value)

    rows: list[dict[str, str]] = []
    for time_s in sorted(values_by_time):
        values = values_by_time[time_s]
        rows.append({
            "time_s": format_value(time_s),
            "station_samples": str(len(values)),
            "mean_cbr": format_value(statistics.mean(values)),
            "median_cbr": format_value(statistics.median(values)),
            "p95_cbr": format_value(percentile(values, 95.0)),
            "max_cbr": format_value(max(values)),
        })
    return rows


def cbr_stats_between(cbr_rows: list[dict[str, str]], start: float | None, end: float | None) -> dict[str, str]:
    if start is None or end is None or not cbr_rows:
        return {
            "cbr_window_start_s": "not_available",
            "cbr_window_end_s": "not_available",
            "cbr_rows": "0",
            "station_samples": "0",
            "mean_cbr": "not_available",
            "max_cbr": "not_available",
        }
    if end < start:
        start, end = end, start
    rows = [
        row for row in cbr_rows
        if start <= float(row["time_s"]) <= end
    ]
    mean_values = [float(row["mean_cbr"]) for row in rows]
    max_values = [float(row["max_cbr"]) for row in rows]
    station_samples = sum(int(row["station_samples"]) for row in rows)
    return {
        "cbr_window_start_s": format_value(start),
        "cbr_window_end_s": format_value(end),
        "cbr_rows": str(len(rows)),
        "station_samples": str(station_samples),
        "mean_cbr": format_value(statistics.mean(mean_values) if mean_values else None),
        "max_cbr": format_value(max(max_values) if max_values else None),
    }


def metric_base(name: str) -> str:
    return name.split(":", 1)[0]


def empty_metric() -> dict[str, list[float]]:
    return {
        "value": [],
        "count": [],
        "mean": [],
        "min": [],
        "max": [],
        "sum": [],
    }


def parse_sca(path: Path) -> dict[str, dict[str, list[float]]]:
    if not path.exists():
        raise FileNotFoundError(f"OMNeT++ scalar file not found: {path}")

    metrics: dict[str, dict[str, list[float]]] = defaultdict(empty_metric)
    current_metric: str | None = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line.startswith("scalar "):
                current_metric = None
                parts = line.split(None, 3)
                if len(parts) != 4:
                    continue
                _, _, name, value = parts
                metric = metric_base(name)
                if metric not in MCM_METRICS:
                    continue
                if name.endswith(":count"):
                    add_number(metrics[metric]["count"], value)
                add_number(metrics[metric]["value"], value)
                continue
            if line.startswith("statistic "):
                parts = line.split(None, 2)
                current_metric = None
                if len(parts) == 3:
                    metric = metric_base(parts[2])
                    if metric in MCM_METRICS:
                        current_metric = metric
                continue
            if current_metric and line.startswith("field "):
                parts = line.split(None, 2)
                if len(parts) == 3 and parts[1] in metrics[current_metric]:
                    add_number(metrics[current_metric][parts[1]], parts[2])
    return metrics


def parse_log_events(path: Path | None) -> list[dict[str, str]]:
    if path is None or not path.exists():
        return []

    events: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, line in enumerate(handle, 1):
            tag_match = TAG_RE.search(line)
            if not tag_match:
                continue
            fields = {
                key: value.strip('"')
                for key, value in KEY_VALUE_RE.findall(line)
            }
            if "action" not in fields or "msg" not in fields:
                fallback = re.search(r"\b(SEND|RECEIVE)\s+(Request|Offer|Confirm|Accept|Reject|Cancel|Execute)\b", line)
                if fallback:
                    fields.setdefault("action", fallback.group(1))
                    fields.setdefault("msg", fallback.group(2))
            fields["tag"] = tag_match.group(1)
            fields["line_number"] = str(line_number)
            events.append(fields)
    return events


def event_time(event: dict[str, str] | None) -> float | None:
    if not event:
        return None
    raw = event.get("t") or event.get("time")
    if raw is None:
        return None
    try:
        return float(raw.rstrip("s"))
    except ValueError:
        return None


def first_event(events: list[dict[str, str]], tag: str, **fields: str) -> dict[str, str] | None:
    for event in events:
        if event.get("tag") != tag:
            continue
        if all(event.get(key) == value for key, value in fields.items()):
            return event
    return None


def first_negotiation_event(events: list[dict[str, str]]) -> dict[str, str] | None:
    for event in events:
        if event.get("tag") == "MCM-NEGOTIATION":
            return event
    return None


def add_number(values: list[float], raw: str) -> None:
    try:
        value = float(raw)
    except ValueError:
        return
    if not math.isnan(value):
        values.append(value)


def summarize_metric(metric: str, fields: dict[str, list[float]]) -> dict[str, str]:
    count_values = fields["count"]
    mean_values = fields["mean"]
    value_values = fields["value"]
    sum_values = fields["sum"]
    return {
        "metric": metric,
        "entries": str(sum(len(values) for values in fields.values())),
        "count_sum": format_value(sum(count_values) if count_values else None),
        "value_sum": format_value(sum(value_values) if value_values else None),
        "mean_of_means": format_value(statistics.mean(mean_values) if mean_values else None),
        "min": format_value(min(fields["min"]) if fields["min"] else None),
        "max": format_value(max(fields["max"]) if fields["max"] else None),
        "sum": format_value(sum(sum_values) if sum_values else None),
    }


def parse_runtime_log(path: Path | None) -> tuple[str, str]:
    if path is None or not path.exists():
        return "not_available", "not_available"
    text = path.read_text(encoding="utf-8", errors="replace")
    elapsed = "not_available"
    memory = "not_available"
    elapsed_match = re.search(r"Elapsed \(wall clock\) time.*\):\s*(.+)", text)
    memory_match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", text)
    if elapsed_match:
        elapsed = elapsed_match.group(1).strip()
    if memory_match:
        memory = memory_match.group(1).strip()
    return elapsed, memory


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def copy_available(summary: dict[str, str], metric_rows: dict[str, dict[str, str]]) -> None:
    mapping = {
        "MCM sent count": ("McmSentCounter", "count_sum"),
        "MCM received count": ("McmReceivedCounter", "count_sum"),
        "Intent MCM sent count": ("McmIntentionSentCounter", "count_sum"),
        "Negotiation MCM sent count": ("McmNegotiationSentCounter", "count_sum"),
        "Execution MCM sent count": ("McmExecutionSentCounter", "count_sum"),
        "negotiation start count": ("NegotiationStartedCounter", "count_sum"),
        "negotiation completion count": ("NegotiationCompletedCounter", "count_sum"),
        "mean negotiation time": ("negotiationTime", "mean_of_means"),
        "execution start count": ("ExecutionStartedCounter", "count_sum"),
        "execution completion count": ("ExecutionCompletedCounter", "count_sum"),
        "mean MCM end-to-end delay": ("EteDelayMcm", "mean_of_means"),
        "mean cooperative CBR at MCM send": ("coopCBR", "mean_of_means"),
        "mean DCC wait if available": ("dccTimeWaitNextMcm", "mean_of_means"),
        "second-request count": ("SecondRequestStartedCounter", "count_sum"),
    }
    for output_name, (metric, field) in mapping.items():
        summary[output_name] = metric_rows.get(metric, {}).get(field, "not_available")


def build_coordination_rows(
    events: list[dict[str, str]],
    active_rows: list[dict[str, str]],
    cbr_rows: list[dict[str, str]],
    metric_rows: dict[str, dict[str, str]],
) -> list[dict[str, str]]:
    event_specs = [
        ("first negotiation event", first_negotiation_event(events)),
        ("first Request", first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Request")),
        ("first medium-priority Request", first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Request", priority="MediumPriority")),
        ("first high-priority Request", first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Request", priority="HighPriority")),
        ("first Offer", first_event(events, "MCM-NEGOTIATION", msg="Offer")),
        ("first Confirm", first_event(events, "MCM-NEGOTIATION", msg="Confirm")),
        ("first Accept", first_event(events, "MCM-NEGOTIATION", msg="Accept")),
        ("first Execute", first_event(events, "MCM-NEGOTIATION", msg="Execute")),
        ("first Reject", first_event(events, "MCM-NEGOTIATION", msg="Reject")),
        ("execution completion marker", first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Cancel")),
    ]

    rows: list[dict[str, str]] = []
    for name, event in event_specs:
        time_s = event_time(event)
        window = cbr_stats_between(cbr_rows, time_s - 1.0 if time_s is not None else None, time_s + 1.0 if time_s is not None else None)
        rows.append({
            "item": name,
            "timestamp_s": format_value(time_s),
            "active_cav_count": nearest_active_count(active_rows, time_s),
            "cbr_window": "+/-1s around event",
            "mean_cbr": window["mean_cbr"],
            "max_cbr": window["max_cbr"],
            "station_samples": window["station_samples"],
            "source": "simulation.log + FCD + ChannelLoad vector",
        })

    request_time = event_time(first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Request"))
    accept_time = event_time(first_event(events, "MCM-NEGOTIATION", msg="Accept"))
    execute_time = event_time(first_event(events, "MCM-NEGOTIATION", msg="Execute"))
    completion_time = event_time(first_event(events, "MCM-NEGOTIATION", action="SEND", msg="Cancel"))
    interval_end = accept_time if accept_time is not None else execute_time
    interval_stats = cbr_stats_between(cbr_rows, request_time, interval_end)
    rows.append({
        "item": "negotiation interval",
        "timestamp_s": f"{format_value(request_time)}..{format_value(interval_end)}",
        "active_cav_count": nearest_active_count(active_rows, request_time),
        "cbr_window": "first Request until first Accept, or first Execute if Accept is unavailable",
        "mean_cbr": interval_stats["mean_cbr"],
        "max_cbr": interval_stats["max_cbr"],
        "station_samples": interval_stats["station_samples"],
        "source": "simulation.log + FCD + ChannelLoad vector",
    })
    rows.append({
        "item": "execution interval",
        "timestamp_s": f"{format_value(execute_time)}..{format_value(completion_time)}",
        "active_cav_count": nearest_active_count(active_rows, execute_time),
        "cbr_window": "first Execute until first execution-completion Cancel marker",
        "mean_cbr": cbr_stats_between(cbr_rows, execute_time, completion_time)["mean_cbr"],
        "max_cbr": cbr_stats_between(cbr_rows, execute_time, completion_time)["max_cbr"],
        "station_samples": cbr_stats_between(cbr_rows, execute_time, completion_time)["station_samples"],
        "source": "simulation.log + FCD + ChannelLoad vector",
    })
    rows.append({
        "item": "mean MCM end-to-end delay during run",
        "timestamp_s": "not_available",
        "active_cav_count": "not_available",
        "cbr_window": "not_available",
        "mean_cbr": "not_available",
        "max_cbr": "not_available",
        "station_samples": "not_available",
        "source": metric_rows.get("EteDelayMcm", {}).get("mean_of_means", "not_available"),
    })
    rows.append({
        "item": "mean cooperative CBR at MCM send during run",
        "timestamp_s": "not_available",
        "active_cav_count": "not_available",
        "cbr_window": "not_available",
        "mean_cbr": metric_rows.get("coopCBR", {}).get("mean_of_means", "not_available"),
        "max_cbr": metric_rows.get("coopCBR", {}).get("max", "not_available"),
        "station_samples": metric_rows.get("coopCBR", {}).get("count_sum", "not_available"),
        "source": "coopCBR scalar statistics",
    })
    rows.append({
        "item": "mean DCC wait during run",
        "timestamp_s": "not_available",
        "active_cav_count": "not_available",
        "cbr_window": "not_available",
        "mean_cbr": "not_available",
        "max_cbr": "not_available",
        "station_samples": metric_rows.get("dccTimeWaitNextMcm", {}).get("count_sum", "not_available"),
        "source": metric_rows.get("dccTimeWaitNextMcm", {}).get("mean_of_means", "not_available"),
    })
    return rows


def read_summary(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        return {row["field"]: row["value"] for row in csv.DictReader(handle)}


def write_comparison(path: Path, baseline: dict[str, str], current: dict[str, str]) -> None:
    metrics = [
        "simulation time limit",
        "wall-clock runtime",
        "peak memory kbytes",
        "peak active CAV count",
        "peak-active timestamp",
        "mean active CAV count",
        "final active CAV count",
        "mean station CBR",
        "median station CBR",
        "95th percentile CBR",
        "maximum station CBR",
        "MCM sent count",
        "Intent MCM sent count",
        "active CAV vehicle-seconds",
        "MCM sent per active-CAV-second",
        "Intent MCM sent per active-CAV-second",
        "CAM sent per active-CAV-second",
        "MCM received count",
        "mean MCM end-to-end delay",
        "negotiation start count",
        "negotiation completion count",
        "mean negotiation time",
        "execution start count",
        "execution completion count",
        "mean DCC wait if available",
        "second-request count",
    ]
    write_csv(
        path,
        ["metric", "10-second run", "30-second run"],
        [{
            "metric": metric,
            "10-second run": baseline.get(metric, "not_available"),
            "30-second run": current.get(metric, "not_available"),
        } for metric in metrics],
    )


def main() -> None:
    args = parse_args()
    input_dir = args.input
    output_dir = args.output
    sca_path = input_dir / f"{args.config}-seed={args.seed}-#{args.run}.sca"
    vec_path = input_dir / f"{args.config}-seed={args.seed}-#{args.run}.vec"
    fcd_path = input_dir / "fcd.xml"

    _, scheduled_demand = parse_route_files(args.sumocfg)
    active_rows = parse_fcd(fcd_path)
    cbr_rows = parse_vec_channel_load(vec_path)
    metric_data = parse_sca(sca_path)
    metric_rows = {
        metric: summarize_metric(metric, fields)
        for metric, fields in sorted(metric_data.items())
    }
    runtime, peak_memory = parse_runtime_log(args.runtime_log)
    events = parse_log_events(args.log)

    active_counts = [int(row["active_cav_count"]) for row in active_rows]
    vehicle_seconds = active_vehicle_seconds(active_rows)
    peak_active = max(active_counts) if active_counts else None
    peak_time = None
    if peak_active is not None:
        for row in active_rows:
            if int(row["active_cav_count"]) == peak_active:
                peak_time = float(row["time_s"])
                break

    summary = {
        "configuration": args.config,
        "run number": args.run,
        "seed": args.seed,
        "simulation time limit": args.time_limit,
        "wall-clock runtime": runtime,
        "peak memory kbytes": peak_memory,
        "scheduled vehicle demand": str(scheduled_demand),
        "active-count source": "SUMO FCD vehicle elements per timestep; all observed SUMO vehicles, matching equipped CAVs in this scenario",
        "peak active CAV count": format_value(peak_active),
        "peak-active timestamp": format_value(peak_time),
        "mean active CAV count": format_value(statistics.mean(active_counts) if active_counts else None),
        "final active CAV count": format_value(active_counts[-1] if active_counts else None),
        "mean station CBR": format_value(statistics.mean([float(row["mean_cbr"]) for row in cbr_rows]) if cbr_rows else None),
        "median station CBR": format_value(statistics.median([float(row["median_cbr"]) for row in cbr_rows]) if cbr_rows else None),
        "95th percentile CBR": format_value(percentile([float(row["p95_cbr"]) for row in cbr_rows], 95.0) if cbr_rows else None),
        "maximum station CBR": format_value(max([float(row["max_cbr"]) for row in cbr_rows]) if cbr_rows else None),
    }
    copy_available(summary, metric_rows)
    summary["active CAV vehicle-seconds"] = format_value(vehicle_seconds)
    summary["MCM sent per active-CAV-second"] = rate_per_vehicle_second(
        summary.get("MCM sent count", "not_available"), vehicle_seconds)
    summary["Intent MCM sent per active-CAV-second"] = rate_per_vehicle_second(
        summary.get("Intent MCM sent count", "not_available"), vehicle_seconds)
    summary["CAM sent per active-CAV-second"] = "not_available"

    write_csv(
        output_dir / "active_population_timeseries.csv",
        ["time_s", "active_cav_count", "vehicle_ids"],
        active_rows,
    )
    write_csv(
        output_dir / "cbr_timeseries.csv",
        ["time_s", "station_samples", "mean_cbr", "median_cbr", "p95_cbr", "max_cbr"],
        cbr_rows,
    )
    write_csv(
        output_dir / "mcm_qos_summary.csv",
        ["metric", "entries", "count_sum", "value_sum", "mean_of_means", "min", "max", "sum"],
        list(metric_rows.values()),
    )
    write_csv(
        output_dir / "diagnostic_summary.csv",
        ["field", "value"],
        [{"field": key, "value": value} for key, value in summary.items()],
    )
    write_csv(
        output_dir / "coordination_period_summary.csv",
        ["item", "timestamp_s", "active_cav_count", "cbr_window", "mean_cbr", "max_cbr", "station_samples", "source"],
        build_coordination_rows(events, active_rows, cbr_rows, metric_rows),
    )

    if args.compare_with:
        if not args.comparison_output:
            raise ValueError("--comparison-output is required with --compare-with")
        write_comparison(args.comparison_output, read_summary(args.compare_with), summary)

    print(f"Wrote diagnostic summary to {output_dir / 'diagnostic_summary.csv'}")
    print(f"Wrote active population time series to {output_dir / 'active_population_timeseries.csv'}")
    print(f"Wrote CBR time series to {output_dir / 'cbr_timeseries.csv'}")
    print(f"Wrote MCM/QoS summary to {output_dir / 'mcm_qos_summary.csv'}")
    print(f"Wrote coordination-period summary to {output_dir / 'coordination_period_summary.csv'}")
    if args.compare_with and args.comparison_output:
        print(f"Wrote comparison to {args.comparison_output}")


if __name__ == "__main__":
    main()
