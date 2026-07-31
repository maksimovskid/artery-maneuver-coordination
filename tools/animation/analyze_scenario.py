#!/usr/bin/env python3
"""Analyze maneuver-coordination animation data captures."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = REPO_ROOT / "scenarios" / "artery-maneuver-coordination"
RESULTS_DIR = SCENARIO_DIR / "results_animation"
ANIMATION_DIR = SCENARIO_DIR / "animation"

RUNS = {
    ("merging", "coordinated"): RESULTS_DIR / "merging" / "coordinated",
    ("merging", "baseline"): RESULTS_DIR / "merging" / "baseline",
    ("lane-change", "coordinated"): RESULTS_DIR / "lane_change" / "coordinated",
    ("lane-change", "baseline"): RESULTS_DIR / "lane_change" / "baseline",
}

ROLE_FILES = {
    "merging": ANIMATION_DIR / "roles_merging.json",
    "lane-change": ANIMATION_DIR / "roles_lane_change.json",
}

STOPPED_SPEED_MPS = 0.1
LOW_SPEED_MPS = 2.0
DEFAULT_DT = 0.1

KEY_VALUE_RE = re.compile(r'([A-Za-z0-9_]+)=("[^"]*"|\S+)')
TAG_RE = re.compile(r"\[(MCM-[^\]]+)\]")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze one captured scenario or create pair comparisons.")
    parser.add_argument(
        "scenario",
        choices=("merging", "lane-change", "all"),
        help="Scenario to analyze, or all.")
    parser.add_argument(
        "variant",
        nargs="?",
        choices=("coordinated", "baseline"),
        help="Analyze only one variant. Omit to analyze both and compare.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def as_float(value: str | None, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_fcd(path: Path) -> dict[str, list[dict[str, Any]]]:
    vehicles: dict[str, list[dict[str, Any]]] = {}
    for event, elem in ET.iterparse(path, events=("end",)):
        if elem.tag != "timestep":
            continue
        time = as_float(elem.get("time"))
        for vehicle in elem.findall("vehicle"):
            vehicle_id = vehicle.get("id")
            if not vehicle_id:
                continue
            sample = {
                "time": time,
                "x": as_float(vehicle.get("x")),
                "y": as_float(vehicle.get("y")),
                "lane": vehicle.get("lane", ""),
                "pos": as_float(vehicle.get("pos")),
                "speed": as_float(vehicle.get("speed")),
                "acceleration": as_float(vehicle.get("acceleration")),
                "angle": as_float(vehicle.get("angle")),
                "type": vehicle.get("type", ""),
            }
            vehicles.setdefault(vehicle_id, []).append(sample)
        elem.clear()
    return vehicles


def parse_lanechanges(path: Path) -> dict[str, list[dict[str, Any]]]:
    lanechanges: dict[str, list[dict[str, Any]]] = {}
    if not path.exists():
        return lanechanges

    root = ET.parse(path).getroot()
    for elem in root.iter():
        if elem.tag not in {"change", "changeStarted", "changeEnded"}:
            continue
        vehicle_id = elem.get("id")
        if not vehicle_id:
            continue
        lanechanges.setdefault(vehicle_id, []).append({
            "type": elem.tag,
            "time": as_float(elem.get("time")),
            "from": elem.get("from", ""),
            "to": elem.get("to", ""),
            "reason": elem.get("reason", ""),
            "speed": as_float(elem.get("speed")),
            "x": as_float(elem.get("x"), math.nan),
            "y": as_float(elem.get("y"), math.nan),
        })
    return lanechanges


def parse_tripinfo(path: Path) -> dict[str, dict[str, Any]]:
    trips: dict[str, dict[str, Any]] = {}
    if not path.exists():
        return trips

    root = ET.parse(path).getroot()
    for elem in root.findall("tripinfo"):
        vehicle_id = elem.get("id")
        if not vehicle_id:
            continue
        trips[vehicle_id] = {
            "depart": as_float(elem.get("depart"), math.nan),
            "arrival": as_float(elem.get("arrival"), math.nan),
            "duration": as_float(elem.get("duration"), math.nan),
            "routeLength": as_float(elem.get("routeLength"), math.nan),
            "waitingTime": as_float(elem.get("waitingTime"), math.nan),
            "timeLoss": as_float(elem.get("timeLoss"), math.nan),
            "departLane": elem.get("departLane", ""),
            "arrivalLane": elem.get("arrivalLane", ""),
        }
    return trips


def parse_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    if not path.exists():
        return events

    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        tag_match = TAG_RE.search(line)
        if not tag_match:
            continue
        fields: dict[str, Any] = {
            "line": line_number,
            "tag": tag_match.group(1),
            "raw": line.strip(),
        }
        for key, value in KEY_VALUE_RE.findall(line):
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            fields[key] = value
        if "simTime" in fields:
            fields["simTime"] = as_float(str(fields["simTime"]), math.nan)
        elif "t" in fields:
            fields["simTime"] = as_float(str(fields["t"]), math.nan)
        events.append(fields)
    return events


def sample_dt(samples: list[dict[str, Any]], index: int) -> float:
    if len(samples) <= 1:
        return DEFAULT_DT
    if index + 1 < len(samples):
        return max(0.0, samples[index + 1]["time"] - samples[index]["time"])
    return max(0.0, samples[index]["time"] - samples[index - 1]["time"])


def vehicle_metrics(
    vehicle_id: str,
    samples: list[dict[str, Any]],
    lanechanges: dict[str, list[dict[str, Any]]],
    tripinfo: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    if not samples:
        return {"vehicle": vehicle_id, "observed": False}

    speeds = [sample["speed"] for sample in samples]
    accelerations = [sample["acceleration"] for sample in samples]
    distance = 0.0
    stopped_duration = 0.0
    low_speed_duration = 0.0
    for index, sample in enumerate(samples):
        dt = sample_dt(samples, index)
        if sample["speed"] < STOPPED_SPEED_MPS:
            stopped_duration += dt
        if sample["speed"] < LOW_SPEED_MPS:
            low_speed_duration += dt
        if index > 0:
            prev = samples[index - 1]
            distance += math.hypot(sample["x"] - prev["x"], sample["y"] - prev["y"])

    metrics = {
        "vehicle": vehicle_id,
        "observed": True,
        "first_observed_time": samples[0]["time"],
        "last_observed_time": samples[-1]["time"],
        "sample_count": len(samples),
        "minimum_speed_mps": min(speeds),
        "maximum_speed_mps": max(speeds),
        "mean_speed_mps": statistics.fmean(speeds),
        "minimum_acceleration_mps2": min(accelerations),
        "peak_deceleration_mps2": max((max(0.0, -a) for a in accelerations), default=0.0),
        "stopped_duration_s": stopped_duration,
        "low_speed_duration_s": low_speed_duration,
        "distance_travelled_m": distance,
        "first_lane": samples[0]["lane"],
        "last_lane": samples[-1]["lane"],
        "unique_lanes": sorted({sample["lane"] for sample in samples if sample["lane"]}),
        "lane_change_count": len(lanechanges.get(vehicle_id, [])),
        "tripinfo": tripinfo.get(vehicle_id, {}),
    }
    return metrics


def first_event(events: list[dict[str, Any]], **criteria: str) -> dict[str, Any] | None:
    for event in events:
        matched = True
        for key, value in criteria.items():
            if str(event.get(key, "")) != value:
                matched = False
                break
        if matched:
            return event
    return None


def events_by_name(events: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for event in events:
        name = str(event.get("event", event.get("tag", "unknown")))
        grouped.setdefault(name, []).append(event)
    return grouped


def lane_edge(lane: str) -> str:
    if "_" not in lane:
        return lane
    return lane.rsplit("_", 1)[0]


def lane_index(lane: str) -> int | None:
    if "_" not in lane:
        return None
    try:
        return int(lane.rsplit("_", 1)[1])
    except ValueError:
        return None


def nearest_time_to_point(samples: list[dict[str, Any]], x: float, y: float) -> float | None:
    if not samples:
        return None
    best = min(samples, key=lambda sample: math.hypot(sample["x"] - x, sample["y"] - y))
    return best["time"]


def duration_within_radius(samples: list[dict[str, Any]], x: float, y: float, radius: float) -> float:
    duration = 0.0
    for index, sample in enumerate(samples):
        if math.hypot(sample["x"] - x, sample["y"] - y) <= radius:
            duration += sample_dt(samples, index)
    return duration


def min_speed_within_radius(samples: list[dict[str, Any]], x: float, y: float, radius: float) -> float | None:
    speeds = [
        sample["speed"] for sample in samples
        if math.hypot(sample["x"] - x, sample["y"] - y) <= radius
    ]
    return min(speeds) if speeds else None


def first_highway_entry_time(samples: list[dict[str, Any]], highway_prefixes: list[str]) -> float | None:
    for sample in samples:
        edge = lane_edge(sample["lane"])
        if any(edge.startswith(prefix) for prefix in highway_prefixes):
            return sample["time"]
    return None


def lane_change_times(changes: list[dict[str, Any]]) -> tuple[float | None, float | None]:
    if not changes:
        return None, None
    starts = [change["time"] for change in changes if change["type"] in {"changeStarted", "change"}]
    ends = [change["time"] for change in changes if change["type"] in {"changeEnded", "change"}]
    return (min(starts) if starts else None, max(ends) if ends else None)


def fcd_lane_transition_times(
    samples: list[dict[str, Any]],
    start_time: float,
) -> dict[str, Any]:
    if not samples:
        return {
            "start_time": None,
            "completion_time": None,
            "occurred": False,
            "initial_lane": "",
            "final_lane": "",
        }

    initial = next((sample for sample in samples if sample["time"] >= start_time), samples[0])
    initial_lane = initial["lane"]
    initial_index = lane_index(initial_lane)
    start_transition = None
    completion = None
    final_lane = samples[-1]["lane"]

    for sample in samples:
        if sample["time"] < start_time:
            continue
        if sample["lane"] != initial_lane and start_transition is None:
            start_transition = sample["time"]
        sample_index = lane_index(sample["lane"])
        if initial_index is not None and sample_index is not None and sample_index > initial_index:
            completion = sample["time"]
            break

    return {
        "start_time": start_transition,
        "completion_time": completion if completion is not None else start_transition,
        "occurred": start_transition is not None,
        "initial_lane": initial_lane,
        "final_lane": final_lane,
    }


def minimum_center_distance(
    first: list[dict[str, Any]],
    second: list[dict[str, Any]],
    start_time: float | None = None,
) -> dict[str, float] | None:
    if not first or not second:
        return None
    second_by_time = {round(sample["time"], 3): sample for sample in second}
    best: dict[str, float] | None = None
    for sample in first:
        if start_time is not None and sample["time"] < start_time:
            continue
        other = second_by_time.get(round(sample["time"], 3))
        if not other:
            continue
        distance = math.hypot(sample["x"] - other["x"], sample["y"] - other["y"])
        if best is None or distance < best["distance_m"]:
            best = {"time": sample["time"], "distance_m": distance}
    return best


def time_behind_vehicle(
    follower: list[dict[str, Any]],
    leader: list[dict[str, Any]],
    max_distance: float,
    start_time: float,
) -> float:
    leader_by_time = {round(sample["time"], 3): sample for sample in leader}
    duration = 0.0
    for index, sample in enumerate(follower):
        if sample["time"] < start_time:
            continue
        other = leader_by_time.get(round(sample["time"], 3))
        if not other:
            continue
        same_lane = sample["lane"] == other["lane"]
        ahead = sample["y"] >= other["y"]
        distance = math.hypot(sample["x"] - other["x"], sample["y"] - other["y"])
        if same_lane and ahead and distance <= max_distance:
            duration += sample_dt(follower, index)
    return duration


def significant_braking_duration(samples: list[dict[str, Any]], start_time: float, threshold: float) -> float:
    duration = 0.0
    for index, sample in enumerate(samples):
        if sample["time"] >= start_time and sample["acceleration"] <= threshold:
            duration += sample_dt(samples, index)
    return duration


def speed_at_or_after(samples: list[dict[str, Any]], time_s: float) -> float | None:
    for sample in samples:
        if sample["time"] >= time_s:
            return sample["speed"]
    return None


def analyze_run(scenario: str, variant: str) -> dict[str, Any]:
    output_dir = RUNS[(scenario, variant)]
    roles = load_json(ROLE_FILES[scenario])
    fcd = parse_fcd(output_dir / "fcd.xml")
    lanechanges = parse_lanechanges(output_dir / "lanechange.xml")
    tripinfo = parse_tripinfo(output_dir / "tripinfo.xml")
    events = parse_events(output_dir / "simulation.log")

    selected_vehicle_ids: list[str]
    if scenario == "merging":
        selected_vehicle_ids = [
            roles["vehicles"]["rv"],
            *roles["vehicles"].get("candidate_cvs", []),
        ]
    else:
        selected_vehicle_ids = [
            roles["vehicles"]["emergency_vehicle"],
            roles["vehicles"]["follower_rv"],
            *roles["vehicles"].get("target_lane_cvs", []),
        ]

    vehicles = {
        vehicle_id: vehicle_metrics(vehicle_id, fcd.get(vehicle_id, []), lanechanges, tripinfo)
        for vehicle_id in selected_vehicle_ids
    }

    scenario_metrics: dict[str, Any] = {}
    grouped_events = events_by_name(events)

    if scenario == "merging":
        rv = roles["vehicles"]["rv"]
        rv_samples = fcd.get(rv, [])
        merge_cfg = roles["merge"]
        merge_x = float(merge_cfg["merge_start"]["x"])
        merge_y = float(merge_cfg["merge_start"]["y"])
        highway_prefixes = list(merge_cfg["highway_entry_edge_prefixes"])
        near_radius = float(merge_cfg["near_merge_radius_m"])
        area_radius = float(merge_cfg["merge_area_radius_m"])
        trip = tripinfo.get(rv, {})
        scenario_metrics = {
            "rv": rv,
            "merge_approach_time_s": nearest_time_to_point(rv_samples, merge_x, merge_y),
            "highway_entry_time_s": first_highway_entry_time(rv_samples, highway_prefixes),
            "time_near_merge_entrance_s": duration_within_radius(rv_samples, merge_x, merge_y, near_radius),
            "minimum_speed_near_merge_mps": min_speed_within_radius(rv_samples, merge_x, merge_y, area_radius),
            "waiting_time_tripinfo_s": trip.get("waitingTime"),
            "time_loss_tripinfo_s": trip.get("timeLoss"),
            "travel_duration_tripinfo_s": trip.get("duration"),
            "request_count": len(grouped_events.get("queued-request", [])) +
                sum(1 for event in events
                    if event.get("tag") == "MCM-NEGOTIATION" and
                    event.get("action") == "SEND" and event.get("msg") == "Request"),
            "negotiation_event_counts": {
                "request": sum(1 for event in events if event.get("msg") == "Request"),
                "offer": sum(1 for event in events if event.get("msg") == "Offer"),
                "confirm": sum(1 for event in events if event.get("msg") == "Confirm"),
                "accept": sum(1 for event in events if event.get("msg") == "Accept"),
                "execute": sum(1 for event in events if event.get("msg") == "Execute"),
            },
            "selected_coordination_participants": next(
                ({
                    "target1": event.get("target1"),
                    "target2": event.get("target2"),
                    "event_time_s": event.get("simTime"),
                }
                 for event in events
                 if event.get("tag") == "MCM-MERGE-TARGET" and
                 event.get("event") == "selection-summary"),
                None,
            ),
        }
    else:
        emergency_cfg = roles["emergency"]
        emergency_vehicle = roles["vehicles"]["emergency_vehicle"]
        follower = roles["vehicles"]["follower_rv"]
        armed = first_event(events, event="safety-critical-trigger-armed")
        if armed and armed.get("vehicleId"):
            follower = str(armed["vehicleId"])
        emergency_start_event = first_event(events, event="emergency-brake-trigger")
        emergency_start = (
            float(emergency_start_event["simTime"])
            if emergency_start_event and not math.isnan(float(emergency_start_event.get("simTime", math.nan)))
            else float(emergency_cfg["start_time_s"])
        )
        follower_samples = fcd.get(follower, [])
        emergency_samples = fcd.get(emergency_vehicle, [])
        follower_trip = tripinfo.get(follower, {})
        lc_start, lc_end = lane_change_times(lanechanges.get(follower, []))
        fcd_transition = fcd_lane_transition_times(follower_samples, emergency_start)
        lane_change_start = lc_start if lc_start is not None else fcd_transition["start_time"]
        lane_change_end = lc_end if lc_end is not None else fcd_transition["completion_time"]
        lane_change_occurred = bool(lanechanges.get(follower)) or bool(fcd_transition["occurred"])
        min_distance = minimum_center_distance(follower_samples, emergency_samples, emergency_start)
        scenario_metrics = {
            "emergency_vehicle": emergency_vehicle,
            "follower_rv": follower,
            "emergency_brake_trigger_time_s": emergency_start,
            "emergency_mcm_suppressed": first_event(events, event="emergency-mcm-suppressed") is not None,
            "emergency_mcm_sent_count": len(grouped_events.get("sent-emergency-execution-mcm", [])),
            "follower_armed_count": len(grouped_events.get("safety-critical-trigger-armed", [])),
            "lane_change_request_count": len(grouped_events.get("queued-request", [])),
            "speed_at_emergency_start_mps": speed_at_or_after(follower_samples, emergency_start),
            "minimum_speed_after_emergency_mps": min(
                (sample["speed"] for sample in follower_samples if sample["time"] >= emergency_start),
                default=None,
            ),
            "peak_deceleration_after_emergency_mps2": max(
                (max(0.0, -sample["acceleration"]) for sample in follower_samples if sample["time"] >= emergency_start),
                default=0.0,
            ),
            "significant_braking_duration_s": significant_braking_duration(
                follower_samples,
                emergency_start,
                float(emergency_cfg["significant_braking_threshold_mps2"]),
            ),
            "stopped_duration_after_emergency_s": sum(
                sample_dt(follower_samples, index)
                for index, sample in enumerate(follower_samples)
                if sample["time"] >= emergency_start and sample["speed"] < STOPPED_SPEED_MPS
            ),
            "lane_change_start_time_s": lane_change_start,
            "lane_change_completion_time_s": lane_change_end,
            "lane_change_occurred": lane_change_occurred,
            "lane_change_time_source": "SUMO lanechange output" if lc_start is not None else "FCD lane transition",
            "fcd_lane_transition": fcd_transition,
            "time_behind_emergency_vehicle_s": time_behind_vehicle(
                follower_samples,
                emergency_samples,
                float(emergency_cfg["behind_distance_m"]),
                emergency_start,
            ),
            "minimum_center_distance_to_emergency_m": min_distance,
            "waiting_time_tripinfo_s": follower_trip.get("waitingTime"),
            "time_loss_tripinfo_s": follower_trip.get("timeLoss"),
            "travel_duration_tripinfo_s": follower_trip.get("duration"),
        }

    return {
        "scenario": scenario,
        "variant": variant,
        "output_dir": str(output_dir.relative_to(REPO_ROOT)),
        "thresholds": {
            "stopped_speed_mps": STOPPED_SPEED_MPS,
            "low_speed_mps": LOW_SPEED_MPS,
        },
        "roles": roles,
        "vehicles": vehicles,
        "scenario_metrics": scenario_metrics,
        "event_counts": {
            name: len(items)
            for name, items in sorted(grouped_events.items())
        },
        "fcd_vehicle_count": len(fcd),
        "fcd_vehicle_ids": sorted(fcd.keys()),
    }


def flatten_metric_rows(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for vehicle_id, data in metrics["vehicles"].items():
        for key, value in data.items():
            if isinstance(value, (dict, list)):
                continue
            rows.append({
                "scope": "vehicle",
                "vehicle": vehicle_id,
                "metric": key,
                "value": value,
            })
    for key, value in metrics["scenario_metrics"].items():
        if isinstance(value, (dict, list)):
            value = json.dumps(value, sort_keys=True)
        rows.append({
            "scope": "scenario",
            "vehicle": "",
            "metric": key,
            "value": value,
        })
    return rows


def write_run_outputs(scenario: str, variant: str, metrics: dict[str, Any]) -> None:
    output_dir = RUNS[(scenario, variant)]
    events = parse_events(output_dir / "simulation.log")
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "events.json").write_text(
        json.dumps(events, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    rows = flatten_metric_rows(metrics)
    with (output_dir / "metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=("scope", "vehicle", "metric", "value"))
        writer.writeheader()
        writer.writerows(rows)


def numeric(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number):
        return None
    return number


def compare_value(coordinated: Any, baseline: Any, lower_is_better: bool, source: str) -> dict[str, Any]:
    c = numeric(coordinated)
    b = numeric(baseline)
    result: dict[str, Any] = {
        "coordinated": coordinated,
        "baseline": baseline,
        "absolute_difference": None,
        "percentage_difference": None,
        "interpretation": "unavailable",
        "data_source": source,
        "confidence": "unavailable",
    }
    if c is None or b is None:
        result["interpretation"] = "Metric unavailable in one or both runs."
        return result

    diff = c - b
    result["absolute_difference"] = diff
    if abs(b) > 1e-9:
        result["percentage_difference"] = diff / b * 100.0

    epsilon = max(1e-6, abs(b) * 0.01)
    if abs(diff) <= epsilon:
        result["interpretation"] = "Effectively equal within a 1% tolerance."
        result["confidence"] = "medium"
    elif (diff < 0 and lower_is_better) or (diff > 0 and not lower_is_better):
        result["interpretation"] = "Coordinated run is lower/better for this metric." if lower_is_better else "Coordinated run is higher/better for this metric."
        result["confidence"] = "medium"
    else:
        result["interpretation"] = "Coordinated run is not better for this metric."
        result["confidence"] = "medium"
    return result


def build_comparison(scenario: str, coordinated: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    if scenario == "merging":
        rv = coordinated["scenario_metrics"]["rv"]
        metric_specs = [
            ("rv_minimum_speed_mps", coordinated["vehicles"][rv]["minimum_speed_mps"], baseline["vehicles"][rv]["minimum_speed_mps"], False, "FCD"),
            ("rv_stopped_duration_s", coordinated["vehicles"][rv]["stopped_duration_s"], baseline["vehicles"][rv]["stopped_duration_s"], True, "FCD"),
            ("rv_low_speed_duration_s", coordinated["vehicles"][rv]["low_speed_duration_s"], baseline["vehicles"][rv]["low_speed_duration_s"], True, "FCD"),
            ("rv_time_near_merge_entrance_s", coordinated["scenario_metrics"]["time_near_merge_entrance_s"], baseline["scenario_metrics"]["time_near_merge_entrance_s"], True, "FCD distance to configured merge point"),
            ("rv_highway_entry_time_s", coordinated["scenario_metrics"]["highway_entry_time_s"], baseline["scenario_metrics"]["highway_entry_time_s"], True, "FCD lane edge transition"),
            ("rv_minimum_speed_near_merge_mps", coordinated["scenario_metrics"]["minimum_speed_near_merge_mps"], baseline["scenario_metrics"]["minimum_speed_near_merge_mps"], False, "FCD distance to configured merge point"),
            ("rv_waiting_time_tripinfo_s", coordinated["scenario_metrics"]["waiting_time_tripinfo_s"], baseline["scenario_metrics"]["waiting_time_tripinfo_s"], True, "SUMO tripinfo"),
            ("rv_time_loss_tripinfo_s", coordinated["scenario_metrics"]["time_loss_tripinfo_s"], baseline["scenario_metrics"]["time_loss_tripinfo_s"], True, "SUMO tripinfo"),
        ]
    else:
        follower = coordinated["scenario_metrics"]["follower_rv"]
        baseline_follower = baseline["roles"]["vehicles"]["follower_rv"]
        metric_specs = [
            ("follower_minimum_speed_after_emergency_mps", coordinated["scenario_metrics"]["minimum_speed_after_emergency_mps"], baseline["scenario_metrics"]["minimum_speed_after_emergency_mps"], False, "FCD"),
            ("follower_peak_deceleration_after_emergency_mps2", coordinated["scenario_metrics"]["peak_deceleration_after_emergency_mps2"], baseline["scenario_metrics"]["peak_deceleration_after_emergency_mps2"], True, "FCD acceleration"),
            ("follower_significant_braking_duration_s", coordinated["scenario_metrics"]["significant_braking_duration_s"], baseline["scenario_metrics"]["significant_braking_duration_s"], True, "FCD acceleration"),
            ("follower_stopped_duration_after_emergency_s", coordinated["scenario_metrics"]["stopped_duration_after_emergency_s"], baseline["scenario_metrics"]["stopped_duration_after_emergency_s"], True, "FCD speed"),
            ("follower_time_behind_emergency_vehicle_s", coordinated["scenario_metrics"]["time_behind_emergency_vehicle_s"], baseline["scenario_metrics"]["time_behind_emergency_vehicle_s"], True, "FCD center positions and lane IDs"),
            ("follower_lane_change_completion_time_s", coordinated["scenario_metrics"]["lane_change_completion_time_s"], baseline["scenario_metrics"]["lane_change_completion_time_s"], True, "SUMO lanechange output or FCD lane transition"),
            ("follower_minimum_center_distance_to_emergency_m", (coordinated["scenario_metrics"]["minimum_center_distance_to_emergency_m"] or {}).get("distance_m"), (baseline["scenario_metrics"]["minimum_center_distance_to_emergency_m"] or {}).get("distance_m"), False, "FCD center-to-center distance"),
            ("follower_waiting_time_tripinfo_s", coordinated["scenario_metrics"]["waiting_time_tripinfo_s"], baseline["scenario_metrics"]["waiting_time_tripinfo_s"], True, "SUMO tripinfo"),
            ("follower_time_loss_tripinfo_s", coordinated["scenario_metrics"]["time_loss_tripinfo_s"], baseline["scenario_metrics"]["time_loss_tripinfo_s"], True, "SUMO tripinfo"),
        ]

    comparisons = {
        name: compare_value(c, b, lower, source)
        for name, c, b, lower, source in metric_specs
    }
    return {
        "scenario": scenario,
        "coordinated_output": coordinated["output_dir"],
        "baseline_output": baseline["output_dir"],
        "metrics": comparisons,
        "claim_assessment": assess_claim(scenario, comparisons),
    }


def assess_claim(scenario: str, comparisons: dict[str, dict[str, Any]]) -> dict[str, str]:
    if scenario == "merging":
        keys = [
            "rv_stopped_duration_s",
            "rv_low_speed_duration_s",
            "rv_time_near_merge_entrance_s",
            "rv_highway_entry_time_s",
            "rv_waiting_time_tripinfo_s",
            "rv_time_loss_tripinfo_s",
        ]
        claim = "merging coordination reduces waiting or time loss"
    else:
        keys = [
            "follower_peak_deceleration_after_emergency_mps2",
            "follower_significant_braking_duration_s",
            "follower_time_behind_emergency_vehicle_s",
            "follower_minimum_speed_after_emergency_mps",
        ]
        claim = "coordinated emergency lane change reduces severe deceleration or prolonged following"

    available = [comparisons[key] for key in keys if comparisons.get(key, {}).get("confidence") != "unavailable"]
    better = [item for item in available if "better" in item["interpretation"] and "not better" not in item["interpretation"]]
    worse = [item for item in available if "not better" in item["interpretation"]]
    equal = [item for item in available if "Effectively equal" in item["interpretation"]]

    if not available:
        status = "inconclusive"
    elif better and not worse:
        status = "supported" if len(better) >= 2 else "partially supported"
    elif better and worse:
        status = "partially supported"
    elif equal and not worse:
        status = "inconclusive"
    else:
        status = "unsupported"

    return {
        "claim": claim,
        "status": status,
        "basis": f"{len(better)} better, {len(equal)} effectively equal, {len(worse)} not better among {len(available)} available metrics.",
    }


def write_comparison_markdown(path: Path, comparison: dict[str, Any]) -> None:
    lines = [
        f"# {comparison['scenario'].replace('-', ' ').title()} Comparison",
        "",
        f"Claim assessment: **{comparison['claim_assessment']['status']}**",
        "",
        comparison["claim_assessment"]["basis"],
        "",
        "| Metric | Coordinated | Baseline | Difference | Interpretation | Source |",
        "| --- | ---: | ---: | ---: | --- | --- |",
    ]
    for name, data in comparison["metrics"].items():
        lines.append(
            "| {name} | {coordinated} | {baseline} | {difference} | {interpretation} | {source} |".format(
                name=name,
                coordinated=format_value(data["coordinated"]),
                baseline=format_value(data["baseline"]),
                difference=format_value(data["absolute_difference"]),
                interpretation=data["interpretation"],
                source=data["data_source"],
            )
        )
    lines.append("")
    lines.append("Center-distance metrics use vehicle centre points, not bumper clearance.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def format_value(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        if math.isnan(value):
            return "n/a"
        return f"{value:.3f}"
    return str(value)


def analyze_and_write(scenario: str, variant: str) -> dict[str, Any]:
    metrics = analyze_run(scenario, variant)
    write_run_outputs(scenario, variant, metrics)
    return metrics


def compare_and_write(scenario: str, coordinated: dict[str, Any], baseline: dict[str, Any]) -> None:
    output_dir = RESULTS_DIR / ("lane_change" if scenario == "lane-change" else scenario)
    output_dir.mkdir(parents=True, exist_ok=True)
    comparison = build_comparison(scenario, coordinated, baseline)
    (output_dir / "comparison.json").write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_comparison_markdown(output_dir / "comparison.md", comparison)


def main() -> int:
    args = parse_args()
    scenarios = ["merging", "lane-change"] if args.scenario == "all" else [args.scenario]

    for scenario in scenarios:
        variants = [args.variant] if args.variant else ["coordinated", "baseline"]
        analyzed: dict[str, dict[str, Any]] = {}
        for variant in variants:
            output_dir = RUNS[(scenario, variant)]
            missing = [name for name in ("fcd.xml", "lanechange.xml", "tripinfo.xml", "simulation.log")
                       if not (output_dir / name).exists()]
            if missing:
                print(f"{scenario} {variant}: missing capture outputs: {', '.join(missing)}", file=sys.stderr)
                return 1
            analyzed[variant] = analyze_and_write(scenario, variant)
            print(f"analyzed {scenario} {variant}: {output_dir.relative_to(REPO_ROOT)}")

        if not args.variant:
            compare_and_write(scenario, analyzed["coordinated"], analyzed["baseline"])
            comparison_dir = RESULTS_DIR / ("lane_change" if scenario == "lane-change" else scenario)
            print(f"wrote comparison: {comparison_dir.relative_to(REPO_ROOT)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
