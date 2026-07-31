#!/usr/bin/env python3
"""Render side-by-side maneuver-coordination comparison animations."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

from render_replay import (
    DEFAULT_VEHICLE_LENGTH_M,
    DEFAULT_VEHICLE_WIDTH_M,
    LOW_SPEED_MPS,
    MEDIA_DIR,
    REPO_ROOT,
    RESULTS_DIR,
    ROUTES_DIR,
    STOPPED_SPEED_MPS,
    TMP_FRAME_DIR,
    VehicleSample,
    VehicleDimensions,
    VehicleSeries,
    Viewport,
    comparison_file,
    compute_viewport,
    draw_text_box,
    format_acceleration,
    format_speed,
    lane_state,
    load_font,
    load_json,
    make_transform,
    parse_fcd,
    parse_lane_shapes,
    parse_vehicle_dimensions,
    require_capture_outputs,
    role_file,
    save_gif,
    scenario_output_dir,
    vehicle_polygon,
)


SCENARIO_DEFAULTS = {
    "merging": {
        "start": 6.5,
        "end": 26.5,
        "poster_time": 12.2,
        "gif": "merging-comparison.gif",
        "png": "merging-comparison.png",
        "title": "Highway merging",
    },
    "lane-change": {
        "start": 11.5,
        "end": 30.0,
        "poster_time": 12.9,
        "gif": "lane-change-comparison.gif",
        "png": "lane-change-comparison.png",
        "title": "Emergency lane change",
    },
}

VIEW_PRESETS = {
    "merging": {
        "overview": {},
        "closeup": {
            "start": 7.0,
            "end": 16.5,
            "poster_time": 12.2,
            "gif": "merging-comparison-closeup.gif",
            "png": "merging-comparison-closeup.png",
            "viewport": Viewport(
                min_x=216535.0,
                max_x=216620.0,
                min_y=452240.0,
                max_y=452520.0,
            ),
        },
        "interaction": {
            "start": 8.5,
            "end": 15.0,
            "poster_time": 12.2,
            "gif": "merging-comparison-interaction.gif",
            "png": "merging-comparison-interaction.png",
            "viewport": Viewport(
                min_x=216568.0,
                max_x=216612.0,
                min_y=452270.0,
                max_y=452505.0,
            ),
        },
    },
    "lane-change": {
        "overview": {},
        "closeup": {
            "start": 11.5,
            "end": 20.0,
            "poster_time": 12.9,
            "gif": "lane-change-comparison-closeup.gif",
            "png": "lane-change-comparison-closeup.png",
            "viewport": Viewport(
                min_x=216585.0,
                max_x=216625.0,
                min_y=452050.0,
                max_y=452350.0,
            ),
        },
        "interaction": {
            "start": 11.5,
            "end": 18.0,
            "poster_time": 12.9,
            "gif": "lane-change-comparison-interaction.gif",
            "png": "lane-change-comparison-interaction.png",
            "viewport": Viewport(
                min_x=216596.0,
                max_x=216616.0,
                min_y=452110.0,
                max_y=452345.0,
            ),
        },
    },
}

ROLE_COLORS = {
    "RV": (31, 119, 180),
    "CV1": (44, 160, 44),
    "CV2": (22, 132, 22),
    "Emergency": (214, 39, 40),
    "Other": (142, 148, 156),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render README-ready comparison media.")
    parser.add_argument("scenario", choices=("merging", "lane-change", "all"))
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--start", type=float)
    parser.add_argument("--end", type=float)
    parser.add_argument("--view", choices=("overview", "closeup", "interaction"), default="overview")
    parser.add_argument("--output", type=Path, help="Optional output directory for final media.")
    parser.add_argument("--keep-frames", action="store_true")
    return parser.parse_args()


class ScenarioData:
    def __init__(self, scenario: str) -> None:
        require_capture_outputs(scenario)
        self.scenario = scenario
        self.roles = load_json(role_file(scenario))
        self.comparison = load_json(comparison_file(scenario))
        self.metrics = {
            variant: load_json(scenario_output_dir(scenario, variant) / "metrics.json")
            for variant in ("baseline", "coordinated")
        }
        self.events = {
            variant: load_json(scenario_output_dir(scenario, variant) / "events.json")
            for variant in ("baseline", "coordinated")
        }
        self.fcd = {
            variant: parse_fcd(scenario_output_dir(scenario, variant) / "fcd.xml")
            for variant in ("baseline", "coordinated")
        }


def role_map(scenario: str, roles: dict[str, Any]) -> dict[str, tuple[str, tuple[int, int, int]]]:
    if scenario == "merging":
        rv = roles["vehicles"]["rv"]
        cv1, cv2 = roles["vehicles"]["candidate_cvs"][:2]
        return {
            rv: ("RV", ROLE_COLORS["RV"]),
            cv1: ("CV1", ROLE_COLORS["CV1"]),
            cv2: ("CV2", ROLE_COLORS["CV2"]),
        }
    emergency = roles["vehicles"]["emergency_vehicle"]
    rv = roles["vehicles"]["follower_rv"]
    cv1, cv2 = roles["vehicles"]["target_lane_cvs"][:2]
    return {
        emergency: ("Emergency", ROLE_COLORS["Emergency"]),
        rv: ("RV", ROLE_COLORS["RV"]),
        cv1: ("CV1", ROLE_COLORS["CV1"]),
        cv2: ("CV2", ROLE_COLORS["CV2"]),
    }


def vehicle_ids_to_draw(data: ScenarioData) -> list[str]:
    roles = role_map(data.scenario, data.roles)
    all_ids = set(roles)
    for variant in ("baseline", "coordinated"):
        all_ids.update(data.fcd[variant].keys())
    return sorted(all_ids)


def timeline(start: float, end: float, fps: int, hold_s: float = 1.5) -> tuple[list[float], int]:
    step = 1.0 / fps
    count = int(round((end - start) * fps)) + 1
    times = [round(start + index * step, 3) for index in range(count)]
    hold_frames = int(round(hold_s * fps))
    times.extend([round(end, 3)] * hold_frames)
    return times, hold_frames


def first_event_time(events: list[dict[str, Any]], **criteria: str) -> float | None:
    for event in events:
        if all(str(event.get(key, "")) == value for key, value in criteria.items()):
            value = event.get("simTime")
            if isinstance(value, (int, float)):
                return float(value)
    return None


def annotation_for(
    data: ScenarioData,
    variant: str,
    time_s: float,
    sample: VehicleSample | None,
    view: str,
) -> str:
    metrics = data.metrics[variant]["scenario_metrics"]
    events = data.events[variant]

    if data.scenario == "merging":
        if view == "interaction":
            if variant == "baseline":
                if sample and sample.speed < STOPPED_SPEED_MPS:
                    return "RV stopped"
                if sample and (sample.acceleration <= -0.5 or sample.speed < 15.0):
                    return "RV braking"
                return "No coordination"

            request_t = first_event_time(events, tag="MCM-NEGOTIATION", action="SEND", msg="Request")
            accept_t = first_event_time(events, tag="MCM-NEGOTIATION", action="SEND", msg="Accept")
            highway_entry = metrics.get("highway_entry_time_s")
            if highway_entry and time_s >= highway_entry:
                return "Merge"
            if accept_t and time_s >= accept_t:
                return "Gap created"
            if request_t and time_s >= request_t:
                return "Coordination request"
            return "Approaching"

        if variant == "baseline":
            if metrics.get("highway_entry_time_s") and time_s >= metrics["highway_entry_time_s"]:
                return "RV enters highway"
            if sample and sample.speed < STOPPED_SPEED_MPS:
                return "RV stopped"
            if sample and sample.speed < LOW_SPEED_MPS:
                return "RV slowing"
            return "No maneuver coordination"

        request_t = first_event_time(events, tag="MCM-NEGOTIATION", action="SEND", msg="Request")
        accept_t = first_event_time(events, tag="MCM-NEGOTIATION", action="SEND", msg="Accept")
        highway_entry = metrics.get("highway_entry_time_s")
        if highway_entry and time_s >= highway_entry:
            return "Merge completed"
        if accept_t and time_s >= accept_t:
            return "Gap cooperation accepted"
        if request_t and time_s >= request_t:
            return "Coordination request"
        return "Approaching merge"

    emergency_t = metrics.get("emergency_brake_trigger_time_s", 12.0)
    if view == "interaction":
        if variant == "baseline":
            if sample and sample.speed < STOPPED_SPEED_MPS:
                return "Follower stopped"
            if sample and sample.acceleration <= -1.0:
                return "Follower braking"
            if time_s >= emergency_t:
                return "Coordination disabled"
            return "Before emergency"

        start = metrics.get("lane_change_start_time_s")
        armed_t = first_event_time(events, event="safety-critical-trigger-armed")
        if start and time_s >= start:
            return "Lane change"
        if armed_t and time_s >= armed_t:
            return "Follower armed"
        if time_s >= emergency_t:
            return "Emergency braking"
        return "Before emergency"

    if variant == "baseline":
        lane_change_t = metrics.get("lane_change_completion_time_s")
        if lane_change_t and time_s >= lane_change_t:
            return "Delayed lane change"
        if sample and sample.speed < STOPPED_SPEED_MPS:
            return "Follower stopped"
        if sample and sample.acceleration <= -1.0:
            return "Follower braking"
        if time_s >= emergency_t:
            return "Coordination disabled"
        return "Before emergency"

    completion = metrics.get("lane_change_completion_time_s")
    start = metrics.get("lane_change_start_time_s")
    request_t = first_event_time(events, event="queued-request")
    armed_t = first_event_time(events, event="safety-critical-trigger-armed")
    emergency_mcm_t = first_event_time(events, event="sent-emergency-execution-mcm")
    if completion and time_s >= completion:
        return "Lane change completed"
    if start and time_s >= start:
        return "Lane change"
    if request_t and time_s >= request_t:
        return "Lane-change request"
    if armed_t and time_s >= armed_t:
        return "Follower armed"
    if emergency_mcm_t and time_s >= emergency_mcm_t:
        return "Emergency MCM"
    if time_s >= emergency_t:
        return "Emergency braking"
    return "Before emergency"


def draw_roads(
    draw: ImageDraw.ImageDraw,
    lanes,
    transform,
    viewport,
) -> None:
    for lane in lanes:
        xs = [point[0] for point in lane.points]
        ys = [point[1] for point in lane.points]
        if max(xs) < viewport.min_x or min(xs) > viewport.max_x or max(ys) < viewport.min_y or min(ys) > viewport.max_y:
            continue
        points = [transform(x, y) for x, y in lane.points]
        draw.line(points, fill=(186, 194, 204), width=2)


def draw_vehicle(
    draw: ImageDraw.ImageDraw,
    sample: VehicleSample,
    dimensions,
    transform,
    label: str | None,
    color: tuple[int, int, int],
    font,
    highlighted: bool,
    label_offset: tuple[int, int] = (0, -22),
) -> None:
    polygon_world = vehicle_polygon(sample, dimensions)
    polygon = [transform(x, y) for x, y in polygon_world]
    outline = (20, 24, 28) if highlighted else (96, 103, 112)
    draw.polygon(polygon, fill=color, outline=outline)
    if highlighted and label:
        cx, cy = transform(sample.x, sample.y)
        text = label
        bbox = draw.textbbox((0, 0), text, font=font)
        tx = int(cx - (bbox[2] - bbox[0]) / 2 + label_offset[0])
        ty = int(cy + label_offset[1])
        draw.rounded_rectangle(
            (tx - 3, ty - 2, tx + bbox[2] - bbox[0] + 3, ty + bbox[3] - bbox[1] + 3),
            radius=3,
            fill=(255, 255, 255),
            outline=(200, 206, 214),
        )
        draw.text((tx, ty), text, fill=(20, 24, 28), font=font)


def metric_panel_lines(
    scenario: str,
    variant: str,
    sample: VehicleSample | None,
    series: VehicleSeries | None,
    time_s: float,
) -> list[str]:
    if scenario == "merging":
        stopped = series.stopped_duration_until(time_s) if series else 0.0
        return [
            f"Speed: {format_speed(sample.speed if sample else None)}",
            f"Stopped: {stopped:.1f} s",
            f"{lane_state(sample)}",
        ]
    return [
        f"Speed: {format_speed(sample.speed if sample else None)}",
        f"Accel: {format_acceleration(sample.acceleration if sample else None)}",
        f"{lane_state(sample)}",
    ]


def final_summary_lines(scenario: str) -> list[str]:
    if scenario == "merging":
        return [
            "Controlled run, seed 10",
            "Coordination avoided 4.9 s of stopping",
            "Time loss: 14.73 s → 3.93 s",
        ]
    return [
        "Controlled run, seed 10",
        "Coordination avoided stopping",
        "Peak decel: 7.81 → 3.28 m/s²",
    ]


def draw_panel(
    base: Image.Image,
    panel_box: tuple[int, int, int, int],
    data: ScenarioData,
    variant: str,
    time_s: float,
    viewport,
    lanes,
    dimensions_by_type,
    roles,
    fonts,
    final_hold: bool,
    view: str,
) -> None:
    draw = ImageDraw.Draw(base)
    left, top, right, bottom = panel_box
    draw.rectangle(panel_box, fill=(247, 249, 252), outline=(216, 222, 230))
    transform, _ = make_transform(viewport, (left + 10, top + 58, right - 10, bottom - 70))
    draw_roads(draw, lanes, transform, viewport)

    primary_id = data.roles["vehicles"]["rv"] if data.scenario == "merging" else data.roles["vehicles"]["follower_rv"]
    primary_sample: VehicleSample | None = None
    primary_series = data.fcd[variant].get(primary_id)

    for vehicle_id, series in data.fcd[variant].items():
        sample = series.at(time_s)
        if sample is None:
            continue
        if vehicle_id == primary_id:
            primary_sample = sample
        if vehicle_id in roles:
            continue
        dimensions = dimensions_by_type.get(sample.vehicle_type)
        if dimensions is None:
            dimensions = VehicleDimensions(DEFAULT_VEHICLE_LENGTH_M, DEFAULT_VEHICLE_WIDTH_M)
        draw_vehicle(draw, sample, dimensions, transform, None, ROLE_COLORS["Other"], fonts["small"], False)

    label_offsets = {
        "RV": (0, -24),
        "CV1": (-20, 8),
        "CV2": (20, -24),
        "Emergency": (0, 10),
    }
    for vehicle_id, (label, color) in roles.items():
        series = data.fcd[variant].get(vehicle_id)
        if series is None:
            continue
        sample = series.at(time_s)
        if sample is None:
            continue
        if vehicle_id == primary_id:
            primary_sample = sample
        dimensions = dimensions_by_type.get(sample.vehicle_type)
        if dimensions is None:
            dimensions = VehicleDimensions(DEFAULT_VEHICLE_LENGTH_M, DEFAULT_VEHICLE_WIDTH_M)
        draw_vehicle(
            draw,
            sample,
            dimensions,
            transform,
            label,
            color,
            fonts["small_bold"],
            True,
            label_offsets.get(label, (0, -22)),
        )

    heading = "Without coordination" if variant == "baseline" else "With coordination"
    draw.text((left + 14, top + 10), heading, fill=(20, 24, 28), font=fonts["title"])
    draw.text((left + 14, top + 34), f"Simulation time: {time_s:.1f} s", fill=(54, 62, 72), font=fonts["body"])
    annotation = annotation_for(data, variant, time_s, primary_sample, view)
    draw_text_box(
        draw,
        (left + 14, top + 62),
        [annotation],
        fonts["body_bold"],
        background=(255, 255, 255),
    )

    metric_lines = metric_panel_lines(data.scenario, variant, primary_sample, primary_series, time_s)
    draw_text_box(draw, (left + 14, bottom - 62), metric_lines, fonts["small"])

    legend_x = right - 126
    legend_y = top + 12
    line_height = 14
    legend_width = 112
    legend_height = 10 + line_height * len(roles)
    draw.rounded_rectangle(
        (legend_x, legend_y, legend_x + legend_width, legend_y + legend_height),
        radius=4,
        fill=(255, 255, 255),
        outline=(210, 216, 224),
    )
    y = legend_y + 7
    for label, color in roles.values():
        draw.rectangle((legend_x + 7, y + 2, legend_x + 17, y + 12), fill=color, outline=(90, 96, 104))
        draw.text((legend_x + 23, y), label, fill=(42, 48, 56), font=fonts["small"])
        y += line_height

    if final_hold:
        lines = final_summary_lines(data.scenario)
        box_w = 360
        box_x = int((left + right) / 2 - box_w / 2)
        draw.rectangle((box_x, bottom - 148, box_x + box_w, bottom - 74), fill=(255, 255, 255), outline=(182, 190, 202))
        cursor_y = bottom - 139
        for index, line in enumerate(lines):
            font = fonts["body_bold"] if index == 0 else fonts["body"]
            draw.text((box_x + 10, cursor_y), line, fill=(24, 30, 38), font=font)
            cursor_y += 20

    del draw


def render_frame(
    data: ScenarioData,
    time_s: float,
    final_hold: bool,
    width: int,
    height: int,
    viewport,
    lanes,
    dimensions_by_type,
    roles,
    fonts,
    view: str,
) -> Image.Image:
    image = Image.new("RGB", (width, height), (235, 239, 245))
    draw = ImageDraw.Draw(image)
    title = SCENARIO_DEFAULTS[data.scenario]["title"]
    draw.text((18, 12), title, fill=(20, 24, 28), font=fonts["main_title"])
    draw.text((width - 230, 16), "1 simulation second = 1 video second", fill=(76, 86, 98), font=fonts["small"])
    del draw

    gap = 10
    panel_top = 44
    panel_bottom = height - 10
    panel_width = (width - gap - 20) // 2
    left_panel = (10, panel_top, 10 + panel_width, panel_bottom)
    right_panel = (10 + panel_width + gap, panel_top, 10 + panel_width * 2 + gap, panel_bottom)
    draw_panel(image, left_panel, data, "baseline", time_s, viewport, lanes, dimensions_by_type, roles, fonts, final_hold, view)
    draw_panel(image, right_panel, data, "coordinated", time_s, viewport, lanes, dimensions_by_type, roles, fonts, final_hold, view)
    return image


def render_scenario(
    scenario: str,
    view: str,
    fps: int,
    width: int,
    height: int,
    start: float,
    end: float,
    output_dir: Path,
    keep_frames: bool,
) -> dict[str, Any]:
    data = ScenarioData(scenario)
    output_dir.mkdir(parents=True, exist_ok=True)
    lanes = parse_lane_shapes(ROUTES_DIR / "new_map.net.xml")
    dimensions_by_type = parse_vehicle_dimensions(ROUTES_DIR / "vehicle_types.xml")
    roles = role_map(scenario, data.roles)

    focus_ids = set(roles)
    focus_series: list[VehicleSeries] = []
    for variant in ("baseline", "coordinated"):
        for vehicle_id in focus_ids:
            series = data.fcd[variant].get(vehicle_id)
            if series is not None:
                focus_series.append(series)
    view_settings = {**SCENARIO_DEFAULTS[scenario], **VIEW_PRESETS[scenario][view]}
    viewport = view_settings.get("viewport")
    if viewport is None:
        viewport = compute_viewport(focus_series, lanes, start, end)

    fonts = {
        "main_title": load_font(18, True),
        "title": load_font(15, True),
        "body": load_font(12, False),
        "body_bold": load_font(12, True),
        "small": load_font(10, False),
        "small_bold": load_font(10, True),
    }

    times, hold_frames = timeline(start, end, fps)
    frame_dir = TMP_FRAME_DIR / scenario
    if frame_dir.exists() and not keep_frames:
        shutil.rmtree(frame_dir)
    frame_dir.mkdir(parents=True, exist_ok=True)

    frames: list[Image.Image] = []
    poster_time = view_settings["poster_time"]
    poster_frame: Image.Image | None = None
    synchronized = True
    for index, time_s in enumerate(times):
        final_hold = index >= len(times) - hold_frames
        frame = render_frame(
            data,
            time_s,
            final_hold,
            width,
            height,
            viewport,
            lanes,
            dimensions_by_type,
            roles,
            fonts,
            view,
        )
        frames.append(frame)
        if keep_frames:
            frame.save(frame_dir / f"{index:04d}.png")
        if poster_frame is None and abs(time_s - poster_time) < (0.5 / fps + 1e-6):
            poster_frame = frame.copy()

    if poster_frame is None:
        poster_frame = frames[min(range(len(times)), key=lambda i: abs(times[i] - poster_time))].copy()

    gif_path = output_dir / view_settings["gif"]
    png_path = output_dir / view_settings["png"]
    poster_frame.save(png_path)
    save_gif(frames, gif_path, fps)

    if not keep_frames and frame_dir.exists():
        shutil.rmtree(frame_dir)

    return {
        "scenario": scenario,
        "gif": gif_path,
        "png": png_path,
        "frame_count": len(frames),
        "fps": fps,
        "duration_s": len(frames) / fps,
        "width": width,
        "height": height,
        "start": start,
        "end": end,
        "view": view,
        "viewport": {
            "min_x": viewport.min_x,
            "max_x": viewport.max_x,
            "min_y": viewport.min_y,
            "max_y": viewport.max_y,
        },
        "poster_time": poster_time,
        "synchronized": synchronized,
    }


def main() -> int:
    args = parse_args()
    scenarios = ["merging", "lane-change"] if args.scenario == "all" else [args.scenario]
    output_dir = args.output if args.output else MEDIA_DIR
    results: list[dict[str, Any]] = []
    for scenario in scenarios:
        if args.view not in VIEW_PRESETS[scenario]:
            print(f"view '{args.view}' is not available for scenario '{scenario}'", file=sys.stderr)
            return 2
        defaults = {**SCENARIO_DEFAULTS[scenario], **VIEW_PRESETS[scenario][args.view]}
        start = args.start if args.start is not None else defaults["start"]
        end = args.end if args.end is not None else defaults["end"]
        result = render_scenario(
            scenario,
            view=args.view,
            fps=args.fps,
            width=args.width,
            height=args.height,
            start=start,
            end=end,
            output_dir=output_dir,
            keep_frames=args.keep_frames,
        )
        results.append(result)
        print(
            f"rendered {scenario} ({args.view}): {result['gif'].relative_to(REPO_ROOT)} "
            f"({result['frame_count']} frames, {result['duration_s']:.1f}s)"
        )
        print(
            "viewport: "
            f"x=[{result['viewport']['min_x']:.1f}, {result['viewport']['max_x']:.1f}], "
            f"y=[{result['viewport']['min_y']:.1f}, {result['viewport']['max_y']:.1f}]"
        )
        print(f"poster: {result['png'].relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
