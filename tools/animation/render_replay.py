#!/usr/bin/env python3
"""Deterministic data-driven replay helpers for maneuver animations."""

from __future__ import annotations

import bisect
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = REPO_ROOT / "scenarios" / "artery-maneuver-coordination"
RESULTS_DIR = SCENARIO_DIR / "results_animation"
ROUTES_DIR = SCENARIO_DIR / "routes"
MEDIA_DIR = REPO_ROOT / "docs" / "media"
TMP_FRAME_DIR = RESULTS_DIR / "tmp_frames"

STOPPED_SPEED_MPS = 0.1
LOW_SPEED_MPS = 2.0
DEFAULT_VEHICLE_LENGTH_M = 5.0
DEFAULT_VEHICLE_WIDTH_M = 1.8


@dataclass(frozen=True)
class VehicleSample:
    time: float
    x: float
    y: float
    lane: str
    pos: float
    speed: float
    acceleration: float
    angle: float
    vehicle_type: str


@dataclass(frozen=True)
class VehicleDimensions:
    length: float = DEFAULT_VEHICLE_LENGTH_M
    width: float = DEFAULT_VEHICLE_WIDTH_M


@dataclass
class VehicleSeries:
    vehicle_id: str
    samples: list[VehicleSample]

    def __post_init__(self) -> None:
        self.times = [sample.time for sample in self.samples]

    def at(self, time_s: float) -> VehicleSample | None:
        if not self.samples:
            return None
        index = bisect.bisect_left(self.times, time_s)
        if index <= 0:
            return self.samples[0]
        if index >= len(self.samples):
            return self.samples[-1]
        before = self.samples[index - 1]
        after = self.samples[index]
        if abs(before.time - time_s) <= abs(after.time - time_s):
            return before
        return after

    def stopped_duration_until(self, time_s: float) -> float:
        return duration_until(self.samples, time_s, lambda sample: sample.speed < STOPPED_SPEED_MPS)


@dataclass(frozen=True)
class LaneShape:
    lane_id: str
    points: tuple[tuple[float, float], ...]
    width: float = 3.2
    index: int = 0
    edge_id: str = ""
    lane_count: int = 1
    internal: bool = False


@dataclass(frozen=True)
class Viewport:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_float(value: str | None, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_fcd(path: Path) -> dict[str, VehicleSeries]:
    vehicles: dict[str, list[VehicleSample]] = {}
    for _, elem in ET.iterparse(path, events=("end",)):
        if elem.tag != "timestep":
            continue
        time_s = parse_float(elem.get("time"))
        for vehicle in elem.findall("vehicle"):
            vehicle_id = vehicle.get("id")
            if not vehicle_id:
                continue
            sample = VehicleSample(
                time=time_s,
                x=parse_float(vehicle.get("x")),
                y=parse_float(vehicle.get("y")),
                lane=vehicle.get("lane", ""),
                pos=parse_float(vehicle.get("pos")),
                speed=parse_float(vehicle.get("speed")),
                acceleration=parse_float(vehicle.get("acceleration")),
                angle=parse_float(vehicle.get("angle")),
                vehicle_type=vehicle.get("type", ""),
            )
            vehicles.setdefault(vehicle_id, []).append(sample)
        elem.clear()
    return {
        vehicle_id: VehicleSeries(vehicle_id, samples)
        for vehicle_id, samples in vehicles.items()
    }


def parse_vehicle_dimensions(path: Path) -> dict[str, VehicleDimensions]:
    dimensions: dict[str, VehicleDimensions] = {}
    root = ET.parse(path).getroot()
    for elem in root.findall("vType"):
        vehicle_type = elem.get("id")
        if not vehicle_type:
            continue
        dimensions[vehicle_type] = VehicleDimensions(
            length=parse_float(elem.get("length"), DEFAULT_VEHICLE_LENGTH_M),
            width=parse_float(elem.get("width"), DEFAULT_VEHICLE_WIDTH_M),
        )
    return dimensions


def vehicle_dimensions(vehicle_type: str, dimensions_by_type) -> VehicleDimensions:
    # TraCI creates per-vehicle types such as car_ml@car_ml1_2 when a maximum
    # speed is changed; inherit the original SUMO vType dimensions.
    return dimensions_by_type.get(vehicle_type,
        dimensions_by_type.get(vehicle_type.split("@", 1)[0], VehicleDimensions()))


def parse_lane_shapes(path: Path) -> list[LaneShape]:
    lanes: list[LaneShape] = []
    root = ET.parse(path).getroot()
    for edge in root.findall("edge"):
        elements = edge.findall("lane")
        for lane in elements:
            lane_id = lane.get("id")
            shape = lane.get("shape")
            if not lane_id or not shape:
                continue
            points = tuple(tuple(map(float, point.split(",")[:2])) for point in shape.split())
            if len(points) >= 2:
                lanes.append(LaneShape(
                    lane_id, points,
                    # SUMO's net reader uses 3.2 m when width is omitted.
                    width=parse_float(lane.get("width"), 3.2),
                    index=int(lane.get("index", "0")),
                    edge_id=edge.get("id", ""),
                    lane_count=len(elements),
                    internal=edge.get("function") == "internal",
                ))
    return lanes


def scenario_output_dir(scenario: str, variant: str) -> Path:
    if scenario == "lane-change":
        return RESULTS_DIR / "lane_change" / variant
    return RESULTS_DIR / scenario / variant


def role_file(scenario: str) -> Path:
    filename = "roles_lane_change.json" if scenario == "lane-change" else "roles_merging.json"
    return SCENARIO_DIR / "animation" / filename


def comparison_file(scenario: str) -> Path:
    directory = "lane_change" if scenario == "lane-change" else scenario
    return RESULTS_DIR / directory / "comparison.json"


def require_capture_outputs(scenario: str) -> None:
    missing: list[Path] = []
    for variant in ("coordinated", "baseline"):
        output = scenario_output_dir(scenario, variant)
        for filename in ("fcd.xml", "lanechange.xml", "tripinfo.xml", "metrics.json", "events.json"):
            path = output / filename
            if not path.exists():
                missing.append(path)
    comparison = comparison_file(scenario)
    if not comparison.exists():
        missing.append(comparison)
    if missing:
        formatted = "\n".join(f"  {path}" for path in missing)
        raise FileNotFoundError(f"missing animation capture/analysis output:\n{formatted}")


def samples_in_window(series: Iterable[VehicleSeries], start: float, end: float) -> list[VehicleSample]:
    samples: list[VehicleSample] = []
    for vehicle in series:
        samples.extend(sample for sample in vehicle.samples if start <= sample.time <= end)
    return samples


def compute_viewport(
    vehicles: Iterable[VehicleSeries],
    lanes: Iterable[LaneShape],
    start: float,
    end: float,
    padding_m: float = 18.0,
) -> Viewport:
    samples = samples_in_window(vehicles, start, end)
    if not samples:
        raise ValueError("cannot compute viewport without FCD samples")
    min_x = min(sample.x for sample in samples) - padding_m
    max_x = max(sample.x for sample in samples) + padding_m
    min_y = min(sample.y for sample in samples) - padding_m
    max_y = max(sample.y for sample in samples) + padding_m

    # Include lane geometry that intersects the vehicle-focused window so road
    # context does not stop exactly at vehicle bounds.
    for lane in lanes:
        xs = [point[0] for point in lane.points]
        ys = [point[1] for point in lane.points]
        if max(xs) < min_x or min(xs) > max_x or max(ys) < min_y or min(ys) > max_y:
            continue
        min_x = min(min_x, min(xs) - 5.0)
        max_x = max(max_x, max(xs) + 5.0)
        min_y = min(min_y, min(ys) - 5.0)
        max_y = max(max_y, max(ys) + 5.0)

    return Viewport(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y)


def make_transform(viewport: Viewport, box: tuple[int, int, int, int]):
    left, top, right, bottom = box
    width = right - left
    height = bottom - top
    scale = min(width / viewport.width, height / viewport.height)
    x_pad = (width - viewport.width * scale) / 2.0
    y_pad = (height - viewport.height * scale) / 2.0

    def transform(x: float, y: float) -> tuple[float, float]:
        px = left + x_pad + (x - viewport.min_x) * scale
        py = bottom - y_pad - (y - viewport.min_y) * scale
        return px, py

    return transform, scale


def duration_until(
    samples: list[VehicleSample],
    time_s: float,
    predicate,
) -> float:
    if len(samples) < 2:
        return 0.0
    duration = 0.0
    for index, sample in enumerate(samples):
        if sample.time > time_s:
            break
        if not predicate(sample):
            continue
        if index + 1 < len(samples):
            dt = max(0.0, min(samples[index + 1].time, time_s) - sample.time)
        else:
            dt = 0.1
        duration += dt
    return duration


def lane_state(sample: VehicleSample | None) -> str:
    if sample is None or not sample.lane:
        return "lane n/a"
    if "_" not in sample.lane:
        return sample.lane
    edge, lane = sample.lane.rsplit("_", 1)
    return f"{edge} lane {lane}"


def vehicle_polygon(
    sample: VehicleSample,
    dimensions: VehicleDimensions,
) -> list[tuple[float, float]]:
    # FCD x/y is the center of the front bumper; the body extends behind it.
    # SUMO angle is measured clockwise from north. Convert to a world-space
    # direction vector with x east and y north.
    rad = math.radians(sample.angle)
    dx = math.sin(rad)
    dy = math.cos(rad)
    px = math.cos(rad)
    py = -math.sin(rad)
    half_w = dimensions.width / 2.0
    corners = [
        (0.0, half_w),
        (0.0, -half_w),
        (-dimensions.length, -half_w),
        (-dimensions.length, half_w),
    ]
    return [
        (
            sample.x + longitudinal * dx + lateral * px,
            sample.y + longitudinal * dy + lateral * py,
        )
        for longitudinal, lateral in corners
    ]


def draw_text_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    lines: list[str],
    font: ImageFont.ImageFont,
    fill: tuple[int, int, int] = (30, 35, 42),
    background: tuple[int, int, int] = (255, 255, 255),
    outline: tuple[int, int, int] = (210, 216, 224),
    padding: int = 6,
) -> tuple[int, int, int, int]:
    line_heights: list[int] = []
    max_width = 0
    for line in lines:
        bbox = draw.textbbox((0, 0), line, font=font)
        max_width = max(max_width, bbox[2] - bbox[0])
        line_heights.append(bbox[3] - bbox[1] + 3)
    width = max_width + padding * 2
    height = sum(line_heights) + padding * 2
    x, y = xy
    rect = (x, y, x + width, y + height)
    draw.rounded_rectangle(rect, radius=4, fill=background, outline=outline)
    cursor = y + padding
    for line, line_height in zip(lines, line_heights):
        draw.text((x + padding, cursor), line, fill=fill, font=font)
        cursor += line_height
    return rect


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def format_speed(speed: float | None) -> str:
    return "n/a" if speed is None else f"{speed:.1f} m/s"


def format_acceleration(acceleration: float | None) -> str:
    return "n/a" if acceleration is None else f"{acceleration:.1f} m/s²"


def save_gif(frames: list[Image.Image], path: Path, fps: int, fixed_colors=()) -> None:
    """Encode every frame with one palette and exact reserved role colors."""
    if not frames:
        raise ValueError("no frames to save")
    duration_ms = int(round(1000 / fps))
    fixed_colors = tuple(dict.fromkeys(fixed_colors))
    # Sample the sequence once for its neutral/UI colors, not independently per
    # frame. Reserved entries keep role fills exact despite changing traffic.
    samples = frames[::max(1, len(frames) // 12)][:12]
    sample = Image.new("RGB", (240, 135 * len(samples)))
    for i, frame in enumerate(samples):
        sample.paste(frame.resize((240, 135), Image.NEAREST), (0, i * 135))
    neutral_count = 128 - len(fixed_colors)
    adaptive = sample.quantize(colors=neutral_count)
    palette = Image.new("P", (1, 1))
    values = [channel for color in fixed_colors for channel in color]
    values += adaptive.getpalette()[:neutral_count * 3]
    values += [0] * (768 - len(values))
    palette.putpalette(values)
    paletted = [frame.quantize(palette=palette, dither=Image.NONE) for frame in frames]
    paletted[0].save(path, save_all=True, append_images=paletted[1:],
                    duration=duration_ms, loop=0, optimize=False, disposal=2)


def parse_junction_shapes(path: Path, excluded_ids: tuple[str, ...] = ()) -> tuple:
    """Read actual SUMO junction surface polygons, including merge connections."""
    return tuple(
        tuple(tuple(map(float, point.split(",")[:2])) for point in junction.get("shape", "").split())
        for junction in ET.parse(path).getroot().findall("junction")
        if junction.get("id") not in excluded_ids and len(junction.get("shape", "").split()) >= 3
    )


def offset_polyline(points, distance: float) -> list[tuple[float, float]]:
    """Offset to the left in world coordinates, using capped miter joins."""
    normals = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        normals.append((-dy / length, dx / length) if length else (0.0, 0.0))
    result = []
    for i, point in enumerate(points):
        before = normals[max(0, i - 1)]
        after = normals[min(i, len(normals) - 1)]
        nx, ny = before[0] + after[0], before[1] + after[1]
        length = math.hypot(nx, ny)
        if length < 1e-9:
            nx, ny = after
        else:
            nx, ny = nx / length, ny / length
        projection = nx * after[0] + ny * after[1]
        shift = distance / max(projection, 0.5)
        result.append((point[0] + nx * shift, point[1] + ny * shift))
    return result
