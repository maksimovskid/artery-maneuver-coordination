# Animation Data and Rendering

This directory contains data-capture, analysis, and deterministic rendering
helpers for README-ready maneuver-coordination animations. Rendering is
data-driven from captured SUMO FCD, lane-change, tripinfo, and event outputs. It
does not record the live SUMO GUI or the desktop.

## Prerequisites

Build the project before capturing data:

```bash
cmake --build build --target core -j 4
```

The capture script uses the repository's existing `tools/run_artery.py`
launcher. Analysis and rendering use Python standard-library modules plus
Pillow. No pandas, GUI recorder, OpenCV, MoviePy, or external video encoder is
required.

## Supported Captures

```bash
python3 tools/animation/capture_scenario.py merging coordinated
python3 tools/animation/capture_scenario.py merging baseline
python3 tools/animation/capture_scenario.py lane-change coordinated
python3 tools/animation/capture_scenario.py lane-change baseline
```

Each capture defaults to run `0`, `Cmdenv`, `--sim-time-limit=30s`, and
`--cmdenv-express-mode=false`. Use `--time-limit` only when a longer metric
window is intentionally needed.

Equivalent Make targets capture and analyze the coordinated/baseline pairs:

```bash
make animation_data_merging
make animation_data_lane_change
make animation_data_all
```

## Analysis

Analyze one pair and write per-run metrics plus a comparison:

```bash
python3 tools/animation/analyze_scenario.py merging
python3 tools/animation/analyze_scenario.py lane-change
```

Analyze all supported captures:

```bash
python3 tools/animation/analyze_scenario.py all
```

## Rendering

Render final side-by-side GIF and PNG media from existing captured data:

```bash
python3 tools/animation/render_comparison.py merging
python3 tools/animation/render_comparison.py merging --view closeup
python3 tools/animation/render_comparison.py lane-change
python3 tools/animation/render_comparison.py lane-change --view closeup
python3 tools/animation/render_comparison.py all
```

Equivalent Make targets are available and do not rerun simulations:

```bash
make animation_render_merging
make animation_render_merging_closeup
make animation_render_lane_change
make animation_render_lane_change_closeup
make animation_render_all
```

The default timing is deterministic:

```text
Merging overview: 6.5 s to 26.5 s, 10 fps, 1x simulation time
Merging close-up: 7.0 s to 16.5 s, 10 fps, 1x simulation time
Lane overview:    11.5 s to 30.0 s, 10 fps, 1x simulation time
Lane close-up:    11.5 s to 20.0 s, 10 fps, 1x simulation time
```

Useful optional renderer arguments:

```text
--fps
--width
--height
--start
--end
--view overview
--view closeup
--output
--keep-frames
```

The default output directory is:

```text
docs/media/
```

Generated media:

```text
docs/media/merging-comparison.gif
docs/media/merging-comparison.png
docs/media/merging-comparison-closeup.gif
docs/media/merging-comparison-closeup.png
docs/media/lane-change-comparison.gif
docs/media/lane-change-comparison.png
docs/media/lane-change-comparison-closeup.gif
docs/media/lane-change-comparison-closeup.png
```

Temporary frames are written only below
`scenarios/artery-maneuver-coordination/results_animation/tmp_frames/` when
`--keep-frames` is used. That generated tree is ignored by Git.

The root README embeds the close-up merging GIF because it shows the selected
RV/CV gap formation and merge movement more clearly. The wider merging overview
is kept as a linked repository asset for scenario context. Both merging views use
a fixed synchronized viewport shared by the baseline and coordinated panels.

The root README also embeds the close-up lane-change GIF because it focuses on
the emergency vehicle, follower RV, target-lane CVs, coordinated lane-change
completion, and baseline heavy braking. The wider lane-change overview remains
available as a linked repository asset to show the later delayed baseline lane
change.

## Output Structure

Generated files are written below:

```text
scenarios/artery-maneuver-coordination/results_animation/
    merging/
        coordinated/
        baseline/
    lane_change/
        coordinated/
        baseline/
```

Each run directory contains:

```text
fcd.xml
lanechange.xml
tripinfo.xml
simulation.log
events.json
metrics.json
metrics.csv
```

Pair-level comparison files are written to:

```text
results_animation/merging/comparison.json
results_animation/merging/comparison.md
results_animation/lane_change/comparison.json
results_animation/lane_change/comparison.md
```

The generated `results_animation` tree is ignored by Git.

## Metric Definitions

General vehicle metrics use SUMO FCD samples. Stopped duration is time with
speed below `0.1 m/s`. Low-speed duration is time with speed below `2.0 m/s`.
Distance travelled is computed from consecutive FCD centre positions.

Merging metrics focus on `car_ml1_1` and derive merge-entry timing from FCD lane
IDs and the configured merge point. Waiting time and time loss come from SUMO
tripinfo when available.

Emergency lane-change metrics focus on `car_hl0_Emergency` and the follower RV.
The coordinated follower is verified from MCM logs when possible; otherwise the
role map uses `car_hl1_1`. Proximity is reported as centre-to-centre distance,
not physical bumper clearance.

## Role Highlighting

Merging highlights:

```text
RV:  car_ml1_1
CV1: car_hl0_1
CV2: car_hl0_2
```

Emergency lane-change highlights:

```text
Emergency: car_hl0_Emergency
RV:        car_hl1_1
CV1:       car_hl2_1
CV2:       car_hl2_2
```

Other vehicles remain visible in a secondary style when present in FCD.

## Limitations

The generated media represents run `0`, seed `10`, and the validated 30-second
captures. A 30-second capture may include unfinished trips; tripinfo uses SUMO's
unfinished-trip output where supported. Vehicle rectangles use SUMO vehicle
length when available and a documented visual width fallback when width is not
provided by the vehicle type.

The animations show one controlled run. They should not be described as a
multi-seed statistical result.
