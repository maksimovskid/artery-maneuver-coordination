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

Canonical media uses the centered titles **Highway Merging** and **Lane Change**
and a fixed synchronized camera shared by baseline and coordinated panels.
Rendering reads existing captures; it does not rerun or modify the simulation.

| Scenario | Simulation interval | Playback | Frames | Frame rate | Total playback | Final hold |
| --- | --- | --- | --- | --- | --- | --- |
| Highway Merging | 3.7–21.4 s | 2× | 188 | 20 fps | 9.4 s | 0.5 s |
| Lane Change | 11.5–18.0 s | 1.5× | 81 | 15 fps average | 5.4 s | 1.0 s |

The centered captions are "2 simulation seconds = 1 video second" and
"1.5 simulation seconds = 1 video second", respectively. Both retain 0.1 s
simulation samples. Lane Change uses 60/70 ms GIF delays to preserve its
average frame rate accurately.

Highway Merging shows RV1's Request → Offers → Confirm → Accepts exchange
with CV1/CV2, followed by "RV1 merging" and "RV1 merge completed".
RV2/RV3 retain simple gap-cooperation and merge-completion statuses; their
handshakes never interrupt RV1's physical merge.

Lane Change shows RV1 with CV1/CV2 and RV2 with CV3/CV4. Two-line boxes
show the RV's Request/Confirm alongside the CVs' Offers/Accepts. During
execution, accepted participants remain as context beneath "Lane change";
completion reads "Lane change done". Baseline captions imply no cooperation.

Participant identities and first message sends come from captured MCM logs.
Transitions use the enclosing OMNeT++ event timestamps because some diagnostic
`simTime` fields cache an earlier tick. A reply is shown only after its logged
send; partial replies name only CVs that have sent that message. Short phases
are never stretched for readability. Lane-change execution/completion use
`MCM-LC-EXEC` events. Merge completion uses whole-body highway establishment
from FCD, preserving the validated completion timing. Overlay anchors stay in
the existing road-free areas. Lane-change panel headings move 2 px left to
clear a passing vehicle label; status-box anchors are unchanged.

Render the canonical outputs from the validated captures:

```bash
python3 tools/animation/render_comparison.py merging --view presentation-road-detail \
  --capture-root scenarios/artery-maneuver-coordination/results_animation/merging_final_spacing/earlier03-front-pos12 \
  --start 3.7 --end 21.4 --fps 20 --playback-speed 2 --hold-seconds 0.5 --poster-time 9.3
python3 tools/animation/render_comparison.py lane-change --view presentation-road-detail \
  --start 11.5 --end 18.0 --fps 15 --playback-speed 1.5 --hold-seconds 1.0 --poster-time 12.7
```

Only these canonical files belong in `docs/media/`:

```text
docs/media/merging-maneuver-comparison.gif
docs/media/merging-maneuver-comparison.png
docs/media/lane-change-maneuver-comparison.gif
docs/media/lane-change-maneuver-comparison.png
```

Use `--output` with a separate directory for experiments. Optional
`--capture-root`, `--start`, `--end`, `--fps`, `--playback-speed`,
`--hold-seconds`, `--poster-time`, `--width`, and `--height` control rendering.
Use the exact commands above to reproduce the final media.

Temporary frames are written below
`scenarios/artery-maneuver-coordination/results_animation/tmp_frames/` when
`--keep-frames` is used. That generated tree is ignored by Git.

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

Lane-change metrics focus on `car_hl0_Emergency` and the follower RV.
The coordinated follower is verified from MCM logs when possible; otherwise the
role map uses `car_hl1_1`. Proximity is reported as centre-to-centre distance,
not physical bumper clearance.

## Role Highlighting

Presentation labels follow real Request/Accept participants in departure order
and remain fixed in both panels:

| Scenario | RV1 / CV1 + CV2 | RV2 / CV3 + CV4 | RV3 / CV5 + CV6 |
| --- | --- | --- | --- |
| Highway Merging | `car_ml1_1` / `car_hl0_1`, `car_hl0_2` | `car_ml1_2` / `car_hl0_3`, `car_hl0_4` | `car_ml1_3` / `car_hl0_5`, `car_hl0_6` |
| Lane Change | `car_hl1_1` / `car_hl2_1`, `car_hl2_2` | `car_hl1_2` / `car_hl2_3`, `car_hl2_4` | `car_hl1_3` / `car_hl2_5`, `car_hl2_6` |

Lane Change also highlights `car_hl0_Emergency` as **Emergency**.
Other SUMO traffic remains visible in a secondary style. Detailed statuses are
limited to merging RV1 and lane-change RV1/RV2, regardless of other highlighted
vehicles.

## Limitations

The generated media represents run `0`, seed `10`, and the validated 30-second
captures. A 30-second capture may include unfinished trips; tripinfo uses SUMO's
unfinished-trip output where supported. Vehicle rectangles use SUMO vehicle
length when available and a documented visual width fallback when width is not
provided by the vehicle type.

The animations show one controlled run. They should not be described as a
multi-seed statistical result.
