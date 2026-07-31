# Animation Data Capture

This directory contains data-capture and analysis helpers for future
README-ready maneuver-coordination animations. The scripts do not render GIFs,
videos, or polished plots.

## Prerequisites

Build the project before capturing data:

```bash
cmake --build build --target core -j 4
```

The scripts use the repository's existing `tools/run_artery.py` launcher and
Python standard-library modules. No pandas, plotting package, video encoder, or
GUI is required for this data step.

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

## Limitations

The analysis is intended to validate whether future animation claims are
supported. It does not create visual media. A 30-second capture may include
unfinished trips; tripinfo uses SUMO's unfinished-trip output where supported.
Any future animation should map simulation time to playback time directly, with
one simulation second equal to one video second unless a clearly labelled
slow-motion segment is added.
