# Trajectory Planning and Conflict Checking

## 1. Purpose and Scope

This document describes how trajectory reference paths, local planning horizons, MCM trajectory coordinates, lane-change approximations, and conflict checking work in the current maneuver-coordination implementation.

The implementation deliberately uses several simulation-specific simplifications. These choices are part of the current prototype design and should not be treated as implementation bugs when they are used consistently inside the supplied scenarios.

This is not a general-purpose online global motion planner, arbitrary-map lane-geometry framework, or standards-oriented MCM coordinate encoder. The implementation is intended for the provided SUMO/OMNeT++ maneuver-coordination scenarios.

## 2. Terminology

### Global reference path

A global reference path is a pre-recorded sequence of absolute SUMO Cartesian lane-centre coordinates associated with a route or maneuver-relevant road section.

### Local planning horizon

A local planning horizon is a finite sequence of future trajectory points generated from the vehicle's current position and expected travelled distance.

In this implementation, "local" refers to the limited near-term planning horizon. It does not mean an ego-centred, vehicle-relative, or otherwise transformed coordinate frame.

### Global SUMO coordinate frame

The global SUMO coordinate frame is the Cartesian `x/y` coordinate system used by SUMO and returned by TraCI through `getPositionSumo()`. Coordinates are expressed in metres.

### Scenario-specific approximation

A scenario-specific approximation is a deliberately simplified method that is valid for the current scenario geometry but is not generally portable to arbitrary maps, arbitrary lane layouts, or external implementations.

## 3. Why Pre-recorded Reference Paths Are Used

The trajectory-planning algorithm requires road-aligned Cartesian lane information that would normally be supplied by a global path planner, a trajectory planner, or a route/lane geometry provider.

A complete online global planner is outside the scope of the current implementation. Instead, the relevant lane-centre coordinates for the maneuver areas were recorded in advance. A reference vehicle travelled at approximately `1 m/s`, producing approximately one recorded point per metre. The recorded coordinates are stored in `scenarios/artery-maneuver-coordination/coordinates_new_map.csv`.

The CSV contains route-specific columns such as `<routeId>_x` and `<routeId>_y`. For example, route-specific reference paths are selected by `TrajectoryCsv.cc` and then used by the trajectory-generation helpers.

This does not mean that SUMO provides no geometry. SUMO and TraCI can expose network and vehicle information depending on the API usage. The current application simply does not implement the general route/lane sampling layer needed to provide this planner with a complete road-aligned point sequence for arbitrary lanes, edges, routes, or maps.

## 4. Coordinate and Trajectory Data Flow

The implemented coordinate flow is:

1. Route-specific reference-path columns are loaded from the CSV.
2. The current vehicle position is obtained through `getPositionSumo()`.
3. The nearest recorded reference point is found using Cartesian distance.
4. Expected travelled distance is calculated from speed, time, and acceleration or deceleration.
5. Travelled distance is converted to a reference-path index offset.
6. Future `TrajPointMCM` points are copied from later global reference-path coordinates.
7. The resulting trajectory is serialized into the MCM.
8. Received trajectory points are decoded into the same global coordinate frame.
9. Ego and received trajectories are compared directly in conflict checking.

| Stage | Main file or function | Input | Output | Coordinate frame | Units |
|---|---|---|---|---|---|
| Load reference paths | `TrajectoryCsv.cc`, `loadTrajectoryCsv` | `coordinates_new_map.csv` columns | In-memory route coordinate columns | Global SUMO `x/y` | metres |
| Select route path | `TrajectoryCsv.cc`, `selectRouteReference` | Route ID and loaded CSV columns | Route-specific `x` and `y` vectors | Global SUMO `x/y` | metres |
| Read current position | `TrajectoryGeneration.cc`, `calculateRefTrajectory`; `McService.cc`, ego-context update paths | `getPositionSumo()` | Current vehicle position | Global SUMO `x/y` | metres |
| Match nearest point | `TrajectoryGeneration.cc`, `calc_nearest_index` | Current SUMO position and route reference path | Closest reference-path index | Global SUMO `x/y` comparison | metres |
| Generate local horizon | `TrajectoryGeneration.cc`, `calculateRefTrajectory`, `calculateExecuteTrajectory`, `calculateSecondReqTraj` | Current state, speed profile, route reference path | `std::vector<TrajPointMCM>` | Global SUMO `x/y` | metres, seconds |
| Select or queue maneuver trajectory | `McApplication.cc` | Ego trajectory, received snapshots, scenario state | Requested, offered, selected, or active trajectory | Global SUMO `x/y` | metres, seconds |
| Serialize MCM trajectory | `McService.cc`, `appendTrajectoryPoint` | `TrajPointMCM` | ASN.1 trajectory point fields | Rounded global SUMO `x/y` stored in delta-named fields | metres, centiseconds for time |
| Decode received MCM trajectory | `McService.cc`, `extractTrajectory` | ASN.1 trajectory point fields | `TrajPointMCM` | Global SUMO `x/y` by convention | metres, seconds |
| Check conflict | `TrajectoryConflict.cc` | Ego and received/generated trajectories | Conflict decision | Global SUMO `x/y` comparison | metres |
| Scenario constants | `McScenarioConfig.cc` | Scenario geometry assumptions | Lane-shift and merge constants | Global SUMO `x/y` | metres |

## 5. Global Reference Path Versus Local Planning Horizon

The full CSV coordinate sequence is the predefined global reference path. Only a finite subset of future points is used for a maneuver. This finite subset is the local planning horizon.

Both the full reference path and the generated local planning horizon remain in the global SUMO coordinate frame. No global-to-ego-local coordinate transformation is performed before planning, MCM serialization, MCM decoding, or conflict checking.

Conceptually:

```text
Global reference path:
P0, P1, P2, ..., P500

Current nearest point:
P210

Generated local horizon:
P215, P220, P225, ..., P260
```

The precise selected indices depend on the expected travelled distance, vehicle speed, acceleration or deceleration, and trajectory time step.

## 6. MCM Coordinate Convention

The current implementation stores rounded absolute SUMO `x/y` values in ASN.1 fields whose names suggest delta or relative positions, such as `deltaLongitudinalPosition` and `deltaLateralPosition`.

This is an intentional simulation-specific convention. It should not be described as an accidental coordinate-frame bug inside this repository's validation scenarios.

### Internal consistency

The sender and receiver use the same interpretation. The generated trajectory, serialized MCM trajectory, decoded received trajectory, and ego trajectory remain in the same global SUMO coordinate frame. Direct Cartesian comparison is therefore valid inside this implementation.

### Interoperability limitation

An external MCM implementation expecting true relative or delta values would interpret these fields differently. This coordinate convention must not be assumed to be generally interoperable.

A future standards-oriented implementation would need a defined reference point, relative-coordinate conversion, scaling rules, and corresponding decoding before exchanging trajectories with other implementations.

## 7. Lane-change Trajectory Approximation

Lane-change trajectory points may be constructed by shifting the global `x` coordinate of the reference path.

The current implementation uses two closely related values:

* approximately `+3.0 m` is used for some planned/requested trajectory paths;
* approximately `3.2 m` represents the measured lane width more accurately and is used by the execution movement over multiple simulation ticks.

This approximation works in the active highway lane-change scenario because:

* the relevant highway lanes are nearly parallel;
* the road runs mainly along the global `y` direction;
* the lateral direction is approximately the global `x` direction;
* adjacent lane centres are approximately `3.2 m` apart.

The shifted points remain absolute SUMO coordinates. This is not an ego-local coordinate transformation. The `3.0 m` planned-trajectory shift introduces a small lateral bias relative to the approximately `3.2 m` lane-centre distance, but it is acceptable as a scenario-specific approximation in the current implementation.

This fixed-axis lane shift is not generally valid for:

* curved road sections;
* roads rotated relative to the coordinate axes;
* changing lane widths;
* nonparallel lanes;
* opposite lane orientations;
* arbitrary SUMO maps.

A general solution would use actual adjacent-lane centreline geometry, a map provider, SUMO lane shapes, or a normal-vector offset calculated along the reference path. This document describes the current implementation and does not require such a general solution.

## 8. Spatial Resolution and Approximation Sources

The main spatial approximation sources are separate effects.

### Reference-path discretization

Reference points are spaced at approximately one metre. Nearest-point matching therefore produces a typical maximum deviation of approximately half the spacing.

This is an approximation, not necessarily a strict maximum for every endpoint, route transition, or discontinuity in the recorded data.

### Fixed lane-width shift

The planned shift of approximately `3.0 m` differs slightly from the measured lane separation of approximately `3.2 m`. This creates a small lateral bias in the generated adjacent-lane trajectory for the active highway lane-change scenario.

### MCM serialization rounding

Coordinates are rounded before serialization into the MCM trajectory fields. Each coordinate can differ by less than approximately `0.5 m` after rounding. The maximum two-dimensional displacement from rounding alone is less than approximately `0.71 m`.

These effects are distinct and should not simply be added together without a justified worst-case error model.

## 9. Conflict and Collision Checking

The active conflict checks compare trajectories in the global SUMO coordinate frame. Point distances are calculated directly from the Cartesian `x/y` values.

The implementation checks selected trajectory indices. Lane-change checks may include an additional lateral condition. Merging checks may focus on final or near-final points.

The active implementation does not use vehicle polygons, swept volumes, or continuous line intersection. The checks are simplified point-distance checks designed for the current validation scenarios.

Heading is not central to the active checks because they use point positions directly. Heading would become more important for bounding-box, polygon, or swept-volume collision models.

## 10. Temporal Alignment

The current checks generally compare equal or selected trajectory indices. This assumes that compared trajectories have compatible:

* start times;
* point counts;
* time steps;
* planning horizons.

Timestamps are available in the trajectory representation, but they are not always used for exact temporal interpolation in the active conflict checks.

This is a collision-model simplification, not a coordinate-frame error.

## 11. Assumptions and Limitations

The current implementation assumes:

* pre-recorded scenario geometry;
* no general online global path planner;
* a global absolute coordinate convention;
* nonstandard use of delta-named MCM fields;
* fixed-axis lane shifting for the active highway lane-change scenario;
* approximately one-metre path discretization;
* integer-coordinate serialization;
* index-based temporal alignment;
* a point-distance conflict model;
* limited arbitrary-map portability.

## 12. Future Generalization

Future generalization possibilities include:

* a route and lane geometry provider;
* automatic extraction of SUMO lane centrelines;
* adjacent-lane lookup;
* path-normal lane offsets;
* relative MCM coordinate encoding;
* timestamp-based interpolation;
* bounding-box or swept-volume collision checking.

These are possible extensions, not requirements for the current scenario implementation.
