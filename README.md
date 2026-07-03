![Artery V2X Simulation Framework](https://raw.githubusercontent.com/riebl/artery/master/logo.png)

# Artery V2X Simulation Framework — Maneuver Coordination Work in Progress

## Project Status

This repository is a **work-in-progress** Artery-based implementation related to V2X maneuver coordination. It is currently being reorganized and restructured from earlier research code into a more maintainable open-source implementation.

The current code base contains working validation scenarios for cooperative merging and safety-critical lane changes. However, this repository should not yet be treated as a finished, stable, or fully validated software release. Interfaces, scenario organization, constants, and documentation may still change. The scenarios documented below are current validation targets, not a complete benchmark suite.

## Overview

Artery enables V2X simulations based on ETSI ITS-G5 protocols such as GeoNetworking and BTP. Single vehicles can be equipped with multiple ITS-G5 services through Artery's middleware, which also provides common Facilities for these services.

This repository extends Artery with a research prototype for V2X-based maneuver coordination between connected and automated vehicles. The implementation focuses on trajectory-based negotiation, priority-aware cooperation decisions, and SUMO/TraCI-based execution control for cooperative merging and safety-critical lane-change scenarios.

The simulation stack uses Artery together with OMNeT++, INET, Vanetza, SUMO, and TraCI. The maneuver-coordination work is being refactored into clearer components while preserving the behavior of the currently validated scenarios.

## Terminology

The current maneuver-coordination scenarios distinguish between the following vehicle roles:

* **Requesting Vehicle (RV):** The vehicle that initiates a maneuver coordination request, for example a merging vehicle or a vehicle that needs to change lanes because of an emergency ahead.
* **Cooperating Vehicle (CV):** A connected and automated vehicle that receives a request and may adapt its trajectory to support the RV's maneuver.
* **Non-Cooperating Vehicle (NCV):** A background or blocking vehicle that is part of the traffic scenario but does not participate in the MCM negotiation flow.

In the current validation scenarios, RVs send Request messages, CVs respond with Offer/Accept messages depending on the negotiation phase, and NCVs are used to model surrounding traffic or blocking vehicles without active cooperation.

## What This Repository Adds

This repository adds a Maneuver Coordination Service (MCS) / Maneuver Coordination Message (MCM)-style research prototype centered around `McService` and `McApplication`. `McService` integrates maneuver coordination into Artery's middleware service architecture, while `McApplication` contains the main Requesting Vehicle (RV) / Cooperating Vehicle (CV) negotiation, trajectory, priority, retry, and execution-control logic.

`McService` connects the application logic to Artery's service lifecycle, middleware, packet handling, and scheduling. The implementation includes message generation and reception for Request, Offer, Confirm, Accept, Execute, Complete, and Cancel handling.

The current research prototype supports one-CV and two-CV negotiation flows, Medium-priority cooperative merging validation, and High-priority emergency lane-change validation. It also contains dynamic trajectory/gap-based target selection, CV cooperation decision and feasibility checks, retry and timeout handling, and SUMO execution control through TraCI.

Communication/QoS configuration hooks from the earlier research implementation are being reintroduced in a cleaner form. These hooks cover intent MCM generation conditions, coordination MCM generation conditions, DCC profile mapping, adaptive generation-rule flags, and future congested-scenario validation. This work is staged; the presence of a configuration option does not imply that the full old behavior has already been reimplemented.

Vehicle IDs shown in this README are validation expectations for the current route files. The application logic should not rely on hard-coded vehicle IDs or station IDs. The code is still being cleaned, documented, and reorganized.

## Main Source Files

* `src/artery/application/McService.*`

  * Artery middleware service integration point for maneuver coordination.
  * Owns the service-level connection between Artery middleware and the MCM application logic.
  * Connects `McApplication` to the Artery service lifecycle, packet generation/reception, middleware callbacks, and scheduling.
  * Handles service-facing MCM packet flow while keeping the main maneuver behavior in `McApplication`.
  * Owns the staged MCM generation/QoS configuration hooks, including intent/coordination triggering policy names and DCC profile selection.

* `src/artery/application/McService.ned`

  * Defines the simulation module/service used by the middleware configuration.
  * Exposes the Maneuver Coordination Service as an Artery/OMNeT++ module that can be configured in scenarios.

* `src/artery/application/Asn1PacketVisitor.h`

  * Helper for inspecting and accessing ASN.1 encoded MCM packet content.
  * Used by packet-handling code that needs to visit or extract encoded message structures.

* `src/artery/application/mcm/McApplication.*`

  * Main MCM application logic.
  * Message generation and reception.
  * RV/CV negotiation state.
  * Request triggers for merging and safety-critical lane changes.
  * Retry and timeout handling.
  * Execute, Complete, and Cancel handling.
  * SUMO/TraCI control hooks.

* `src/artery/application/mcm/McScenarioConfig.*`

  * Scenario and validation constants.
  * Route IDs, emergency vehicle ID, scenario speeds/timing, merge thresholds, and emergency thresholds.
  * Old-implementation carryovers that are documented explicitly.
  * Constants that may later move to `omnetpp.ini`.

* `src/artery/application/mcm/TrajectoryPlanner.*`

  * Prerecorded trajectory support.
  * Trajectory helper functions and trajectory manipulation.
  * Feasibility/cost support.
  * Conflict and gap-related helpers.

* `src/artery/application/mcm/TrajectoryEnvironmentState.*`

  * Latest received MCM snapshots.
  * Station, route, and trajectory data used by environment-aware decisions.
  * Dynamic target selection support.

* `src/artery/application/mcm/TrajectoryEnvironment.*`, `TrajectoryConflict.*`, `TrajectoryCsv.*`, and `TrajectoryGeneration.*`

  * Supporting trajectory/environment helpers used by the maneuver-coordination implementation.

## Message and Negotiation Flow

For two-CV cooperative merging, the current high-level flow is:

```text
Request -> Offer(s) -> Confirm -> Accept(s) -> Execute -> Complete
```

The Requesting Vehicle (RV) sends a Request. Cooperating Vehicles (CVs) evaluate trajectory feasibility and cooperation cost. CVs send Offer messages, the RV sends Confirm after receiving the required Offer messages, CVs send Accept, and the RV sends Execute. Participants reset after Complete. Cancel and timeout paths return participants to intent sharing or an idle coordination state.

One-CV and two-CV flows are handled separately. Retry and timeout handling exists for the Request, Confirm, Offer, and Accept phases. This is currently a research implementation, and the message handling is still being refactored and documented.

## Validation Scenarios

The repository currently uses two split validation scenarios. These are correctness-validation targets for the present implementation, not final benchmark scenarios.

### Merging Scenario

Config:

```text
envmod-19CAVs-merging
```

Purpose:

* Medium-priority cooperative merging validation.

SUMO config:

```text
routes/test_19CAVs_merging.sumocfg
```

Loaded route files:

* `routes/test/merging_lane_1.rou.xml`
* `routes/test/highway_lane_0.rou.xml`

Expected behavior:

* Three sequential merging maneuvers.
* Merging vehicles act as RVs and selected highway-lane vehicles act as CVs.
* Dynamic target/gap selection.
* No emergency vehicle.
* No emergency broadcast.

For the current validation route, the expected target pairs are:

```text
car_ml1_1 -> car_hl0_1 / car_hl0_2
car_ml1_2 -> car_hl0_3 / car_hl0_4
car_ml1_3 -> car_hl0_5 / car_hl0_6
```

These pairs are validation expectations for the current route timing. They are not hard-coded target assignments.

### Emergency Lane-Change Scenario

Config:

```text
envmod-19CAVs-emergency-lane-change
```

Purpose:

* High-priority safety-critical lane-change validation.

SUMO config:

```text
routes/test_19CAVs_emergency_lane_change.sumocfg
```

Loaded route files:

* `routes/test/highway_lane_1.rou.xml`
* `routes/test/highway_lane_2.rou.xml`
* `routes/test/highway_lane_0_right_ncv.rou.xml`

Expected behavior:

* Emergency vehicle brakes and broadcasts emergency MCMs.
* Following lane-1 vehicles trigger High-priority lane-change negotiation.
* Lane-1 vehicles affected by the emergency act as RVs and selected lane-2 vehicles act as CVs.
* Cooperating CVs are selected from lane 2.
* Right-lane Non-Cooperating Vehicles (NCVs) act as background blockers / non-cooperative vehicles.
* No merging vehicles are present.

For the current validation route, the expected target pairs are:

```text
car_hl1_1 -> car_hl2_1 / car_hl2_2
car_hl1_2 -> car_hl2_3 / car_hl2_4
car_hl1_3 -> car_hl2_5 / car_hl2_6
```

Expected emergency broadcast count:

```text
10 Hz for 15 s = 150 emergency MCMs in the 30 s validation run
```

GUI configs are also available:

* `envmod-19CAVs-merging-gui`
* `envmod-19CAVs-emergency-lane-change-gui`

## Build

Build from the repository root:

```bash
cd artery-maneuver-coordination
cmake --build build
```

The scenario-specific run target may also be available from the build directory:

```bash
cd artery-maneuver-coordination/build
make run_artery_maneuver_coordination
```

This README does not duplicate Artery installation instructions. See the original Artery documentation linked below for general framework setup.

## Running the Validation Scenarios

Headless merging:

```bash
cd artery-maneuver-coordination

tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-merging -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

Headless emergency lane-change:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-emergency-lane-change -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

GUI merging:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Qtenv -c envmod-19CAVs-merging-gui -r 0
```

GUI emergency lane-change:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Qtenv -c envmod-19CAVs-emergency-lane-change-gui -r 0
```

## Useful Validation Checks

The commands below are lightweight regression checks for the current work-in-progress state.

Merging target pairs:

```bash
grep -E 'car_ml1_1.*SEND Request|car_ml1_2.*SEND Request|car_ml1_3.*SEND Request' /tmp/mcm-merging-final-30s.log
```

No emergency logs in merging:

```bash
grep '\[MCM-EMERGENCY\]' /tmp/mcm-merging-final-30s.log | wc -l
```

Emergency broadcast count:

```bash
grep '\[MCM-EMERGENCY\].*queue-emergency-execution-mcm' /tmp/mcm-emergency-final-30s.log | wc -l
```

No merging vehicles in emergency:

```bash
grep 'car_ml1_' /tmp/mcm-emergency-final-30s.log | wc -l
```

Error scan:

```bash
grep -E '<!> Error|Segmentation fault|core dumped|what\(\)|Teleporting vehicle|collision with vehicle|peer shutdown|unterminated comment|input ended' /tmp/mcm-*.log
```

## Publications and Research Background

Related peer-reviewed publications and research outputs are available through the author's Google Scholar and ResearchGate profiles. These publications provide the research background for the implemented maneuver coordination service, including priority-based cooperation, negotiation patterns, V2X channel-load-aware MCM handling, and scenario-based evaluation.

* Google Scholar: https://scholar.google.com/citations?user=gCw5sAcAAAAJ&hl=en
* ResearchGate: https://www.researchgate.net/profile/Daniel-Maksimovski

## Notes and Limitations

* This repository is work in progress.
* It is not a finished or fully validated release.
* Scenario constants are documented in `McScenarioConfig`.
* Some constants may later move to `omnetpp.ini`.
* The split scenarios are current correctness-validation targets.
* The combined 19-CAV scenario is not the primary correctness validation target because mixing merging and emergency lane-change use cases can create SUMO side effects.
* GUI/SUMO shutdown behavior may differ from headless Cmdenv; headless validation is the main regression check.
* More refactoring and documentation are ongoing.
* Communication/QoS options for DCC profile selection and adaptive generation are being restored incrementally; defaults preserve the current validation behavior unless explicitly changed.

## Cleanup

Generated SUMO files should not be committed:

```bash
rm -f scenarios/artery-maneuver-coordination/results_sumo/test/simulation.statistic.xml
rm -f scenarios/artery-maneuver-coordination/results_sumo/test/simulation.tripinfo.xml
```

## Original Artery Documentation

Artery started as an extension of the [Veins framework](http://veins.car2x.org) but can be used independently nowadays. Please refer to the Artery documentation for installation and general framework usage.

The Artery documentation website is available at [artery.v2x-research.eu](http://artery.v2x-research.eu). The [install instructions](http://artery.v2x-research.eu/install/) previously found in the original Artery README have also been moved to that website. If you want to build the website yourself, see the [mkdocs guide](http://artery.v2x-research.eu/mkdocs).
