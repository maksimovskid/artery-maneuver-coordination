# MCM User and Developer Guide

This guide describes the maneuver-coordination implementation in this repository for users who want to run the current scenarios and developers who want to extend them. The repository is research-oriented and under active restructuring. It contains working validation paths for cooperative merging and safety-critical lane-change experiments, but it should not be treated as a finished, stable, or fully validated product.

## 1. Overview

This repository extends Artery for V2X maneuver-coordination experiments. The implementation focuses on Maneuver Coordination Messages (MCMs), a Maneuver Coordination Service (MCS), negotiation between Requesting Vehicles (RVs) and Cooperating Vehicles (CVs), and measurement of communication and traffic effects in SUMO/OMNeT++ simulations.

The current implementation supports two main validation targets:

* Medium-priority cooperative merging.
* High-priority emergency lane-change coordination.

The code is organized around an Artery middleware service, `McService`, and an application-level state machine, `McApplication`. The service handles Artery integration, packet generation/reception, scheduling, DCC/QoS behavior, and OMNeT++ statistics. The application handles RV/CV roles, MCM negotiation state, trajectory selection, cooperation decisions, retry/timeout handling, and SUMO/TraCI execution hooks.

## 2. Key Terminology

**MCM:** Maneuver Coordination Message. In this repository, MCMs carry intent, negotiation, or execution information for coordinated maneuvers.

**MCS / Maneuver Coordination Service:** The Artery middleware service that creates, sends, receives, classifies, and records MCMs. It is implemented mainly in `src/artery/application/McService.*` and declared as an OMNeT++ service module in `src/artery/application/McService.ned`.

**RV / Requesting Vehicle:** The vehicle that initiates a coordination request, such as a merging vehicle or a vehicle that needs to change lanes because of an emergency vehicle ahead.

**CV / Cooperating Vehicle:** A vehicle that receives an RV request and may adapt speed, acceleration, deceleration, time gap, or trajectory to support the RV.

**NCV / Non-Cooperating Vehicle:** A vehicle present in the traffic scenario but not participating in the active MCM negotiation flow.

**Intent MCM:** An MCM without maneuver negotiation or execution containers. It communicates regular vehicle state and planned trajectory information.

**Negotiation MCM:** An MCM with a maneuver-negotiation container. The current flow uses Request, Offer, Confirm, Accept, Reject, Cancel, and related subtypes.

**Execution MCM:** An MCM with a maneuver-execution container. It is used after negotiation to coordinate or signal execution state.

**Emergency MCM:** In the current emergency lane-change validation path, the emergency vehicle broadcasts execution-container Abort messages with EmergencyPriority.

**QoS / DCC:** Communication-quality and Decentralized Congestion Control behavior. `McService` exposes configuration hooks for DCC restriction, DCC profile mapping, fixed-rate generation, and adaptive intent-frequency experiments.

**CBR / Channel Busy Ratio:** A radio-channel load measure. The helper reports `ChannelLoad` and the MCM service emits `coopCBR` from local channel-load input.

**PER / Packet Error Rate:** Packet error rate from radio statistics, reported as `packetErrorRate` in OMNeT++ results.

**Cooperative Age of Information:** `CoopVehicleAgeOfInformation`, measured for coordination-relevant MCMs received by a listed cooperating participant. It is receive time minus MCM generation timestamp.

**negotiationTime:** RV-side duration from the first Request to the final required Accept, with a current fallback that can treat a missing expected CV Accept as observed Execute evidence.

**Cooperation cost:** A planner cost used to summarize how much a CV adapts. It combines speed change, acceleration/deceleration, lane-change cost, time-gap reduction, and currently zero TTC cost.

**Trajectory type / trajectory category:** The planner returns raw trajectory types that are recorded as OMNeT++ counters. The CSV helper adds contiguous public `trajectory_category` labels from 0 to 5 while preserving raw metric names.

## 3. Architecture

The current MCM implementation is split across service integration, application state, trajectory support, scenarios, and result-analysis tooling.

| Path | Responsibility |
| --- | --- |
| `src/artery/application/McService.*` | Artery middleware service integration, MCM packet generation/reception, scheduling, DCC/QoS settings, OMNeT++ signals/statistics, and handoff to `McApplication`. |
| `src/artery/application/McService.ned` | OMNeT++ service declaration and configurable parameters for MCM generation, DCC behavior, retry limits, timestamp source, prerecorded trajectories, and statistics. |
| `src/artery/application/Asn1PacketVisitor.h` | ASN.1 packet visitor helper used by packet-handling code. |
| `src/artery/application/mcm/McApplication.*` | Maneuver-coordination state machine, RV/CV behavior, request triggers, retry/timeout handling, cooperation decisions, execution-control hooks, and planner metric emission. |
| `src/artery/application/mcm/McScenarioConfig.*` | Current scenario constants: route IDs, emergency-vehicle ID, calibrated timing/speed values, merge geometry, request trajectory horizon, and cache limits. |
| `src/artery/application/mcm/TrajectoryPlanner.*` | Trajectory feasibility, conflict checks, cooperation-cost calculation, and planner category selection. |
| `src/artery/application/mcm/TrajectoryEnvironment*.*` | Environment and received-message helper functions used by trajectory decisions. |
| `src/artery/application/mcm/TrajectoryConflict.*` | Trajectory conflict and collision-related checks. |
| `src/artery/application/mcm/TrajectoryCsv.*` | CSV trajectory support for prerecorded intent trajectories. |
| `src/artery/application/mcm/TrajectoryGeneration.*` | Reference and execution trajectory generation. |
| `tools/analyze_mcm_qos_results.py` | OMNeT++ `.sca` result summarizer for MCM/QoS metrics. |
| `scenarios/artery-maneuver-coordination/omnetpp.ini` | Scenario configurations for 19-CAV validation and 200/500-CAV QoS experiments. |

`McService` owns the Artery-facing service lifecycle. It reads NED parameters, builds ASN.1 MCMs, fills the common MCM containers, selects DCC traffic class/profile behavior, sends packets through the middleware, receives packets, extracts MCM snapshots, emits OMNeT++ signals/statistics, and calls into `McApplication`.

`McApplication` owns the maneuver-coordination behavior. It tracks whether the vehicle is in Intention Sharing, Maneuver Negotiation, or Maneuver Execution mode; whether it is acting as RV, CV, NCV, or emergency vehicle; which MCM subtype should be sent next; and which execution control action is currently selected.

`TrajectoryPlanner` and the helper files provide the planner support. They calculate reference trajectories, check conflict/gap conditions, evaluate speed/acceleration/deceleration/lane-change options, compute cooperation cost, and classify planner outputs for measurement.

Scenario files under `scenarios/artery-maneuver-coordination/` provide SUMO routes, SUMO configs, OMNeT++ configs, services, sensors, and prerecorded trajectory data.

## 4. MCM Containers and Flow

The application uses three main operation modes:

* **Intention Sharing mode:** default mode. Vehicles send regular intent MCMs with basic state and planned trajectory information.
* **Maneuver Negotiation mode:** active negotiation mode. Vehicles exchange Request, Offer, Confirm, Accept, Reject, and Cancel-style negotiation MCMs.
* **Maneuver Execution mode:** active execution mode. Vehicles send repeated Execute messages or emergency execution messages and monitor maneuver progress.

The normal high-level negotiation flow is:

```text
Request -> Offer -> Confirm -> Accept -> Execute -> Complete/Cancel
```

The current implementation supports one-CV and two-CV paths. In the two-CV merging path, the RV sends a Request to two selected CVs, waits for both Offers, sends Confirm, waits for both Accepts, and then enters execution. In the one-CV high-priority lane-change path, the RV can receive an Accept directly after Request.

The RV and CV roles are separated:

* The RV identifies that coordination is needed, selects target CVs, queues a Request, tracks Offer/Accept state, handles Rejects and timeouts, and queues Execute.
* A CV evaluates whether the incoming Request targets it, checks trajectory feasibility, computes planner cost, sends Offer or Accept/Reject depending on phase, and applies selected speed-control behavior where implemented.
* NCVs continue normal intent sharing and traffic behavior without joining the negotiation.

Retry and timeout handling exists for the active Request, Confirm, Offer, and Accept phases. The retry interval and negotiation limits are exposed through `McService.ned` and `omnetpp.ini` parameters such as `negotiationRetryInterval`, `negotiationLimitMerging`, and `negotiationLimitLaneChange`.

`negotiationTime` is measured on the RV side. It starts with the first Request and ends when the required Accept evidence is complete. If a required Accept is not observed but an expected CV's Execute is later observed, the implementation can count that Execute as acceptance evidence for completing the measurement.

Coordination message generation is rate-limited. The default validation setup uses the same 0.1 s cadence as the middleware update interval and MCM fixed-rate behavior. `timeGenMcm = "CurrentTime"` is available for timestamp-sensitive MCMs; otherwise the service can use the data-provider timestamp source.

Emergency behavior is separate from the normal Request/Offer/Confirm/Accept flow. The configured emergency vehicle broadcasts emergency execution MCMs at 10 Hz for 15 s in the current emergency lane-change validation run.

The high-priority lane-change RV path can queue one second Request after a Reject. The RV uses the rejecting CV's latest planned trajectory and the second-request planner helper to build a new requested trajectory with a new request ID, then sends it through the normal rate-limited MCM generation path. Full generalized cascading behavior remains limited unless a scenario explicitly validates it.

## 5. Validation Scenarios

The current correctness-validation targets are split into two 19-CAV scenarios.

### `envmod-19CAVs-merging`

Purpose: Medium-priority cooperative merging validation.

Key files:

* OMNeT++ config: `scenarios/artery-maneuver-coordination/omnetpp.ini`
* SUMO config: `scenarios/artery-maneuver-coordination/routes/test_19CAVs_merging.sumocfg`
* Route files:
  * `scenarios/artery-maneuver-coordination/routes/test/merging_lane_1.rou.xml`
  * `scenarios/artery-maneuver-coordination/routes/test/highway_lane_0.rou.xml`

Expected purpose:

* Merging vehicles act as RVs.
* Highway-lane vehicles selected by current gap/trajectory snapshots act as CVs.
* Target pairs are validation expectations from the current route timing, not fixed protocol assignments.
* The scenario validates medium-priority cooperative merging and CV speed adaptation behavior.

### `envmod-19CAVs-emergency-lane-change`

Purpose: High-priority safety-critical lane-change validation.

Key files:

* OMNeT++ config: `scenarios/artery-maneuver-coordination/omnetpp.ini`
* SUMO config: `scenarios/artery-maneuver-coordination/routes/test_19CAVs_emergency_lane_change.sumocfg`
* Route files:
  * `scenarios/artery-maneuver-coordination/routes/test/highway_lane_1.rou.xml`
  * `scenarios/artery-maneuver-coordination/routes/test/highway_lane_2.rou.xml`
  * `scenarios/artery-maneuver-coordination/routes/test/highway_lane_0_right_ncv.rou.xml`

Expected purpose:

* The configured emergency vehicle brakes and broadcasts emergency MCMs.
* Following lane-1 vehicles arm high-priority lane-change negotiation.
* Lane-2 vehicles can act as CVs.
* Right-lane NCVs provide non-cooperating/background traffic.

### `envmod-19CAVs-second-request-smoke`

Purpose: Focused second-request validation.

This optional smoke-test config extends `envmod-19CAVs-emergency-lane-change` and enables a disabled-by-default validation hook. One selected CV rejects its first high-priority lane-change Request, allowing the RV to queue and send a second Request through the normal rate-limited MCM generation path. Use it to check the second-request state machine; do not treat it as a baseline traffic or communication result configuration.
* No merging vehicles should appear in this scenario.

The emergency broadcast validation expectation for a 30 s run is 150 queued emergency execution MCMs, from 10 Hz over a 15 s emergency broadcast window.

### Congested QoS Configs

`scenarios/artery-maneuver-coordination/omnetpp.ini` also defines 200-CAV and 500-CAV QoS experiment configs:

* `envmod-200CAVs-qos-baseline-freespace`
* `envmod-200CAVs-qos-adapt-intent-freespace`
* `envmod-200CAVs-qos-mco-1hz-freespace`
* `envmod-200CAVs-qos-dcc-profiles-freespace`
* `envmod-500CAVs-qos-baseline-freespace`
* `envmod-500CAVs-qos-adapt-intent-freespace`
* `envmod-500CAVs-qos-mco-1hz-freespace`
* `envmod-500CAVs-qos-dcc-profiles-freespace`

These configs use Free Space path loss to exercise higher channel-load conditions and compare baseline, adaptive Intent, MCO-specific 1 Hz Intent reduction, and DCC-profile mapping settings.

The 19 designated maneuver-coordination vehicles remain the intended coordination participants. Additional CAVs are background/load vehicles. They may generate regular Intent MCMs and, with the current service list, CAMs. They are communication-load traffic and should not be interpreted as active negotiation participants.

## 6. How to Run Scenarios

Build from the repository root:

```bash
cmake --build build
```

Run 19-CAV merging:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-merging -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

Run emergency lane-change:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-emergency-lane-change -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

GUI configs are available:

* `envmod-19CAVs-merging-gui`
* `envmod-19CAVs-emergency-lane-change-gui`

GUI/SUMO shutdown messages can occur when closing the GUI. If a headless Cmdenv run passes and the GUI issue appears only during manual close, treat it separately from maneuver-coordination correctness errors.

Simulation runs may create OMNeT++ result files under `scenarios/artery-maneuver-coordination/results/` and SUMO output under configured SUMO result paths such as `scenarios/artery-maneuver-coordination/results_sumo/...`.

## 7. Basic Validation Checks

Useful log checks for fatal or simulation-level issues:

```bash
grep -E '<!> Error|Segmentation fault|core dumped|what\(\)|Teleporting vehicle|collision with vehicle|peer shutdown|unterminated comment|input ended' /tmp/mcm-*.log | tail -80
```

Merging Request sanity check:

```bash
grep -E 'car_ml1_1.*SEND Request|car_ml1_2.*SEND Request|car_ml1_3.*SEND Request' /tmp/mcm-*-merging-30s.log
```

Emergency checks:

```bash
grep '\[MCM-EMERGENCY\].*queue-emergency-execution-mcm' /tmp/mcm-*-emergency-30s.log | wc -l
grep 'car_ml1_' /tmp/mcm-*-emergency-30s.log | wc -l
```

For the current 30 s emergency validation run, the expected emergency queue count is 150. The expected `car_ml1_` count in the emergency scenario is 0. Merging target-pair checks are useful sanity checks for the current route files, but they should be treated as validation expectations from route timing rather than protocol-level fixed assignments.

## 8. How to Create a New Scenario

A new maneuver-coordination scenario usually needs:

* SUMO network and route files.
* A SUMO `.sumocfg`.
* An OMNeT++ config entry in `scenarios/artery-maneuver-coordination/omnetpp.ini`.
* Vehicle role assumptions for RVs, CVs, and NCVs.
* Optional scenario constants in `src/artery/application/mcm/McScenarioConfig.*`.
* Trajectory CSV/map data if prerecorded intent trajectories are required.
* Validation expectations and sanity-check commands.

Checklist:

1. Define the traffic objective and maneuver type.
2. Define RV, CV, and NCV roles.
3. Create or copy route files.
4. Add a `.sumocfg` that loads the route files and desired SUMO outputs.
5. Add an OMNeT++ config in `scenarios/artery-maneuver-coordination/omnetpp.ini`.
6. Ensure `services-mco-envmod.xml` or the selected service file activates the MC service.
7. Verify MCM generation and participant IDs in a short run.
8. Run a short headless Cmdenv test.
9. Inspect logs, `.sca` output, and SUMO output.
10. Add scenario-specific validation notes.

Prefer dynamic target selection when possible. Avoid adding scenario-specific constants unless the scenario cannot be expressed through existing route/config inputs. Keep background/load vehicles conceptually separate from coordination participants. Document expected sanity checks so future changes can distinguish scenario tuning from behavior regressions.

## 9. OMNeT++ Communication Result Analysis

OMNeT++ result files are written under `scenarios/artery-maneuver-coordination/results/` by the current scenario setup. Scalar/statistic values are stored in `.sca`; vector data may appear in `.vec`/`.vci` when enabled.

The helper `tools/analyze_mcm_qos_results.py` summarizes selected MCM/QoS scalar and statistic output:

```bash
python3 tools/analyze_mcm_qos_results.py \
  --input scenarios/artery-maneuver-coordination/results \
  --output scenarios/artery-maneuver-coordination/results/mcm_qos_summary.csv \
  --aggregate-output scenarios/artery-maneuver-coordination/results/mcm_qos_aggregate.csv \
  --group-without-module
```

The flat CSV contains one row per matching module/metric entry. The aggregate CSV groups by config, metric, and module by default. With `--group-without-module`, it aggregates at config/metric level. Multi-seed output is grouped using run/seed information parsed from `.sca` metadata and filenames.

The helper adds percentage columns for fraction-style metrics:

* `ChannelLoad` -> `cbr_percent_*`
* `packetErrorRate` -> `per_percent_*`
* `coopCBR` -> `coop_cbr_percent_*`

For trajectory counters, it adds:

* `metric_label`
* `trajectory_category`
* `raw_trajectory_type`

Important communication metrics include:

* `McmSentCounter`
* `McmReceivedCounter`
* `McmIntentionSentCounter`
* `McmIntentionReceivedCounter`
* `McmNegotiationSentCounter`
* `McmNegotiationReceivedCounter`
* `McmExecutionSentCounter`
* `McmExecutionReceivedCounter`
* `McmExecutionEmergencySentCounter`
* `McmExecutionEmergencyReceivedCounter`
* `EteDelayMcm`
* `EteDelayMcmNegotiation`
* `EteDelayMcmExecution`
* `EteDelayMcmEmergency`
* `dccTimeWaitNextMcm`
* `ChannelLoad`
* `packetErrorRate`
* `coopCBR`
* `CoopVehicleAgeOfInformation`
* `negotiationTime`
* `NegotiationStartedCounter`
* `NegotiationCompletedCounter`
* `ExecutionStartedCounter`
* `ExecutionCompletedCounter`
* `TrajectoryCost`
* `CounterCoordPossiblePriorityLow`
* `CounterCoordPossiblePriorityMedium`
* `CounterCoordPossiblePriorityHigh`
* `CounterTrajectoryType0/1/2/4/5/6`
* `currentMCSoperatingMode`

## 10. SUMO Traffic Result Analysis

SUMO outputs are configured in the scenario `.sumocfg` files. The current 19-CAV merging and emergency configs write:

* `tripinfo-output`
* `statistic-output`

Several additional outputs are present but commented in the current configs, including lane-change, collision, vehroute, lanedata, edgedata, SSM, and FCD output. If enabled, these outputs can support deeper safety and comfort analysis.

Likely result locations include `scenarios/artery-maneuver-coordination/results_sumo/...`, depending on the active SUMO config.

Traffic metrics to inspect include:

* **Safety:** minimum time gap, minimum TTC, collision/teleport checks, emergency-braking checks, lane-change conflict diagnostics when configured.
* **Comfort:** acceleration, deceleration, maximum acceleration/deceleration, and jerk if available from enabled outputs or future tools.
* **Efficiency:** travel time, time loss, mean speed, throughput, completed trips/routes.
* **Coordination-specific traffic effects:** trajectory cost, CV cooperation cost, priority class, trajectory category, and affected non-cooperating vehicles if that metric is added later.

There is currently no dedicated SUMO traffic-analysis helper in `tools/`. A future `tools/analyze_sumo_traffic_results.py` would be useful for parsing `tripinfo.xml`, `statistic.xml`, optional SSM/FCD output, and producing a traffic-focused CSV summary.

## 11. Maneuver Coordination and Cooperation-Cost Metrics

The current implementation adds MCM-specific OMNeT++ signals/statistics through `McService.ned` and `McService.cc`. Planner metrics are measurement-only; they do not change maneuver decisions or trajectory generation.

Planner/cost metrics:

* `TrajectoryCost`
* `CounterCoordPossiblePriorityLow`
* `CounterCoordPossiblePriorityMedium`
* `CounterCoordPossiblePriorityHigh`
* `CounterTrajectoryType0`
* `CounterTrajectoryType1`
* `CounterTrajectoryType2`
* `CounterTrajectoryType4`
* `CounterTrajectoryType5`
* `CounterTrajectoryType6`

Public trajectory mapping:

| `trajectory_category` | Raw metric name | Meaning |
| --- | --- | --- |
| 0 | `CounterTrajectoryType0` | constant speed / no adaptation / normal time gap |
| 1 | `CounterTrajectoryType1` | deceleration / speed reduction with lane-change trajectory |
| 2 | `CounterTrajectoryType2` | lane-change trajectory |
| 3 | `CounterTrajectoryType4` | constant speed / no adaptation / reduced time gap |
| 4 | `CounterTrajectoryType5` | acceleration with lane-change trajectory / normal time gap |
| 5 | `CounterTrajectoryType6` | acceleration with lane-change trajectory / reduced time gap |

The raw OMNeT++ counter names are intentionally preserved. They are non-contiguous, while the CSV helper provides contiguous public categories from 0 to 5. No synthetic `CounterTrajectoryType3` metric is added.

Cooperation cost is calculated in `TrajectoryPlanner::calculateTrajectoryCost`. The current formula averages speed-change cost, acceleration/deceleration cost, lane-change cost, time-gap-reduction cost, and TTC cost. The TTC cost term is currently zero in the implemented formula.

## 12. Current Limitations

Current limitations and TODOs:

* This is a research/WIP repository, not a fully validated product.
* Scenario constants still live in `src/artery/application/mcm/McScenarioConfig.*`; some should eventually move to configuration.
* Negotiation-related adaptive reduction is not implemented.
* Important Intent Sharing exemption is explicitly marked as TODO in the adaptive intent-generation path.
* Full generalized cascading behavior should be treated as limited unless a scenario explicitly validates it.
* The second-request path is scoped to one retry after a rejected high-priority lane-change Request; broader multi-stage negotiation policies need additional scenario coverage.
* CV lateral lane-change execution is currently logged as not applied in one control path because the lateral target and step counter are not represented there yet.
* Completion semantics are represented through the currently available container path in some execution-completion logic.
* Some detailed metrics are deferred or incomplete:
  * per-second rates,
  * periodicity,
  * delayed MCM counters,
  * DCC transmitted/dropped packets by DP/class,
  * CAM-specific metrics,
  * affected NCV counters,
  * RV trajectory-cost metrics.
* There is no dedicated SUMO traffic-analysis helper in `tools/` yet.
* The 200/500-CAV configs are QoS evaluation scaffolding and should not be treated as final benchmark scenarios.

## 13. Suggested Next Steps for Contributors

Start with the 19-CAV merging and emergency lane-change scenarios. Run short headless simulations, inspect the MCM/QoS CSV summaries, and compare logs against the sanity checks above.

When adding a scenario, make one small change at a time: route files, SUMO config, OMNeT++ config, participant roles, then validation expectations. Add or expose metrics before changing maneuver decisions so behavior changes can be measured rather than inferred.
