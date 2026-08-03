# Maneuver Coordination Architecture and Implementation Status

This document records the current architecture, completed refactoring decisions, accepted compatibility behavior, validation evidence, and future boundaries of the maneuver-coordination research implementation.

## 1. Executive Summary

The supplied 19-CAV validation scenarios implement priority-based Requesting Vehicle/Cooperating Vehicle coordination for merging and emergency-triggered lane changing. Communication integration is separated from application decisions, negotiation and execution containers are used according to phase, trajectory ownership is explicit, scenario calibration is largely runtime-configured, and five simulation regressions protect the established behavior.

The implementation is coherent for its scenario-oriented research scope. It is not a production controller or a general arbitrary-map maneuver framework.

## 2. Current Architecture

| Area | Current implementation | Main evidence |
| --- | --- | --- |
| Communication boundary | `McService` reads/validates NED parameters, extracts ASN.1 snapshots, serializes pending commands, selects containers, and transmits through Artery/Vanetza. | `src/artery/application/McService.*` |
| Application boundary | `McApplication` owns roles, negotiation, retries, trajectory state, decisions, execution, failure/reset state, diagnostics, and scenario triggers. | `src/artery/application/mcm/McApplication*` |
| Planner boundary | Planner-facing CV and second-Request results use named fields; planner mathematics remains in trajectory helpers. | `TrajectoryPlanner.*`, `McApplicationCvDecision.cc`, `McApplicationNegotiationCommon.cc` |
| Runtime configuration | Five typed objects are constructed and validated by `McService`, then passed to `McApplication`. | `McService.ned`, `McService.cc`, `McApplication.h` |
| Validation | Five full Artery/SUMO scenarios are asserted through stable diagnostic events. | `tests/mcm/` |

`McApplication` is physically split across cohesive `.cc` files but remains one class and one state owner. No separate state-machine framework is implied.

## 3. Completed Architectural Changes

- Shared enum display and priority mappings replace duplicated local helpers.
- Scenario calibration was separated from protocol semantics.
- Initial and repeated Execute use `PendingMcmCommand::Kind::Execution` and `ManeuverExecutionContainer`.
- CVs accept current execution-container Execute and retain guarded legacy negotiation-container compatibility.
- Successful completion-as-Cancel construction and recognition are isolated behind role/state-specific helpers.
- RV requested and negotiated trajectories and CV selected and negotiated trajectories have explicit lifetimes.
- Planner-facing positional tuples were replaced by named results.
- `RvCoordinationFailureReason` replaced the broad failure boolean.
- RV and CV reset blocks were consolidated into focused lifecycle helpers.
- Emergency source, merging coordination, merging execution, lane change, and execution-restoration safety are typed runtime configurations.
- Wire, trajectory, failure, state, and configuration diagnostics provide stable regression evidence.

## 4. Protocol Semantics

| Phase/event | Current representation |
| --- | --- |
| Request, Offer, Confirm, Accept, Reject | Negotiation command/container with `requestID` and negotiation participants. |
| Initial Execute | Execution command/container with `cooperationID` and cooperation participants. |
| Repeated Execute | Execution command/container with the same cooperation identity and participants. |
| Emergency Abort | Execution container with `EmergencyPriority`. |
| Successful completion | Isolated negotiation-container Cancel compatibility workaround. |

The numeric successful request ID is carried forward as cooperation ID. No separate cooperation-ID allocator exists. The application supports one or two cooperating CVs.

No ASN.1 Complete category exists. Completion-as-Cancel is accepted current behavior and is deliberately kept separate from pre-execution cancellation. It should not be presented as the final ideal protocol semantic.

## 5. Trajectory Semantics

| State | Meaning |
| --- | --- |
| `mRvRequestedTrajectory` | Active proposal sent in Request and reused by retries. |
| `mRvNegotiatedTrajectory` | Fixed RV execution/completion reference after all required Accepts. |
| `mCvSelectedTrajectory` | CV proposal before execution. |
| `mCvNegotiatedTrajectory` | Fixed CV execution reference after guarded Execute reception. |
| `mEgoContext.plannedTrajectory` | Rolling live trajectory shared through intention information. |

Execution containers have no trajectory field. Initial/repeated Execute do not replace the fixed reference; live intent continues through intention sharing.

## 6. Runtime Configuration

| Type | Responsibility |
| --- | --- |
| `EmergencySourceConfig` | Source identity, braking timing/cadence, and speeds. |
| `MergingCoordinationConfig` | Trigger and target-selection geometry/filtering. |
| `MergingExecutionConfig` | Physical merging speeds. |
| `SafetyCriticalLaneChangeConfig` | Follower identity, geometry, selection, actuation, active safety, and fallback. |
| `ExecutionRestorationSafetyConfig` | Generic post-execution restoration safety. |

`McService` validates these values and `McApplication` consumes typed objects without NED lookup. `McScenarioConfig` remains for limited compile-time planner/scenario calibration; not all configuration has moved to NED.

## 7. Validation Status

The simulation-level suite covers:

- coordinated medium-priority merging;
- merging baseline suppression;
- coordinated emergency lane change;
- emergency braking baseline with MCM suppression;
- forced-Reject second Request;
- initial/repeated execution-container Execute;
- cooperation identity and participants;
- emergency Abort;
- completion workaround;
- trajectory establishment/cleanup;
- runtime configuration defaults; and
- role-specific reset evidence.

These are full simulations with log-based regression oracles, not isolated C++ unit tests. Successful runs use and remove unique temporary output directories.

Important gaps are timeout, final rejection, forced fallback, invalid NED configuration, pre-execution Cancel rollback, direct TraCI failures, explicit legacy Execute injection, and multi-seed statistical evaluation. They should be documented when interpreting coverage but do not block the stated validation scenarios.

## 8. Accepted Workarounds and Constraints

- Completion uses negotiation-container Cancel because ASN.1 has no Complete category.
- Legacy negotiation-container Execute remains accepted by CV receivers.
- Execution-container full-object validation is bypassed around a generated `CooperationID` constraint-descriptor limitation; encoding still occurs.
- Global SUMO coordinates are stored by simulation convention in trajectory fields whose names suggest deltas.
- Second-Request planning may retain a usable non-empty fallback trajectory even when its `found` diagnostic is false.
- A known RV failure reason may remain latched according to existing merging lifecycle behavior until the established clear point.

These constraints should be changed only in separately reviewed, regression-protected work.

## 9. Scenario Scope

Reusable architecture includes encoding/decoding, negotiation/execution lifecycle, retry/timeout infrastructure, priority/cooperation-cost logic, typed configuration, diagnostics, and regression testing.

Scenario-specific behavior includes route-based role recognition, global validation coordinates, lane-index assumptions, merge and lane-change geometry, shifted/prerecorded trajectories, incremental physical lane-change execution, and calibrated speed/safety values.

SUMO supports arbitrary road networks. Generalizing this application requires road-topology, route-relative geometry, adjacent-lane, conflict-area, and maneuver-area providers; the absence of those abstractions is not an inherent SUMO limitation.

## 10. Future Enhancements

- Add deterministic negative-path tests before changing timeout, rejection, fallback, cancellation, or TraCI-failure behavior.
- Add route/lane geometry and maneuver-area abstractions for new networks.
- Generalize participant tracking only if a scenario requires more than two CVs.
- Keep 200/500-CAV QoS configurations labeled as experimental until multi-seed validation supports performance claims.
- Consider explicit completion semantics only as a separately versioned ASN.1/protocol task.
- Abstract direct TraCI control only when driven by a concrete testability or controller-integration requirement.

## 11. Change-Safety Notes

- Keep `McService` communication concerns separate from `McApplication` decisions.
- Do not collapse requested, negotiated, and live trajectory state.
- Do not infer cooperation identity independently from the successful request under the current policy.
- Do not treat priority as an event subtype.
- Do not remove legacy compatibility or completion workarounds without mixed-version/scenario evidence.
- Do not derive participant-normalized metrics from a fixed three-vehicle assumption.
