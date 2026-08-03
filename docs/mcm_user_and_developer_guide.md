# MCM User and Developer Guide

This guide describes the functioning maneuver-coordination research implementation, its validation scenarios, extension points, diagnostics, and current scope. It should be read together with the [trajectory guide](trajectory_planning_and_conflict_checking.md) and [regression-test guide](../tests/mcm/README.md).

## 1. Research Scope

The repository extends Artery with decentralized V2X maneuver coordination between Requesting Vehicles (RVs) and Cooperating Vehicles (CVs). It validates medium-priority cooperative merging and high-priority safety-critical lane changing in predefined SUMO scenarios.

The reusable application and communication lifecycle is separated from scenario calibration, but the current physical behavior is intentionally scenario-oriented. It is suitable for reproducible research and validation, not production vehicle control or arbitrary-map deployment without adaptation.

## 2. Architecture

### McService

`src/artery/application/McService.*` owns communication integration. It reads and validates NED parameters, constructs typed configuration, updates the application ego context, extracts received ASN.1 MCM snapshots, serializes pending commands, selects the negotiation or execution container, adds mandatory intention sharing, and transmits through Artery/Vanetza.

### McApplication

`src/artery/application/mcm/McApplication.*` owns role and scenario recognition, negotiation, retry/timeout logic, planner decisions, trajectory state, execution control, execution progress, failure state, reset behavior, diagnostics, and measurements.

It remains one class whose implementation is split for navigation:

| File | Responsibility |
| --- | --- |
| `McApplication.cc` | Lifecycle, dispatch, receive handlers, and common orchestration. |
| `McApplicationMerging.cc` | Merging trigger and target selection. |
| `McApplicationLaneChange.cc` | Emergency source/follower behavior, lane-change trigger, and fallback. |
| `McApplicationCvDecision.cc` | CV feasibility, planner interpretation, and protocol decision. |
| `McApplicationRetry.cc` | Request, Confirm, Offer, and Accept retry/timeout paths. |
| `McApplicationExecutionControl.cc` | RV/CV TraCI execution and restoration. |
| `McApplicationExecutionProgress.cc` | Repeated Execute and completion evaluation. |
| `McApplicationNegotiationCommon.cc` | Command builders, guards, completion predicates, resets, and second Request. |
| `McApplicationDiagnostics.cc` | Focused scenario diagnostics. |

These files share one `McApplication` state object; they are not independent modules or a separate state-machine framework.

## 3. Initialization and Runtime Configuration

`McService` reads NED parameters during initialization, validates them with OMNeT++ runtime errors, builds typed values, and calls focused `McApplication` setters. `McApplication` never performs direct NED parameter lookup.

| Type | Scope |
| --- | --- |
| `EmergencySourceConfig` | Source identity, braking timing, broadcast interval, normal speed, and emergency speed. |
| `MergingCoordinationConfig` | Requesting route, global trigger geometry, lane filtering, snapshot freshness, and target-selection gap. |
| `MergingExecutionConfig` | RV merging speed and CV acceleration target. |
| `SafetyCriticalLaneChangeConfig` | Follower/source routes, lane assumptions, planned shift, execution steps, target selection, safety, and fallback. |
| `ExecutionRestorationSafetyConfig` | Shared post-execution leader distance, time-gap, and TTC thresholds. |

Defaults in `McService.ned` preserve the supplied validation scenarios; relevant `omnetpp.ini` configurations can override them. Values are grouped by responsibility so emergency source behavior, merging selection, merging actuation, active lane-change safety, and generic restoration safety do not share accidental sources of truth.

`McScenarioConfig.*` still contains a limited set of compile-time planner/scenario calibration values: `scHighwayMergingRouteId`, `scNormalHighwaySpeed`, `scMergingTimeGap`, `scRequestTrajectorySteps`, `scRequestTrajectoryDt`, and `scMaxReceivedMcmCache`. New runtime scenario calibration should normally use a focused typed NED configuration rather than expanding this file indiscriminately.

## 4. Roles and Operation Modes

- **RV:** initiates coordination, selects targets, tracks responses, and controls execution.
- **CV:** evaluates a targeted request, selects a feasible proposal, responds, and applies agreed control.
- **NCV:** contributes surrounding traffic and intent information without joining negotiation.
- **Emergency source:** brakes according to configuration and may emit `EmergencyPriority` Abort messages.

Application modes are intention sharing, maneuver negotiation, and maneuver execution. Focused RV/CV progress values track the current lifecycle within those modes.

## 5. Message and Identity Lifecycle

Negotiation messages use `ManeuverNegotiationContainer` and `requestID`:

```text
Request -> Offer -> Confirm -> Accept
```

Reject is handled explicitly. In the validated high-priority path, the first Reject can create one replacement Request with a new request ID and proposal. The final successful request identity establishes the cooperation.

Initial and repeated Execute use `ManeuverExecutionContainer`. The successful request ID is carried forward numerically as `cooperationID`, and `cooperationVehicleID1` plus the optional second partner identify participants:

```text
Accept -> Execute -> repeated Execute while execution is active
```

CV reception retains legacy compatibility for negotiation-container Execute. That path still requires the active RV sender, matching request ID, ego participant membership, and eligible CV state. Current senders use the execution container.

Emergency behavior uses `EmergencyPriority` plus Abort in an execution container. Priority describes urgency; Abort describes the event.

No dedicated ASN.1 Complete category exists. Successful execution completion is temporarily encoded by narrowly scoped helpers as negotiation-container Cancel. Recognition requires the matching role and completion state; subtype Cancel alone is insufficient. Sender-local control restoration and cleanup then occur at the established point. This compatibility workaround is accepted current behavior and is distinct from pre-execution negotiation cancellation.

## 6. One-CV and Two-CV Negotiation

The application supports a maximum of two cooperating CVs.

- In two-CV merging, the RV waits for both Offers, sends Confirm, waits for both Accepts, and then starts execution.
- In a one-CV high-priority flow, the targeted CV can send Accept directly after Request.
- Participant IDs remain ordered in the existing first/optional-second fields.
- Retries do not overwrite another pending command.

Execute evidence from an expected CV can complete missing-Accept measurement evidence when the established compatibility guards match. It does not weaken participant or identity validation.

## 7. Trajectory Ownership

Trajectory members have distinct lifetimes:

- `mRvRequestedTrajectory` is the active Request proposal. Retries reuse it, and a second Request replaces it.
- `mRvNegotiatedTrajectory` becomes active only after all required Accepts and remains the fixed RV completion reference.
- `mCvSelectedTrajectory` is the CV proposal before execution.
- `mCvNegotiatedTrajectory` is fixed when a valid Execute arms CV execution.
- `mEgoContext.plannedTrajectory` is rolling live intent generated from current ego state.

Execution containers carry identity, priority, and partners but no trajectory. Live trajectory information remains in the mandatory intention-sharing container. Repeated Execute neither serializes nor replaces the fixed negotiated reference.

The planner-facing CV and second-Request APIs use named result fields. In the second-Request compatibility contract, a non-empty returned trajectory remains usable even when the planner's `found` field is false; the caller records `found` diagnostically and rejects only an empty result.

## 8. Validation Scenarios

### Cooperative merging

`envmod-19CAVs-merging` validates medium-priority, trajectory/gap-based target selection, one/two-CV negotiation behavior, execution-container Execute, scenario-configured actuation, safe restoration, and completion cleanup.

### Emergency lane change

`envmod-19CAVs-emergency-lane-change` validates the emergency Abort source, follower qualification, HighPriority lane-change negotiation, target-lane cooperation, incremental movement, active-execution safety, fallback plumbing, and completion cleanup.

The planned lateral trajectory shift is `3.0 m`. Physical execution uses 10 increments of `0.32 m`, totaling `3.2 m`. They are separate scenario calibrations and should not be derived from one another.

### Baselines and second Request

The merging baseline suppresses maneuver coordination. The emergency baseline preserves source braking while suppressing emergency MCM transmission and follower coordination. `envmod-19CAVs-second-request-smoke` verifies replacement of the first rejected proposal and continuation with a new identity.

Experimental 200/500-CAV QoS configurations add background communication load. They are evaluation scaffolding, not validated coordination benchmarks.

## 9. Running and Testing

Build:

```bash
cmake --build build -j2
```

Run merging:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-merging -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

Run emergency lane change:

```bash
tools/run_artery.py -l build -s scenarios/artery-maneuver-coordination -- omnetpp.ini -u Cmdenv -c envmod-19CAVs-emergency-lane-change -r 0 --sim-time-limit=30s --cmdenv-express-mode=false
```

Run the regression suite:

```bash
python3 -m unittest discover -s tests/mcm -p 'test_*.py'
```

The five tests run complete Artery/SUMO simulations and inspect stable diagnostic events. They are simulation-level regressions, not isolated C++ unit tests. Successful runs remove unique temporary output directories; failed runs preserve their logs and outputs for diagnosis.

## 10. Diagnostics and Measurements

Stable diagnostic families include:

- `[MCM-WIRE]`: command/container/subtype/identity/participant evidence.
- `[MCM-TRAJECTORY]`: requested and negotiated trajectory lifecycle.
- `[MCM-FAILURE]`: explicit RV failure-reason transitions.
- `[MCM-STATE]`: role-specific reset evidence.
- `[MCM-CONFIG]`: validated runtime configuration.
- `[MCM-NEGOTIATION]`, `[MCM-EMERGENCY]`, and scenario-specific target/execution tags.

Useful signals include MCM/container counters, subtype delays, DCC wait, channel load, negotiation/execution counters, second-Request counters, planner priority, trajectory category, and trajectory cost. `tools/analyze_mcm_qos_results.py` can summarize matching `.sca` output.

The simulator records selected trajectory category and associated cooperation cost for evaluation and diagnostics. They represent internal planner decisions used to compare scenario outcomes; they are not MCM protocol fields and do not alter wire messages. Raw counter names remain available to analysis tools without requiring a category table in this guide.

## 11. Adding or Adapting a Scenario

1. Define RV, CV, NCV, and emergency roles for the intended use case.
2. Add or adapt SUMO network, route, and `.sumocfg` files.
3. Add an `omnetpp.ini` configuration and enable the MC service.
4. Override the relevant typed `McService` NED parameters.
5. Preserve McService validation; do not query NED parameters from McApplication.
6. Verify route IDs, lane relationships, global coordinates, trigger/conflict areas, and participant ordering.
7. Provide continuous road-aligned reference trajectory data for the maneuver area.
8. Validate physical control and safety calibration separately from negotiation logic.
9. Add stable diagnostics only where existing evidence is insufficient.
10. Add a simulation regression that proves the intended sequence and baseline behavior.

Configuration alone does not make arbitrary maps compatible. New geometry may require extending the route/lane geometry abstraction before planner results are meaningful.

## 12. Reset and Failure Handling

Focused RV and CV helpers clear negotiation, execution, response-tracking, retry, trajectory, and control state without hiding lifecycle differences. Vehicle restoration remains separate from protocol-state clearing where ordering matters.

`RvCoordinationFailureReason` distinguishes `None`, timeout, rejection, unsafe environment, and control failure according to existing paths. Reasons are cleared only at the established lifecycle points; a recoverable first Reject does not incorrectly terminate the second-Request path.

Successful completion clears requested/selected and negotiated trajectory state through the completion-specific helpers. Timeout, final rejection, Cancel reception, and fallback retain their distinct cleanup behavior.

## 13. Current Scope and Future Generalization

SUMO supports arbitrary road networks. This application currently relies on configured route IDs, lane relationships, global SUMO coordinates, validation-map thresholds, predefined maneuver areas, shifted/prerecorded trajectories, and scenario-calibrated physical control. Arbitrary-network support would require road-topology, route-relative geometry, adjacent-lane, conflict-area, and maneuver-area abstractions.

The current point/index conflict model, maximum two-CV flow, direct TraCI coupling, legacy Execute compatibility, and completion workaround are explicit research-scope boundaries. Missing deterministic tests include timeout, final rejection, forced fallback, invalid NED configuration, pre-execution Cancel rollback, and direct TraCI failures. These gaps limit claims but do not invalidate the covered scenarios.

Future contributions should generalize one boundary at a time and preserve regression evidence for the established scenarios.
