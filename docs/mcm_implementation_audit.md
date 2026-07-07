# Maneuver Coordination Implementation Status

This document summarizes the current Maneuver Coordination implementation, the evidence in the repository, validation status, and recommended next steps. It is a public status and roadmap document, not a validation claim.

The implementation currently covers the main 19-CAV validation behavior and a staged communication/QoS measurement path. The repository remains research-oriented and under active restructuring, so incomplete items are described as current limitations or planned extensions.

## 1. Executive Summary

The strongest areas are the core maneuver-coordination path: RV/CV roles, Request/Offer/Confirm/Accept/Execute/Complete-as-Cancel handling, split 19-CAV scenarios, dynamic merge target selection, emergency lane-change behavior, service/application separation, and baseline MCM/QoS result extraction.

The main remaining gaps are evaluation completeness and broader scenario coverage:

* negotiation-related adaptive frequency reduction is not implemented;
* Important Intent Sharing needs an explicit current state before exemption rules can be implemented;
* delayed-message counters, per-second rates, DCC transmitted/dropped packets by profile, and some detailed planner counters are deferred;
* second-request handling is available as a scoped high-priority lane-change retry path, while generalized cascading remains limited;
* congested 200-CAV/500-CAV QoS configs exist as evaluation scaffolding and need full multi-seed validation before performance claims.

## 2. Implementation Status

| Area | Current implementation | Evidence/files | Status | Recommendation |
| --- | --- | --- | --- | --- |
| Service/application structure | `McService` handles middleware/service tasks, while `McApplication` handles maneuver coordination state and trajectory helpers support planner logic. | `src/artery/application/McService.*`, `src/artery/application/mcm/McApplication.*`, `src/artery/application/mcm/Trajectory*.{cc,h}` | Implemented | Keep service, application, and planner responsibilities separate. |
| Packet generation/reception | The service builds ASN.1 MCMs, sends them through Artery middleware, decodes received MCMs, and passes snapshots to the application. | `McService::sendMcm`, `McService::indicate`, `McApplication::handleSentMcm`, `McApplication::handleReceivedMcm` | Implemented | Route maneuver behavior changes through `McApplication`; keep packet plumbing in `McService`. |
| Intent vs coordination triggering | Intent and coordination MCMs use separate trigger paths. | `McService::shouldGenerateIntentMcm`, `McService::shouldGenerateCoordinationMcm` | Implemented | Keep Intent sharing and maneuver-negotiation triggering separate. |
| Coordination 10 Hz rate limit | Coordination messages are queued by the application and transmitted through the service generation path, preserving same-vehicle/service spacing. | `shouldGenerateCoordinationMcm`, `intervalForCoordinationTriggeringCondition` | Implemented | Preserve the 0.1 s validation cadence unless a scenario explicitly changes it. |
| DCC profile mapping | Differentiated DCC profile mapping is available behind `dccProfiles=true`; the default keeps DP2 behavior. | `McService::selectMcmDccProfile`, `McService.ned` | Implemented, needs congested validation | Validate DCC-profile configs under congestion before drawing conclusions. |
| Timestamp source | Intent MCMs use the configured timestamp source. Negotiation and execution messages use current-time generation timestamps when commands are present. | `McService.ned`, `McService::sendMcm` | Implemented | Document timestamp settings when comparing end-to-end delay results. |
| Local CBR input | Local CBR is observed from channel-load signals and emitted as `coopCBR`. | `RadioDriverBase::ChannelLoadSignal`, `mLocalCbr`, `coopCBR` | Implemented | Use as the baseline input for QoS validation. |
| Adaptive Intent reduction | General adaptive Intent reduction and MCO-specific 1 Hz Intent reduction are available behind disabled-by-default flags. | `McService::updateAdaptiveIntentFrequency`, `newGenMcmRulesIntent`, `newGenMcmRulesIntent1Hz_MCO` | Experimental | Keep disabled by default until congested scenario validation is complete. |
| Negotiation adaptive reduction | Adaptive reduction for negotiation-related traffic is not implemented. | `McService::updateAdaptiveIntentFrequency` | Planned extension | Implement as a separate QoS task, not as a change to maneuver decisions. |

## 3. Implemented Maneuver Coordination Features

| Area | Current implementation | Evidence/files | Status | Recommendation |
| --- | --- | --- | --- | --- |
| RV/CV/NCV role model | Vehicles are tracked as NCV, RV, CV, or emergency vehicle for coordination state and logging. | `cooperatingVehicleType` in `McApplication.h` | Implemented | Keep roles explicit in traces and metrics. |
| Request/Offer/Confirm/Accept | Current validation paths support two-CV negotiation and one-CV high-priority lane-change negotiation. | `handleReceivedOfferAsRv`, `handleReceivedConfirmAsCv`, `handleReceivedAcceptAsRv`, `haveAllExpectedRvOffers`, `haveAllExpectedRvAccepts` | Implemented for validation flows | Continue testing one-CV and two-CV paths separately. |
| Missing Accept evidence handling | An RV can use Execute evidence from an expected CV/request to complete RV-side negotiation measurement when Accept was not observed. | `isExecuteEvidenceForActiveRvRequest`, `handleReceivedExecuteEvidenceAsRv` | Implemented | Keep evidence matching tied to expected CV and request ID. |
| Retry/timeout helpers | Request, Confirm, Offer, and Accept retry/timeout helpers are present for active negotiation phases. | `evaluateRvRequestRetry`, `evaluateRvConfirmRetry`, `evaluateCvOfferRetry`, `evaluateCvAcceptRetry` | Implemented | Avoid broad timeout changes without trace validation. |
| Emergency execution | The emergency vehicle sends high-priority emergency execution MCMs in the emergency lane-change validation scenario. | `evaluateEmergencyBrakingTrigger`, `envmod-19CAVs-emergency-lane-change` | Implemented for validation scenario | Keep scenario constants isolated in `McScenarioConfig`. |
| Dynamic merge targets | Merging RVs select target CVs dynamically from current trajectory/environment information. | `evaluateMergingRequestTrigger`, merge target selection helpers, README validation pairs | Implemented | Keep target selection data-driven and avoid scenario-specific station-ID assignments. |
| Complete-as-Cancel | Current completion signaling uses the available Cancel-style path to represent maneuver completion. | `resetRvCoordinationStateAfterComplete`, `resetCvCoordinationStateAfterComplete`, Cancel handlers | Implemented with documented limitation | Document carefully when adding distinct Complete/Cancel/Abort semantics. |
| Current operating mode metric | The service emits numeric current MCS operating mode values. | `McService.ned`, `emitCurrentOperatingModeIfChanged` | Implemented | Use for result correlation with message traces. |
| Planner/cost presentation | Planner cost and trajectory category counters are emitted and labeled in CSV output. | `TrajectoryCost`, `CounterTrajectoryType0/1/2/4/5/6`, `tools/analyze_mcm_qos_results.py` | Implemented | Keep raw OMNeT++ metric names stable and use public CSV labels for readability. |

## 4. Partial Features and Validation Gaps

| Area | Current implementation | Evidence/files | Status | Recommendation |
| --- | --- | --- | --- | --- |
| Important Intent Sharing exemption | The adaptive path notes that an explicit important-intent state is needed before exemption rules can be implemented. | `updateAdaptiveIntentFrequency` TODO | Planned extension | Define the current important-intent state first. |
| `PeriodicFixed0.5Hz` transitions | `PeriodicFixed0.5Hz` exists as a typed triggering condition, but the adaptive transition path does not currently use it. | `IntentTriggeringCondition::PeriodicFixedHalfHz` | Deferred | Keep deferred until congested validation identifies a need. |
| `PeriodicFixedNoDCC` | Coordination mode can bypass DCC gating while using periodic timing. | `CoordinationTriggeringCondition::PeriodicFixedNoDcc`, `dccGate` logic | Partially validated | Add a targeted smoke validation with config override. |
| Second request / cascading flows | A rejected high-priority lane-change Request can trigger one RV-generated second Request using `TrajectoryPlanner::findSecondRequestTrajRV`; generalized cascading remains limited. | `McApplication::handleReceivedRejectAsRv`, `McApplication::makeRvSecondRequestCommand`, `TrajectoryPlanner::findSecondRequestTrajRV` | Scoped support | Add scenario coverage before broadening to additional negotiation policies. |
| Reject/Cancel behavior | Reject and Cancel handlers exist for current flows; broader semantics need scenario coverage. | `handleReceivedRejectAsRv`, `handleReceivedCancelAsCv` | Partial | Add scenario tests before broadening Cancel/Reject behavior. |
| Congested QoS configs | 200-CAV and 500-CAV configs are available for Free Space baseline/adaptive/MCO/DCC-profile evaluation. | `envmod-200CAVs-qos-*`, `envmod-500CAVs-qos-*` | Evaluation scaffolding | Run multi-seed validation before benchmark claims. |

## 5. Known Limitations and Planned Extensions

| Area | Current implementation | Evidence/files | Status | Recommendation |
| --- | --- | --- | --- | --- |
| Delayed MCM counters | Delay statistics are available, but dedicated delayed-message counters by subtype are not implemented. | `EteDelayMcm*`, `McService.ned` | Planned if needed | Define threshold and subtype logic before adding counters. |
| Per-second MCM subtype rates | Counters exist; direct per-second subtype-rate signals are not implemented. | `Mcm*Counter` signals, `.vec` support when enabled | Planned if needed | Prefer deriving rates from vector output unless scalar KPIs are required. |
| MCM periodicity metric | Direct periodicity scalar is not implemented. | generation timestamps, send counters | Planned if needed | Add only if generation regularity must be a scalar KPI. |
| Number of transmitting vehicles | No dedicated MCM transmitting-vehicle counter is exposed. | module-level sent counters | Deferred | Prefer deriving from modules with sent counts. |
| DCC transmitted/dropped by profile/subtype | These are lower-layer statistics and are not fabricated in `McService`. | DCC/radio layer metrics | External/deferred | Use lower-layer statistics sources. |
| CAM-specific metrics | CAM metrics are outside the current MCM helper scope. | CA service and lower-layer metrics | Separate analysis scope | Keep MCM and CAM analysis separate unless doing coexistence studies. |
| Affected NCV counters | Affected non-cooperating-vehicle counters are not currently exposed as validated metrics. | planner/application TODO scope | Deferred | Add only after defining scenario semantics. |
| RV trajectory-cost metrics | `TrajectoryCostRV` is exported for the scoped second-request path; broader RV trajectory-type counters remain deferred. | `TrajectoryCostRV`, `McApplication::makeRvSecondRequestCommand` | Scoped support | Keep the metric tied to validated RV-side cost calculations. |
| Negotiation MCMs per cooperating vehicle | Config-level ratios exist; participant-normalized ratios need role-specific counters or a safe denominator. | `tools/analyze_mcm_qos_results.py` | Deferred | Avoid fixed participant divisors across one-CV and two-CV flows. |

## 6. Metrics Status

| Metric | Current status | Notes |
| --- | --- | --- |
| `McmSentCounter`, `McmReceivedCounter` | Implemented | Emitted on actual send/receive. |
| Intent/Negotiation/Execution/Emergency subtype counters | Implemented | Classification is based on decoded/current MCM metadata. |
| `msgsizeReceived` | Implemented | Current receive-side size statistic. |
| `EteDelayMcm*` | Implemented | Uses reconstructed generation timestamp; negative wraparound samples are guarded. |
| `dccTimeWaitNextMcm` | Implemented | Emitted from DCC interval calculation. |
| `coopCBR` | Implemented | Local CBR sampled/emitted through MCM service. |
| `ChannelLoad` / CBR | Extracted if present | Lower-layer radio statistic; analysis tool includes it. |
| `packetErrorRate` / PER | Extracted if present | Lower-layer radio statistic; analysis tool includes it. |
| `CoopVehicleAgeOfInformation` | Implemented | Coordination-relevant negotiation/execution MCMs for listed participants only. |
| `negotiationTime` | Implemented | First RV Request send to final required Accept, or missing-CV Execute evidence. |
| Negotiation MCMs per completed negotiation | Derived | `McmNegotiationSentCounter / NegotiationCompletedCounter` in aggregate output. |
| Negotiation MCMs per started negotiation | Derived | `McmNegotiationSentCounter / NegotiationStartedCounter` in aggregate output. |
| Negotiation MCMs per cooperating vehicle | Deferred | Needs role-specific counters or safe participant denominator. |
| `currentMCSoperatingMode` | Implemented | Numeric mode signal. |
| Priority/planner/cooperation-cost metrics | Implemented for current CV planner path | Includes `TrajectoryCost`, possible-priority counters, and trajectory category counters. |
| Second-request metrics | Implemented for scoped second-request path | Includes `SecondRequestStartedCounter`, `SecondRequestCompletedCounter`, `SecondRequestRejectedCounter`, and `TrajectoryCostRV`. |
| Delayed MCM, rate, periodicity, DCC transmitted/dropped metrics | Missing or external | See Section 5. |

## 7. Scenario Validation Status

| Scenario/config | Current status | Notes |
| --- | --- | --- |
| `envmod-19CAVs-merging` | Baseline validation target | Current expected target pairs are documented in README and checked in recent runs. |
| `envmod-19CAVs-emergency-lane-change` | Baseline validation target | Emergency queue count and absence of `car_ml1_` emergency logs are tracked as smoke checks. |
| GUI variants | Configured | Should remain behavior-equivalent to command-line variants, apart from GUI/SUMO shutdown behavior. |
| `envmod-200CAVs-qos-*` | Needs validation | QoS scaffolding with Free Space baseline/adaptive/MCO/DCC-profile variants. |
| `envmod-500CAVs-qos-*` | Needs validation | Same as 200-CAV; avoid heavy runs during routine documentation tasks. |

The README documents the distinction between the designated 19 coordination vehicles and 200/500-CAV background load vehicles. That distinction should remain explicit in future scenario changes.

## 8. Documentation Gaps

* Add a compact architecture diagram or sequence table for the current RV/CV message flow: Request, Offer, Confirm, Accept, Execute, Complete-as-Cancel.
* Document delayed-message and DCC profile metrics if they become part of the required result set.
* Document DCC profile mapping in a small table, including the default `dccProfiles=false` behavior.
* Add a short note that `timeGenMcm=DataProvider` remains the NED default for Intent behavior, while coordination timestamps use current simulation time.
* Keep the 200/500-CAV QoS configs labeled as experimental until multi-seed congested validation is complete.

## 9. Suggested Next Tasks

1. Keep planner measurement counters aligned with current planner decisions.
2. Add delayed MCM and periodicity metrics if they are required for congested QoS tables.
3. Add role-specific negotiation MCM counters if per-cooperating-vehicle metrics are required.
4. Validate `PeriodicFixedNoDCC` and DCC-profile mapping with short targeted smoke configs before using them in congested campaigns.
5. Design the Important Intent Sharing state before implementing exemption logic in adaptive rules.
6. Evaluate negotiation-related adaptive reduction as a separate QoS task.
7. Run 200-CAV and 500-CAV multi-seed QoS experiments only after the metric set is stable.

## 10. Risks / Things Not To Change Blindly

* Do not collapse service, application, and planner responsibilities into one class.
* Do not use a fixed three-vehicle divisor for negotiation MCMs per vehicle; current scenarios include one-CV and two-CV flows.
* Do not treat background 200/500-CAV Intent MCMs as coordination participants.
* Do not merge Intent and coordination triggering paths.
* Do not use RV Execute send time as a proxy for negotiation completion.
* Do not change DCC profile defaults while adding evaluation configs.
* Do not fabricate PER or DCC transmitted/dropped metrics inside `McService`; use the lower-layer statistics source.
* Do not overclaim congested-scenario performance until multi-seed Free Space QoS runs are complete.
