# MCM Implementation Audit

This audit compares the current refactored maneuver-coordination implementation with the old reference implementation under `reference/mcm_old`. It is a status document, not a validation claim. The current implementation has restored the main 19-CAV validation behavior and much of the communication/QoS measurement path, but it is not a full restoration of every old research hook.

## 1. Executive Summary

The current implementation is strongest in the core maneuver-coordination path: RV/CV roles, Request/Offer/Confirm/Accept/Execute/Complete-as-Cancel handling, split 19-CAV scenarios, dynamic merge target selection, emergency lane-change behavior, and the service/application split are all present and validated on the baseline scenarios.

The main remaining gaps are research/evaluation completeness rather than immediate scenario behavior:

* negotiation-related adaptive frequency reduction from the old `newGenMcmRules` path is still missing;
* the old Important Intent Sharing exemption is not restored because the refactored implementation does not expose an equivalent state yet;
* detailed old research metrics for delayed MCMs, per-second rates, DCC transmitted/dropped packets by profile, priority-planner counters, trajectory-type counters, and trajectory costs are still partial or missing;
* second-request/cascading negotiation counters and flows are represented in enums/helpers but are not fully restored as validated behavior;
* congested 200-CAV/500-CAV QoS configs exist as WIP scaffolding and need full multi-seed validation.

## 2. Current Implementation Status

| Area | Old behavior | Current status | Evidence/files | Recommendation |
|---|---|---|---|---|
| Service/application structure | Old `McmService` owned service, generation, reception, maneuver state, planner calls, and metrics in one large class. | Restored with refactoring | `McService.*`, `mcm/McApplication.*`, `mcm/Trajectory*.{cc,h}` | Keep the split. Do not re-merge old monolithic logic. |
| Packet generation/reception | Old service built and sent ASN.1 MCMs and decoded received MCMs directly. | Restored | `McService::sendMcm`, `McService::indicate`, `McApplication::handleSentMcm`, `handleReceivedMcm` | Continue routing behavior changes through `McApplication`, not `McService`. |
| Intent vs coordination triggering | Old service had separate `checkTriggeringConditions` and `checkTriggeringConditionsCoordination`. | Restored | `McService::shouldGenerateIntentMcm`, `McService::shouldGenerateCoordinationMcm` | Keep the two trigger paths separate. |
| Coordination 10 Hz rate limit | Old coordination messages waited for service-generation intervals. | Restored | `shouldGenerateCoordinationMcm`, `intervalForCoordinationTriggeringCondition` | Preserve same-vehicle/service minimum 0.1 s spacing. |
| DCC profile mapping | Old `dccProfiles=true` selected differentiated DPs. | Partially restored | `McService::selectMcmDccProfile`, `McService.ned` | Defaults intentionally preserve current DP2 behavior; validate DCC-profile configs under congestion. |
| Timestamp source | Old default `timeGenMcm=CurrentTime`; current default remains `DataProvider` for Intent. Negotiation/execution use current time. | Partially restored | `McService.ned`, MCM timestamp generation path | Keep negotiation/execution on CurrentTime; document Intent behavior when comparing E2E delay. |
| Local CBR input | Old used local CBR for adaptive rules and `coopCBR`. | Restored | `RadioDriverBase::ChannelLoadSignal`, `mLocalCbr`, `coopCBR` | Good baseline for QoS validation. |
| Adaptive Intent reduction | Old reduced regular Intent MCM generation under CBR thresholds. | Partially restored | `updateAdaptiveIntentFrequency` | General and MCO 1 Hz intent paths are present; old negotiation reduction remains missing. |
| MCO 1 Hz rule | Old separate `newGenMcmRulesIntent1Hz_MCO` block. | Restored experimentally | `newGenMcmRulesIntent1Hz_MCO`, `freqReduceCBRmco` | Disabled by default; validate in congested scenarios before claims. |

## 3. Old Implementation Behavior Already Restored

| Area | Old behavior | Current status | Evidence/files | Recommendation |
|---|---|---|---|---|
| RV/CV role model | Old used `NCV`, `RV`, `CV`, `EmergencyV`. | Restored | `cooperatingVehicleType` in `McApplication.h` | Keep roles explicit in logs and metrics. |
| Request/Offer/Confirm/Accept | Old supported two-CV negotiation and one-CV high-priority paths. | Restored for current validation flows | `handleReceivedOfferAsRv`, `handleReceivedConfirmAsCv`, `handleReceivedAcceptAsRv`, `haveAllExpectedRvOffers`, `haveAllExpectedRvAccepts` | Continue testing one-CV and two-CV paths separately. |
| Missing Accept fallback | RV may treat CV Execute as successful evidence if Accept was missed. | Restored | `isExecuteEvidenceForActiveRvRequest`, `handleReceivedExecuteEvidenceAsRv` | Keep this tied to expected CV/request only. |
| Retry/timeout helpers | Old had response timers and retry behavior. | Restored in focused form | `evaluateRvRequestRetry`, `evaluateRvConfirmRetry`, `evaluateCvOfferRetry`, `evaluateCvAcceptRetry` | Avoid broad timeout changes without trace validation. |
| Emergency execution | Old emergency vehicle sent high-priority execution/emergency messages. | Restored for validation scenario | `evaluateEmergencyBrakingTrigger`, emergency-lane-change config | Keep scenario constants isolated in `McScenarioConfig`. |
| Dynamic merge targets | Old behavior relied on route timing and hard-coded expectations. | Improved/restored | `evaluateMergingRequestTrigger`, merge target selection helpers, README validation pairs | Keep target selection data-driven; do not hard-code current station IDs. |
| Complete-as-Cancel | Old ASN.1 used Cancel as a Complete workaround. | Restored | `resetRvCoordinationStateAfterComplete`, `resetCvCoordinationStateAfterComplete`, Cancel handlers | Document carefully when adding true Cancel/Abort semantics. |
| Current operating mode metric | Old emitted `currentMCSoperatingMode`. | Restored | `McService.ned`, `emitCurrentOperatingModeIfChanged` | Useful for result correlation. |

## 4. Partial Restorations / Known Limitations

| Area | Old behavior | Current status | Evidence/files | Recommendation |
|---|---|---|---|---|
| Important Intent Sharing exemption | Old adaptive rules avoided reducing important intent sharing with `mIntentionSharingImportant == false`. | Missing state, documented TODO | `updateAdaptiveIntentFrequency` TODO | Restore only after defining an explicit current important-intent state. |
| Negotiation frequency reduction | Old reduced non-participant Intent generation after observing negotiation messages under CBR. | Missing | old `McmService.cc` negotiation CBR block around negotiation receive; current adaptive code only handles Intent rules | Implement later as a separate QoS task; do not mix with current MCO Intent rule. |
| `PeriodicFixed0.5Hz` adaptive transitions | Old had transitions involving 0.5 Hz in some adaptive paths. | Deferred intentionally | `IntentTriggeringCondition::PeriodicFixedHalfHz` exists but adaptive transition not restored | Keep deferred until congested validation identifies need. |
| `PeriodicFixedNoDCC` | Old coordination mode bypassed DCC but still used periodic timing. | Partially restored | `CoordinationTriggeringCondition::PeriodicFixedNoDcc` and `dccGate` logic | Needs explicit smoke validation with config override. |
| Second request / cascading flows | Old had `SecondRequest`, cascading enums/counters, and second request planning. | Partially present, not validated/restored | enums in `McApplication.h`; `TrajectoryPlanner::findSecondRequestTrajRV`; missing signals/counters | Do not revive old fixed assumptions blindly; define current use cases first. |
| Reject/Cancel behavior | Old had Reject, Cancel, Abort-like branches with overlapping ASN.1 workarounds. | Partially restored | `handleReceivedRejectAsRv`, `handleReceivedCancelAsCv` | Add scenario tests before broadening Cancel/Reject semantics. |
| Trajectory/planner metrics | Old emitted cost/type/priority counters. | Planner logic exists, metrics mostly missing | `TrajectoryPlanner`, old NED priority-planner signals | Restore only metrics that map to current planner decisions. |
| Congested QoS configs | Old research used congested assets/evaluations. | WIP scaffolding | `envmod-200CAVs-qos-*`, `envmod-500CAVs-qos-*` | Run multi-seed validation before presenting benchmark conclusions. |

## 5. Missing Old Behavior

| Area | Old behavior | Current status | Evidence/files | Recommendation |
|---|---|---|---|---|
| Full delayed MCM counters | Old emitted `numDelayedMCM`, `numDelayedMCMnegotiation`, `numDelayedMCMexecution` based on delay thresholds. | Missing | old `McmService.ned`, old receive delay checks | Restore if delayed-message KPIs are needed; define threshold and subtype logic. |
| Per-second MCM subtype rates | Old emitted `numMCMPerSec`, `numIntentMCMPerSec`, `numNegotiationMCMPerSec`, execution subtype rates. | Missing | old `McmService.ned`, `scalarPrint.py` | Could be derived from vectors, but direct signals are not restored. |
| MCM periodicity metric | Old emitted `periodicityMcm`. | Missing | old send path emitted `scSignalPeriodicityMcm` | Restore if generation regularity must be a scalar KPI. |
| Number of transmitting vehicles | Old had `NumberOfTransmittingVehiclesMcmCounter`. | Missing | old `McmService.ned`, `scalarPrint.py` | Prefer deriving from modules with sent counts instead of old counter semantics. |
| DCC transmitted/dropped by profile/subtype | Old scripts consumed `packetsTransmitted*` and `packetsDropped*` metrics. | Missing/external | `scalarPrint.py`; likely DCC/radio layer source | Do not fake in `McService`; restore from DCC source if available. |
| CAM-specific metrics | Old scripts also consumed CAM counts, size, delay, periodicity. | Not in MCM tool scope | `scalarPrint.py` | Keep separate from MCM unless doing MCM+CAM coexistence analysis. |
| Full priority planner counters | Old emitted priority possible counters, trajectory type counters, affected NCV counters, trajectory costs. | Missing | old NED priority-planner block | Restore staged counters once planner decisions are stable. |
| Old fixed divisor for negotiation MCMs per vehicle | Old script used `McmNegotiationSentCounter / (NegotiationStartedCounter * 3)`. | Not restored intentionally | `scalarPrint.py` | Current one-CV/two-CV flows make fixed three-vehicle divisor unsafe. |

## 6. Metrics Restored vs Missing

| Metric | Current status | Notes |
|---|---|---|
| `McmSentCounter`, `McmReceivedCounter` | Restored | Emitted on actual send/receive. |
| Intent/Negotiation/Execution/Emergency subtype counters | Restored | Classification is based on decoded/current MCM metadata. |
| `msgsizeReceived` | Restored | Current receive-side size statistic. |
| `EteDelayMcm*` | Restored | Uses reconstructed generation timestamp; negative wraparound samples are guarded. |
| `dccTimeWaitNextMcm` | Restored | Emitted from DCC interval calculation. |
| `coopCBR` | Restored | Local CBR sampled/emitted through MCM service. |
| `ChannelLoad` / CBR | Extracted if present | Lower-layer radio statistic; analysis tool includes it. |
| `packetErrorRate` / PER | Extracted if present | Lower-layer radio statistic; analysis tool includes it. |
| `CoopVehicleAgeOfInformation` | Restored | Coordination-relevant negotiation/execution MCMs for listed participants only. |
| `negotiationTime` | Restored | First RV Request send to final required Accept, or missing-CV Execute evidence. |
| Negotiation MCMs per completed negotiation | Derived | `McmNegotiationSentCounter / NegotiationCompletedCounter` in aggregate output. |
| Negotiation MCMs per cooperating vehicle | Deferred intentionally | Needs role-specific counters or safe participant denominator. |
| `currentMCSoperatingMode` | Restored | Numeric mode signal. |
| Delayed MCM, rate, periodicity, DCC transmitted/dropped metrics | Missing or external | See Section 5. |
| Priority/planner/cooperation-cost metrics | Mostly missing | Planner logic exists; detailed research signals are not restored. |

## 7. Scenario Validation Status

| Scenario/config | Current status | Notes |
|---|---|---|
| `envmod-19CAVs-merging` | Validated baseline | Current expected target pairs are documented in README and checked in recent runs. |
| `envmod-19CAVs-emergency-lane-change` | Validated baseline | Emergency queue count and no `car_ml1_` emergency logs are tracked as smoke checks. |
| GUI variants | Configured | Should remain behavior-equivalent to command-line variants. |
| `envmod-200CAVs-qos-*` | Needs validation | WIP QoS scaffolding with Free Space baseline/adaptive/MCO/DCC-profile variants. |
| `envmod-500CAVs-qos-*` | Needs validation | Same as 200-CAV; avoid heavy runs during routine code tasks. |

The README now documents the distinction between the designated 19 coordination vehicles and 200/500-CAV background load vehicles. That distinction should remain explicit in future scenario changes.

## 8. Documentation Gaps

* Add a compact architecture diagram or sequence table for the current RV/CV message flow: Request, Offer, Confirm, Accept, Execute, Complete-as-Cancel.
* Document exactly which old research metrics are intentionally not restored yet, especially delayed-message and priority-planner metrics.
* Document the DCC profile mapping in a small table, including the default `dccProfiles=false` behavior.
* Add a short note that `timeGenMcm=DataProvider` remains the NED default for Intent behavior, while coordination timestamps use current simulation time.
* Keep the 200/500-CAV QoS configs labeled as experimental until multi-seed congested validation is complete.

## 9. Suggested Next Tasks

1. Restore priority/planner measurement counters in a focused metrics-only task: trajectory type, trajectory cost, possible priority, affected NCV count.
2. Add delayed MCM and periodicity metrics if they are required for the congested QoS tables.
3. Add role-specific negotiation MCM counters if per-cooperating-vehicle metrics are still required.
4. Validate `PeriodicFixedNoDCC` and DCC-profile mapping with short targeted smoke configs before using them in congested campaigns.
5. Design the missing Important Intent Sharing state before implementing the old exemption in adaptive rules.
6. Evaluate whether negotiation-related adaptive reduction should be restored, and keep it separate from Intent reduction.
7. Run 200-CAV and 500-CAV multi-seed QoS experiments only after the metric set is frozen.

## 10. Risks / Things Not To Change Blindly

* Do not copy the old monolithic `McmService` logic back into `McService`; current separation is a real improvement.
* Do not restore the old fixed `* 3` denominator for negotiation MCMs per vehicle; current scenarios include one-CV and two-CV flows.
* Do not treat background 200/500-CAV Intent MCMs as coordination participants.
* Do not merge Intent and coordination triggering paths; the old implementation kept them separate.
* Do not use RV Execute send time as a proxy for negotiation completion.
* Do not change DCC profile defaults while adding evaluation configs; old defaults differ from current preserved behavior.
* Do not fabricate PER/DCC transmitted/dropped metrics inside `McService`; use the lower-layer statistics source.
* Do not overclaim congested-scenario performance until multi-seed Free Space QoS runs are complete.
