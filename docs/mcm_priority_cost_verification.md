# Maneuver priority and cooperation cost verification

Verified 2026-10-20 against the current checkout and the supplied `reference/mcm_old` directory. Reference files were read only. Only the confirmed merging feasibility bug was fixed; priorities, formulas, search tiers, scenario settings and communication behavior were not changed.

## Highway merging — VERIFIED

Configuration: `envmod-19CAVs-merging`, run 0, 30 seconds, as used by `tests/mcm/test_mcm_scenarios.py`. `omnetpp.ini` configures the requesting route, trigger, candidate lane, freshness and gap selection through `*.node[*].middleware.MC.*`. Priority is **hard-coded by the scenario-specific application trigger**, not supplied by an INI priority parameter.

End-to-end path:

1. `McService` loads `MergingCoordinationConfig`; `McApplication::evaluateMergingRequestTrigger()` in `McApplicationMerging.cc` selects the lead/follow CV pair from current snapshots using `selectMergeGapTargets()`. This is geometric target selection, not scalar-cost minimization.
2. The trigger sets `command.priority = MediumPriority` and `mPriorityMcmCategory = MediumPriority` (enum value 1).
3. `McService::addManeuverNegotiationContainer()` serializes `command.priority` with `priorityToAsnPriority()`. ASN.1 `PriorityManeuver_medium` is 1.
4. `getMcmOperationMetadata()` and reception `extractMcmSnapshot()` in `McService.cc` preserve `priorityManeuver`. `McApplication::evaluateCvRequestResponse()` converts it with `priorityFromMcm()` and stores it in `mPriorityMcmCategory`.
5. `classifyCvMergingControlManeuver()` checks the existing planned trajectory with `check_traj_conflict_merging()`. No conflict is a feasible DoNothing candidate. Otherwise it passes the received numeric priority (1) to `findSuitableTrajectoryCV(..., mergingReqTraj=true, ...)`.
6. The planner tries LOW, then MEDIUM if needed, generating candidates with `calculateRefTrajectory()` and checking the same merging conflict helper. The first feasible candidate is selected. `calculateTrajectoryCost()` computes its reported cost afterward; it does not rank all candidates.
7. The classifier now returns explicit feasibility. `buildCvMergingResponse()` builds Reject on false; otherwise it builds Offer for two CVs or Accept for one CV, using the selected feasible trajectory. No failed result becomes a fallback Offer.
8. `handleReceivedOfferAsRv()` records the targeted CV Offers and queues Confirm after both arrive. `handleReceivedConfirmAsCv()` queues Accept for the same agreement. `handleReceivedAcceptAsRv()` starts execution after the expected Accepts. Completion uses the existing Cancel workaround and state reset.

Initial evaluations in the real run (all received/transmitted MEDIUM, planner level 1):

| RV / station | Request | CV / station | Entered tiers | Required level | Cost | Selected candidate | Result |
|---|---:|---|---|---:|---:|---|---|
| RV1 `car_ml1_1` / 29 | 37 | `car_hl0_1` / 169 | No search: outer no-conflict check | No planner result; effectively LOW | 0, inferred DoNothing | Existing planned trajectory | Feasible, Offer |
| RV1 / 29 | 37 | `car_hl0_2` / 309 | LOW → MEDIUM | 1 | 0.10 | Decelerate; speed fraction .25, deceleration 2.5 m/s², gap 1 s | Feasible, Offer |
| RV2 `car_ml1_2` / 449 | 116 | `car_hl0_3` / 589 | Inner planner no-conflict shortcut | 0 | 0 | DoNothing | Feasible, Offer |
| RV2 / 449 | 116 | `car_hl0_4` / 729 | LOW → MEDIUM | 1 | 0.13 | Decelerate; fraction .40, deceleration 2.5, gap 1 | Feasible, Offer |
| RV3 `car_ml1_3` / 869 | 157 | `car_hl0_5` / 1009 | LOW | 0 | 0.02 | Accelerate; fraction .05, acceleration .5, gap 1 | Feasible, Offer |
| RV3 / 869 | 157 | `car_hl0_6` / 1149 | LOW → MEDIUM | 1 | 0.11 | Decelerate; fraction .30, deceleration 2.5, gap 1 | Feasible, Offer |

A subsequent pre-issuance evaluation for CV 1149 reports .12 (fraction .35). These are separate evaluations as state advances, not a scalar threshold or a cost ranking. Issued Offer retransmissions retain their original proposals, as checked by the regression suite.

Selected pairs remain 169/309, 589/729, and 1009/1149. RV completion times are 11.384426574409, 15.709394050599 and 19.5209157028 seconds. RV1 execution starts at 7.417120259878. All three complete. MEDIUM materially matters: the identical RV1/CV309 planner input fails at LOW and succeeds at MEDIUM.

## Safety-critical lane change — VERIFIED

Configuration: `envmod-19CAVs-emergency-lane-change`, run 0, 30 seconds. The source vehicle sends EmergencyPriority execution Abort. The follower's cooperative lane change is a **separate HIGH-priority Request**.

1. `evaluateEmergencyBrakingTrigger()` creates the source Abort with `EmergencyPriority` (3). `handleReceivedEmergencyAsFollower()` recognizes it and arms the HIGH-priority follower maneuver.
2. `evaluateSafetyCriticalLaneChangeTrigger()` in `McApplicationLaneChange.cc` chooses target CVs and writes `command.priority = HighPriority` and `mPriorityMcmCategory = HighPriority` (2). Follower/target routes and trigger geometry come from `SafetyCriticalLaneChangeConfig` / the MC NED parameters, not an INI priority setting.
3. Request serialization/reception uses the same service functions as merging, preserving ASN.1 high=2.
4. `evaluateCvRequestResponse()` dispatches HIGH Requests on the target CV route to `evaluateCvCooperationDecision()` in `McApplicationCvDecision.cc`. Both CVs independently run this path.
5. `priorityLevel(mPriorityMcmCategory)` is 2 and is passed to `findSuitableTrajectoryCV(..., mergingReqTraj=false, ...)`. Search expands LOW → MEDIUM → HIGH as needed, using lane-change conflict checks on generated trajectories.
6. Acceptance requires `result.found`, `0 <= possiblePriorityLevel <= requestPriority`, and no acceleration blocked by a nearby slower leader. The leader block requires Accelerate, leader seen, distance < max(3 m, .5 × ego speed), and leader speed < ego speed. There is no scalar `cost <= threshold` test.
7. Feasible two-CV decisions produce Offers with the selected trajectories; failures produce Reject. RV waits for both Offers and both Accepts, then executes and completes through the existing completion/reset path.

Initial real evaluations (all received/transmitted HIGH, planner level 2):

| RV / station | Request | CV / station | Entered tiers | Required level | Cost | Candidate | Result |
|---|---:|---|---|---:|---:|---|---|
| RV1 `car_hl1_1` / 449 | 5 | CV1 `car_hl2_1` / 309 | Planner no-conflict shortcut | 0 | 0 | DoNothing | Feasible, Offer |
| RV1 / 449 | 5 | CV2 `car_hl2_2` / 589 | LOW → MEDIUM | 1 | .12 | Decelerate; fraction .25, deceleration .5, gap .7 | Feasible, Offer |
| RV2 `car_hl1_2` / 1149 | 244 | CV3 `car_hl2_3` / 869 | LOW | 0 | .04 | Accelerate; fraction .05, acceleration .5, gap .9 | Feasible, Offer |
| RV2 / 1149 | 244 | CV4 `car_hl2_4` / 1289 | LOW → MEDIUM → HIGH | 2 | .26 | Decelerate; fraction .45, deceleration 3.5, gap .5 | Feasible, Offer |

All four log `leaderSeen=0`, distance=999 and speed=99 (absence sentinels). Therefore none is blocked by the leader constraint. The logged .6 threshold is descriptive only. Selected pairs are 309/589 and 1289/869; both complete their ten-step lane changes at 13.590234857821 and 14.947175157582 seconds and subsequently reset on completion. RV3 also completes; its initial costs are .10 and .28, with a later reevaluation .26 for the rear CV.

HIGH materially enables stronger cooperation: the identical RV2/CV1289 input has no feasible candidate at MEDIUM, but HIGH finds the .26 candidate.

## Reference comparison

The supplied directory contains `McmService.cc/.h`, `McObject.cc/.h`, `VehicleController.cc/.h`, NED and scenario files. **It does not contain TrajectoryPlanner source/header.** Consequently its exact old cost formula, search implementation and any internal planner cost bound cannot be reconstructed from this directory. No claim of exact historical formula/tier equivalence is made.

| Behavior | Current | Supplied reference | Interpretation |
|---|---|---|---|
| No feasible merging candidate | Before fix could still Offer; now explicit false → Reject | `foundSuitableTraj == false` sets `SendReject` (e.g. McmService.cc:1715 and 1783) | Confirmed regression fixed |
| Feasible merging | Found/no-conflict → Offer or Accept | Same found/no-conflict branch structure | Preserved response semantics |
| Scalar cooperation cost | Calculated after selection and recorded/logged | `foundTrajCost` read from tuple element 3, emitted as SignalTrajectoryCost | Reporting role preserved in visible service logic |
| Numeric .20/.40/.60/.80 bounds | `costThresholdForPriority()` used only for diagnostic fields | No corresponding scalar acceptance comparison found in supplied code | Current reporting values; historical acceptance rule unproven |
| Required priority level | Planner tier output; lane-change wrapper additionally checks ordering | Tuple element 5 used for counters; visible service does not compare it against received priority | Wrapper guard is a current difference; planner internals absent |
| Search priority | Received priority selects tier expansion | Received priority is passed into `findSuitableTrajectoryCV()` for merging and lane change; calls restricted to 0/1/2 | Interface preserved; exact old tiers unavailable |
| Request priorities | Merging MEDIUM, safety-critical follower HIGH | `mMergingPriorityLevel=MediumPriority` and **`mPriorityLevel=MediumPriority`** in this supplied version (lines 238/243); comment mentions earlier HIGH | Scenario/research variant difference, not proof current HIGH is a regression |
| Multi-CV | Each addressed CV evaluates; RV waits for both | Same Offer/Confirm pattern after per-CV feasibility | Preserved at protocol level |
| Trajectory fallback | No infeasible candidate is offered after fix | Failure selects own trajectory for Reject state; no-conflict uses own trajectory | Reject content is distinct from offering a feasible candidate |

No optional `useCooperationCostEvaluation` parameter was added: the condition for implementing it was not established. There is no verified reference scalar rule to implement faithfully, and adding .2/.4/.6/.8 rejection would invent behavior. Cost ON/OFF comparison and an enabling INI example are therefore **not applicable**. The actual service path is `*.node[*].middleware.MC`, not `middleware.McService`. No unsupported INI setting is supplied.

## Emergency notification path — verified separately from cooperative planning

Current code:

- `McApplication::evaluateEmergencyBrakingTrigger()` in `McApplicationLaneChange.cc:64` selects the configured source vehicle. At `startTime`, it assigns `EmergencyPriority` to the pending command and application state, sets role `EmergencyV`, mode `ManeuverExecutionMode`, progress `SendExecute`, and subtype **Abort** (assignments around lines 188–212). `SendExecute` is state machinery; the transmitted subtype is Abort, not Execute or a cooperative Request.
- The command is serialized by `McService::addManeuverExecutionContainer()` and sent by `McService::sendMcm()` via BTP MCM port and GeoNetworking **single-hop broadcast (SHB)** (`McService.cc:1672–1680`). The packet carries the source's planned trajectory in the intention-sharing container. This is the current emergency-MCM representation, using execution-container Abort + EmergencyPriority rather than a dedicated emergency subtype.
- The source physically brakes through `setMaxSpeed(targetSpeed)` in `evaluateEmergencyBrakingTrigger()`. The validated INI sets start=12 s, target=.1 m/s, broadcast interval=.1 s and duration=15 s. Source braking remains enabled in the braking-only baseline while emergency broadcasts and follower coordination are suppressed. This is source emergency deceleration, not a cost-based cooperative decision.
- Received MCMs enter `McApplication::handleReceivedMcm()`, which calls `handleReceivedEmergencyAsFollower()` (`McApplicationLaneChange.cc:638`). Radio reception is broadcast to surrounding stations; reaction is selectively gated. The handler requires the configured follower route, a different vehicle from the source, an execution-container Abort with EmergencyPriority, nonempty trajectories, the same lane, and the emergency vehicle ahead. It does not require the Abort's placeholder participant ID to address the follower. Other-route vehicles do not enter this follower-reaction path; same-route vehicles failing the lane/ahead gates ignore it. Repeats do not re-arm an already armed follower.
- Qualifying followers store `mEmergencyReceived`, switch to `CheckForCoordination`, and assign **HighPriority** to their own lane-change planning state. `evaluateSafetyCriticalLaneChangeTrigger()` subsequently generates a distinct cooperative Request if its target-lane conflict conditions require coordination. Nearby target CVs evaluate that HIGH Request, not Emergency as a search tier. Gap/TTC values at the initial notification handler are diagnostics; same-lane/ahead gates arm it. The existing fallback brake for failed HIGH coordination is separate from the source notification.

Observed coordinated-scenario evidence: source `car_hl0_Emergency` / station 29 logs braking to maxSpeed=.1 and a sent EmergencyPriority execution Abort at 12.084426574409 s. `car_hl1_1` / 449 logs receiving/arming from that notification at 12.110218240124 s and subsequently sends HIGH Request 5. Followers `car_hl1_2` / 1149 and `car_hl1_3` / 1849 also log qualifying emergency reception and complete their separate cooperative maneuvers. Source broadcasts continue at the configured cadence. Receiving a notification does not mean every surrounding vehicle performs a lane change.

Reference evidence supports the same architectural distinction:

- `reference/mcm_old/McmService.cc:936–947` brakes `car_hl0_Emergency` to .1 m/s, assigns role EmergencyV, subtype Abort, EmergencyPriority and execution mode, and generates an emergency deceleration trajectory (8 m/s² argument).
- `McmService.cc:2327–2341` explicitly identifies Abort as the emergency notification representation and handles execution subtype 7 with priority 3. Its follower processing checks the emergency trajectory and arms coordination when relevant (lines 2371–2407). The reaction uses the separate `mPriorityLevel`, whose supplied default is MEDIUM; the current validated follower scenario uses HIGH.
- Visible cooperative planner call sites restrict negotiation priorities to 0/1/2 (e.g. lines 1627–1634 and 1727–1729). The reference therefore supports LOW/MEDIUM/HIGH cooperative planning versus Emergency notification. Its missing TrajectoryPlanner source still prevents a full historical planner-internals comparison; it does not undermine this visible communication/state distinction.

## Priority semantics

`priorityMcmCategory` in `McApplication.h` uses these exact names (not Desired/Necessary/SafetyCritical):

Low, Medium and High are cooperative planning priorities. Emergency is a separate safety notification/state used to communicate an ongoing emergency maneuver to surrounding vehicles; it is not a cooperative search tier.

| Cooperative priority / enum value | Planner level | Active search tiers | Verified behavior |
|---|---:|---|---|
| LowPriority / 0 | 0 | LOW only | Rejects the replayed merging CV309 and lane-change CV1289 inputs |
| MediumPriority / 1 | 1 | LOW then MEDIUM if needed | Offers for merging CV309; rejects lane-change CV1289 |
| HighPriority / 2 | 2 | LOW then MEDIUM then HIGH if needed | Offers for both replayed inputs; lane-change CV1289 requires HIGH |

The diagnostic `costThresholdForPriority()` helper returns .20/.40/.60 for these three cooperative priorities. These are reporting values, not scalar acceptance bounds.

**EmergencyPriority (enum value 3)** is separate from cooperative candidate-search escalation. It notifies surrounding vehicles of an emergency maneuver through the emergency/safety communication path described above. Although shared mapping utilities return 3 from `priorityLevel(EmergencyPriority)` and .80 from `costThresholdForPriority(EmergencyPriority)`, neither value defines an Emergency cooperative planning tier. Its absence from the cooperative search is intentional architecture, not missing functionality.

**NoPriority (enum value 4)** is a sentinel, not a cooperative planning priority. Its mapping utilities return -1/0. It has no wire representation; serialization falls back to LOW, and reception of an unrecognized wire value defaults to MEDIUM.

The active candidate tables in `findSuitableTrajectoryCV()` are:

| Tier | Speed reduction fractions | Deceleration m/s² | Time-gap values s |
|---|---|---|---|
| LOW | .05, .10, .15, .20 | .5, 1, 1.5, 2 | 1, .9 |
| MEDIUM | .25, .30, .35, .40 | .5, 1, 1.5, 2, 2.5, 3, 3.5, 4 | 1, .9, .7, .6 |
| HIGH | .45, .50, .55, .60 | 3.5, 4, 4.5, 5 | 1, .9, .8, .7, .6, .5, .4, .3 |

Acceleration uses .05/.10/.15/.20 speed increases and .5/1/1.5/2/2.5/3 m/s² for all priorities, combined with tier-specific time-gap search where applicable. LOW is always tried first; later tiers run only when earlier ones fail. `possiblePriorityLevel` is a search/category label, not the scalar cost rounded into a priority. Some acceleration/time-gap branches assign lower required levels within a higher search tier.

Merging acceptance is feasibility within its permitted search; lane change additionally checks required level and the leader constraint. A feasible LOW result remains acceptable to MEDIUM/HIGH. The planner chooses the first feasible candidate in loop order; neither path minimizes `cooperationCost`.

## Cost formula and trajectory checks

`TrajectoryPlanner::calculateTrajectoryCost(PlannedTrajValues, bool isRouteAffected)`:

```
(speed_change
 + (positive deceleration_change, else positive acc_change, else 0) / 10
 + (lane_change ? (isRouteAffected ? .4 : .2) : 0)
 + (time_gap_change < 1 ? 1 - time_gap_change : 0)
 + 0 /* TTC */) / 5
```

Speed change is a fraction, not m/s. Acceleration/deceleration is in m/s² divided by the literal 10. Time gap is numerically compared with the 1-second baseline. The result is a dimensionless inconvenience score by convention; the implementation mixes these normalized numerical components rather than enforcing dimensional analysis. All five components have equal divisor 5; priority does not change weights. TTC contribution is always zero and `TTC_change` is not read by this formula. Both CV paths call the formula with `isRouteAffected=false` and disable candidate lane changes, so lane-change cost is zero in these runs.

Candidate generation feeds the actual `TrajectoryPlanner` conflict methods in `TrajectoryConflict.cc`. Merging checks endpoint Euclidean distance against time gap × speed, with its existing received-age compensation. Lane change samples six indices with lateral overlap <2.5 m and distance <time gap × speed. These are the actual cooperative feasibility checks, not an isolated scalar exercise.

`check_traj_collision()` provides a relative-speed TTC test, but there are **no callers** in current source. Reserved TTC-reduction arrays in the CV search are unused. Separate execution logic does use live time-gap/TTC safety guards. Thus the cost/priority search must not be described as actively testing every candidate with the TTC collision helper.

## Priority reads and writes audit

A complete textual occurrence inventory for the application/service was captured in `/tmp/mcm-priority-read-write-audit.txt`; the final application-only inventory is `/tmp/mcm-priority-consumers-final.txt`.

| Location/functions | Role / control-flow effect |
|---|---|
| McApplicationMerging.cc: evaluateMergingRequestTrigger | Writes command and RV state MEDIUM |
| McApplicationLaneChange.cc: evaluateSafetyCriticalLaneChangeTrigger, handleReceivedEmergencyAsFollower, evaluateEmergencyBrakingTrigger | Writes HIGH cooperative state or Emergency source Abort; Emergency receive guard changes execution |
| McService.cc: priorityToAsnPriority, addManeuverNegotiationContainer, addManeuverExecutionContainer | Wire serialization; NoPriority falls back to low |
| McService.cc: getMcmOperationMetadata, snapshot construction | Reads containers and transfers priority to ReceivedMcm |
| McApplication.cc: priorityFromMcm, evaluateCvRequestResponse | Converts/stores received priority; HIGH selects the specialized evaluator; smoke-reject gate reads HIGH |
| McApplicationMerging.cc: classifyCvMergingControlManeuver | Received 0/1/2 selects real planner tier input |
| McApplicationCvDecision.cc: evaluateCvCooperationDecision | Converts level, calls planner, gates required level; threshold assigned/logged only |
| TrajectoryPlanner.cc: findSuitableTrajectoryCV | Reads numeric priority to enable MEDIUM/HIGH searches; computes candidate costs after feasibility |
| McApplicationNegotiationCommon.cc: makeRvSecondRequestCommand | Uses priority for estimated rejection adaptation (.2/.4/.5 speed fractions and 2/4/5 decelerations); carries priority into new Request |
| McApplicationNegotiationCommon.cc, McApplicationRetry.cc, McApplicationExecutionProgress.cc | Copies active priority to follow-ups/execution; HIGH active-request helper selects timing/retry/failure paths |
| McApplication.cc, McApplicationFreshness.cc, McApplicationExecutionControl.cc | HIGH selects lane-change state, freshness/execution and fallback/control paths; several other reads only gate lane-change diagnostics |
| McService.cc: selectMcmDccProfile and emergency classification | Priority changes configured transport profile/classification; audited, unchanged |
| McmEnumUtils.h: priorityName, diagnostic log calls and planner measurement counters | Display/measurement only; these reads do not decide acceptance |

Cosmetic/dead portions: `costThresholdForPriority()`/.threshold are reporting-only; `requestPriority` formal parameters in `findSecondRequestTrajRV()` and unused `newPlannedTrajRV()` are not consumed in those bodies; `getMinGapPriority()` (.8/.6/.3) has no current callers. This does not make the actual cooperative Request priority cosmetic: its tier-selection branches are active and experimentally verified.

## Bug fix and focused regression

`classifyCvMergingControlManeuver()` now returns bool. Missing planning prerequisites or `result.found=false` return false. No-conflict explicitly selects the current trajectory and returns true; feasible planner results store the selected trajectory and return true.

`evaluateCvRequestResponse()` passes this explicit result to `buildCvMergingResponse()`. False sets Reject/SendReject and never constructs Offer content. True uses the selected trajectory and the existing one-/two-CV subtype. Existing common Reject packet construction may carry the CV's own planned trajectory; that is Reject wire content, not a fallback Offer.

`tests/mcm/mcm_timing_tests.cc` adds deterministic response-boundary coverage with a controlled planner result: false plus nonempty result/fallback content → Reject; true plus selected trajectory → Offer preserving that trajectory; one-CV false → Reject. This injects the planner result at the response boundary rather than mocking SUMO or changing candidate search. Actual planner failures are additionally demonstrated by the identical-input runtime replay below.

## Counterfactual replay

Temporary diagnostics replayed the exact same Request trajectory, vehicle state, route reference, timing, direction flags and planning arguments with cooperative priorities LOW/MEDIUM/HIGH (numeric 0/1/2). The original diagnostic capture also included a direct numeric-3 probe; that out-of-domain call is not evidence about intended cooperative planning and is excluded from this comparison. No probe applied a vehicle command or changed scenario configuration. Entered search tiers and returned results were recorded; diagnostics were then removed and the final core rebuilt. Assertions on the replay results passed.

| Same input | Priority / level | Entered stages | Found | Required level | Cost | Feasibility/priority decision |
|---|---|---|---|---:|---:|---|
| Merging RV1/CV309, req 37 | LOW / 0 | LOW | No | 10 | N/A | Reject |
| same | MEDIUM / 1 | LOW → MEDIUM | Yes | 1 | .10 | Offer |
| same | HIGH / 2 | LOW → MEDIUM | Yes | 1 | .10 | Offer |
| Lane change RV2/CV1289, req 244 | LOW / 0 | LOW | No | 10 | N/A | Reject |
| same | MEDIUM / 1 | LOW → MEDIUM | No | 10 | N/A | Reject |
| same | HIGH / 2 | LOW → MEDIUM → HIGH | Yes | 2 | .26 | Offer; live leader block absent |

Failed results return cost=0 as a default; it is **not** a calculated zero-cost feasible candidate. The LOW/MEDIUM lane-change rows exercise the planner/evaluator acceptance boundary; they do not change the application's HIGH-specific dispatch.

Emergency is not evaluated as a cooperative planner tier because it represents emergency-maneuver notification rather than cooperative trajectory selection.

Cost OFF means the actual existing behavior above (there is no switch). Cost ON results are unavailable because no verified historical scalar rule exists in the supplied files. A hypothetical comparison against reported .4/.6 would be fabricated semantics and was deliberately not implemented.

## Regression and evidence

The five original scenario regressions pass with the fix: coordinated merging; merging baseline; coordinated safety-critical lane change; emergency baseline; forced rejection followed by a second Request. Each runs config run 0 for 30 simulated seconds. `mcm_timing_tests` also passes, including the new focused merging coverage.

Initial uninstrumented evidence:

- `/tmp/mcm-regression-envmod-19CAVs-merging-20uyn_wo/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-zu56zmmi/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-baseline-4kyujvy7/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-baseline-lasyfo28/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-second-request-smoke-v21w4pat/simulation.log`

Replay evidence:

- `/tmp/mcm-regression-envmod-19CAVs-merging-dehjaz3o/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-g_k1g06a/simulation.log`
- `/tmp/mcm-counterfactual-merging.json`
- `/tmp/mcm-counterfactual-emergency-lane-change.json`
- `/tmp/mcm_priority_instrument.py` records the temporary instrumentation procedure.

No changes to reference/, INI/NED, traffic, trajectories, priority assignments, search tiers, formulas, DCC, EDCA, timing or retransmission behavior. No commit or push.

Final uninstrumented rerun after explicitly storing the no-conflict candidate (all five pass, 37.844 seconds wall time):

- `/tmp/mcm-regression-envmod-19CAVs-merging-4m6j1cvm/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-bdomb88y/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-baseline-0f7jd1bz/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-baseline-mrey3s9c/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-second-request-smoke-1uptp3sk/simulation.log`
- Focused C++ result: `/tmp/mcm-priority-focused-tests.log`.

## Final Emergency semantics cleanup and commit-readiness check

This cleanup changes only this report. Runtime sources, test code and reference files are unchanged from the completed priority/cost task. SHA-256 comparisons confirm the three previously modified runtime files and focused test file are byte-for-byte unchanged. The overall task's only runtime behavior change remains infeasible merging candidate → Reject.

After the documentation correction, the focused C++ tests and all five scenario regressions pass again (38.069 seconds for the scenario suite). Emergency notification, follower reaction, coordinated completion, baseline braking/suppression and rejection/second-Request behavior remain verified. `git diff --check` passes. No commit or push was performed; the task is ready for review and commit.

Cleanup verification evidence:

- `/tmp/mcm-emergency-cleanup-focused.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-ekj531bl/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-3i539tni/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-baseline-xdgs0qwk/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-baseline-j9i92nl2/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-second-request-smoke-fljkrcxi/simulation.log`
