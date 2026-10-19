# MCM trajectory timing audit — final schema-preserving design

Current checkout audited on 2026-10-19. No AGENTS.md was found in the repository,
relevant source/test subdirectories, or ancestors checked. The original research
files were read only. Nothing was staged, committed or pushed.

## Final invariant

**For every live trajectory transmitted in a newly generated MCM, the trajectory
is referenced to that MCM's generationDeltaTime. Receiver-side age is therefore
evaluation time minus reconstructed generationDeltaTime.**

No additional trajectory-origin timestamp is sent. The new planned/requested/
offered reference-time ASN.1 extension has been removed completely. Both
PriMa-MCM.asn files and McmParameters.h match the pre-task submodule revision.
The existing absolute-centisecond trajectory-point encoding is restored.

Two cases are deliberately distinct:

1. A **newly generated message** contains freshly calculated live predictions
   referenced to its current generation timestamp.
2. An **exact issued Request/Offer retransmission** repeats the entire immutable
   original MCM, including its intention trajectory and original
   generationDeltaTime. It is never rebuilt with a newer header timestamp.

Confirm/Accept agreement snapshots are not live predictions. Their copies
preserve the previously issued proposal; their message's intention trajectory is
fresh. Subtype semantics make this distinction without another ASN.1 field.

## Verified original defects and retained fixes

- Application evaluation used `max(0, now - receivedAt)`, excluding network and
  queue age on immediate reception. All prediction age now starts at the
  reconstructed header generation time, including cached reevaluation.
- RV retries reused stored content with a fresh timestamp. They now repeat the
  original complete packet and timestamp, while a separate live candidate keeps
  updating locally.
- Initial Confirm and Confirm retry could carry different RV trajectories.
  Both now echo the actual issued Request.
- Initial merging Offer could advertise intent despite choosing another CV
  maneuver. Offer uses the selected contribution and freezes it after issuance.
- Offer retry could change the RV echo; Accept retry could change cooperation
  metadata/participant count. The chain now retains the issued content and
  protocol values. Request retry also copies its complete original command,
  rather than rebuilding cooperationType from a different local enum.
- General/lane-change first-point compensation compared point zero with itself,
  yielding zero correction. It now estimates speed from the first segment.
- Merging checks retained stale conflict evidence after the compensated endpoint
  was safe. They now compare the corrected endpoint once. The second-request
  lane-change-index search also consumes information age exactly once.
- Route cursor advancement previously depended on sends, and serialization made
  an independent intent plan. Cursor/intent refresh on service updates; actual
  new-message generation refreshes state and predictions again after any wait.

## Time and wire contract

| Time | Meaning |
| --- | --- |
| `McEgoContext::sampledAt` | Provider's latest available vehicle-state sample time. |
| `McEgoContext::now` | Current planning/evaluation time. |
| New live plan origin | Current generation time, at the header's 1 ms resolution. Latest available controller/provider state is used to calculate a new plan. |
| `generationDeltaTime` | Generation time in Vanetza-clock milliseconds modulo 65536 (65.536 seconds), epoch 2004-01-01. Preserved on exact packet retransmission. |
| `ReceivedMcm::generatedAt` | Signed simulation-time reconstruction of that single header timestamp. |
| `ReceivedMcm::receivedAt` | Arrival time, retained separately; not the origin of prediction age. |
| Evaluation age | Evaluation time minus `generatedAt`; fractional evaluation precision is retained. |
| Point `deltaTime` | Existing absolute simulation-centisecond sample time, not a new origin field and not used to infer message age. |
| Internal `TrajPointMCM::mReferenceTime` | Local generation/planning invariant only; never serialized as another timestamp. |

New ego plans use the latest available state as the current state estimate.
They do not reuse a previously cached trajectory and simply change its origin.
Sampling time remains visible locally for diagnostics. A plan's first point is
one configured point interval after its generation reference; the header itself
has millisecond precision.

`sendMcm` refreshes ego context before preparing new content. Before submission,
runtime checks reject a live intent, new Request, or new Offer contribution
whose internal origin differs from that message's generation reference.
Facilities-DCC waiting **before** generation is handled by replanning at the
actual opportunity. MAC/DCC waiting **after** generation, network delay, and
subsequent cache residence are all included by subtraction from the header time.
A retransmission's age additionally includes all elapsed time since original
issuance, including waits between retransmissions.

The legacy `timeGenMcm` parameter stays accepted for existing configurations, but
cannot select an inconsistent sampling timestamp for a new MCM. New-message
headers identify generation. No eligibility/rate/DCC algorithm was changed.

## Proposal, retry and execution lifecycle

| Message/content | Policy |
| --- | --- |
| Live intention | Recalculated each configured service update, and again immediately before new-message generation. |
| Initial/new Request | Current-state candidate, current generation timestamp. Pending unissued content refreshes across deferred service ticks. |
| Issued Request retry | Complete original MCM and generationDeltaTime repeated unchanged. Separate local live candidate may update; issued trajectory cannot mutate. |
| New Offer | Current CV state and current Request age used for candidate selection; selected CV trajectory references the Offer's current generation timestamp. Requested copy is the old RV agreement echo. |
| Issued Offer retry | Complete original Offer MCM repeated unchanged, including original intent, selected contribution, RV echo and header timestamp. |
| Confirm, including retry | Fresh intention, fresh header; requested trajectory is the immutable issued RV agreement snapshot. |
| Accept, including retry | Fresh intention, fresh header; requested trajectory echoes the RV snapshot and offered trajectory preserves the CV's issued contribution. These are agreement snapshots. |
| Reject | Fresh intention for the alternative-request planner; copied negotiation information is not treated as a newly planned prediction. |
| Execute | Fresh remaining prediction and current generation timestamp; cooperation ID, participant set and fixed accepted maneuver remain unchanged. |
| Completion/cancellation | Existing protocol path, with fresh intention; no new ASN.1 category or timing fields. |

`McmRetransmissionCache` holds at most one Request and one Offer. It keeps the
original immutable shared ASN.1 object, avoiding a codec-based deep copy.
Replay validates identity, targets, priority, cooperation type and trajectory
snapshots against the issued command. Changed content cannot enter this path.
A fresh proposal replaces the corresponding retained packet. Application timeout
and cancellation/reset paths invalidate obsolete pending commands; a retired CV
Request cannot reopen the old chain through a delayed retransmission.

A materially new proposal follows the existing new Request ID path, including
the rejection/alternative Request. Old Offer/Accept flags and stored CV
contributions are cleared for **all** participants. Each CV's Accept must match
that CV's stored Offer and the current RV Request. Comparisons use the existing
wire coordinate/heading/absolute-point-time quantization, not a new timestamp.

The existing 8-bit Request ID is not reused locally within the 32.768-second
reconstruction window. This prevents a delayed old Execute, which has only its
cooperation ID, from aliasing a later agreement. Exhaustion is explicit; IDs
are not silently recycled. Normal request selection keeps the preferred existing
ID when it is available.

Receiver code age-compensates intention and a Request's requested prediction.
Offer/Confirm/Accept trajectory copies are compared/stored as agreements, not
used as fresh age-compensated predictions. `requestedAge()` explicitly rejects
non-Request snapshots, preventing accidental use of a Confirm's newer header to
pretend its older agreement is a fresh prediction.

Negotiated RV/CV trajectories remain fixed until completion/reset/failure.
Remaining execution prediction uses current state, established route/lane target
and selected speed/rates, with the existing execution planner and unchanged
configured point count/spacing. No per-tick renegotiation is introduced.

## Receiver reconstruction and compensation

The existing Timer already reconstructs the nearest modulo-65536 millisecond
epoch. The MCM wrapper validates raw range before narrowing, rejects exact
half-period ambiguity, diagnoses genuinely future values, and preserves signed
generation times before simulation zero. It never silently clamps negative age.

Timer's microsecond simulation origin is used consistently during conversion,
so fractional reception-event remainders cannot contaminate reconstructed
message times. Evaluation still uses full simulation precision. Information
older than 32.768 seconds cannot be uniquely disambiguated by this field alone;
normal packet/negotiation lifetimes are substantially shorter.

The verified example is:

| Generation | Reception | Evaluation | Age | Correction at 27 m/s |
| --- | --- | --- | --- | --- |
| 10.000 s | 10.100 s | 10.100 s | 0.100 s | 2.7 m |
| 10.000 s | 10.100 s | 10.150 s | 0.150 s | 4.05 m |

Compensation remains **directional segment speed × information age**, not
time-aligned interpolation. General/lane-change/collision checks support
negative SUMO Y; the corresponding merging branch additionally supports positive
SUMO X. The existing final-segment direction gate is retained. Correction is
strictly **above** 50 ms; exactly 50 ms receives none. Empty/one-point trajectories
have no segment-speed correction; nonpositive segment-time differences do not
produce a speed. First point uses the forward segment; others the preceding
segment. Curves and lateral motion retain this documented scalar-speed/axis
approximation, without arbitrary lateral extrapolation. Source caches are not
shifted in place, avoiding double compensation.

## Runtime content audit and minimal codec decision

The audit follows McService's fill/extraction functions into McApplication's
merging, lane-change, CV evaluation, agreement handlers and TrajectoryPlanner.
Extraction into McmSnapshot alone does not imply an algorithmic dependency.

| Category | Wire fields | Actual consumer / meaning |
| --- | --- | --- |
| Predicted trajectory | intentionSharingContainer.plannedTrajectory; negotiation requestedTrajectory and offeredTrajectory | RV conflict checks, CV candidate evaluation, immutable proposal comparison; execution remaining prediction is the live planned trajectory |
| Point geometry | deltaLongitudinalPosition, deltaLateralPosition | Despite their names, absolute SUMO X/Y in integer metres; distance, conflict, collision and candidate planning |
| Point time | deltaTime | Absolute simulation centiseconds; adjacent differences estimate speed; no separate point-speed field |
| Point orientation | deltaHeading | Signed 0.1 degree heading, extracted as radians; TrajectoryGeneration's received-trajectory initial yaw uses it, although distance checks use X/Y |
| Time origin | generationDeltaTime | 65536-millisecond modulo timestamp; full generation-to-evaluation age for live predictions |
| Sender | header.stationID | RV/CV association, per-sender cache, response selection and execution sender validation; sender is implicit requesting vehicle on Request, CV on Offer/Accept |
| Request identity | requestID | Existing 8-bit proposal identity, retained across Offer/Confirm/Accept, replaced for materially changed RV proposal; no invented version field |
| Agreement identity | cooperationID | Execution agreement association, matched to accepted request identity |
| Participants | negotiationVehicleID1, optional negotiationVehicleID2, numberOfVehicles; execution cooperationVehicleID1, optional cooperationVehicleID2 | Addressing, target membership, separate CV contributions/Accepts and agreed execution participants; CV responses address the RV |
| Protocol metadata | mcmCategory, priorityManeuver, container presence | Message dispatch, mode, priority/cost decisions and emergency classification |
| Current lane | optional intentionSharingContainer.laneID | Merging/lane-change CV selection and emergency same-lane gate |
| Current dynamic state | intent speed.speedValue | Read for diagnostic speed/gap/TTC reports, including emergency diagnostics; those TTC values are not the emergency decision gate. Remote trajectory speed for planning comes from point geometry/time |
| Basic position / heading | basic referencePosition latitude/longitude; intent heading.headingValue | Extracted and preserved, but not used to position remote conflict trajectories or select candidates; local ego provider/controller supplies ego state |
| Static vehicle data | stationType, vehicleLength, vehicleWidth, automation level; confidence and driveDirection fields | Sent by McService; not read by the application planner. Length/width use unavailable values, not collision geometry. No CAM field dependency is inferred |

`cooperationTypeMCM` is a **mandatory ENUMERATED** with agreementSeeking(0),
prescriptive(1), interceptionLevel(2), roadSafetyLevel(3). The runtime extracts
and carries it into responses/retries; no decision branches on it for candidate
selection, priority, execution or message classification. It remains an existing
mandatory wire field, tested for preservation. No codec repair is retained for
this field's sake.

The runtime populates optional intent roadID/laneID/laneIDEnd together. Only
laneID is consumed for coordination. The optional acceleration, yawRate,
vehicleOrientationAngle and lanePosition fields are not populated or used.
Negotiation alternativeTrajectory, reasonRequest, reasonReject, laneID,
laneIDTrajEnd and cooperationCostTrajectory are not populated by the runtime;
alternative presence/count is only extracted. They are not prerequisites for
this task. The optional second-participant IDs and the negotiation/execution
container presence are genuinely required.

### Controlled descriptor removal

The focused fixture uses original-schema runtime-shaped Request, Offer, Confirm,
Accept and Execute packets, plus intention-only packets, with one and two CVs, nonzero sender/participant and
request/cooperation IDs, lane fields, current speed/heading/geographic state,
and 20-point trajectories. Request has the real zero offered placeholder;
Offer/Accept address the RV without a fabricated second target. No unused
optional metadata is populated. The artificial simultaneous negotiation plus
execution-container fixture was removed.

Each descriptor was restored to submodule HEAD independently while keeping the
other fixes, rebuilding the codec and rerunning the same byte test:

| Descriptor restored | Runtime failure | Retained minimum |
| --- | --- | --- |
| All four original | Runtime packet byte decoding fails | Original codec is insufficient |
| IntentSharingContainer.c | Decoded laneID is absent | Original optional map 7–15, required for laneID and bit alignment |
| ManeuverNegotiationContainer.c | Runtime negotiation byte decoding fails | Original optional map 6–11,14, needed for trajectory/participant decoding |
| ManeuverExecutionContainer.c | One-CV Execute byte decoding fails (two-CV alone had passed) | Original optional map 4, needed to encode absence/presence of cooperationVehicleID2 correctly |
| McmParameters.c | One-CV Execute byte decoding fails | Original optional map 2,3 with existing root/extension placement, needed for negotiation/execution containers |

All four are therefore required for actual runtime shapes, not an artificial
test of unused descriptive metadata. An OPTIONAL presence bitmap must include
every schema-defined optional slot, even when absent; deleting the unused slots
would change bit positions and the wire schema. The four changes remain exactly
**8 insertions, 4 deletions**, adding only missing original-member maps/counts.
There are no schema/header changes, timing fields or unrelated codec repairs.
Both original ASN.1 copies and McmParameters.h match HEAD. Vanetza cannot be
completely reverted while preserving these runtime-critical byte round trips.
Shared-object simulation delivery can mask byte-codec defects, so scenario
success alone is insufficient evidence.

### Decoded trajectory validation

The byte tests assert all 20 point coordinates, centisecond times, signed
headings, sender, request/cooperation identity, participant membership, lane,
speed, heading, position and serialized geometry representation. The actual
McService `decodeReceivedMcm` extraction supplies the decoded live planned/Request vectors to the
real TrajectoryPlanner `check_traj_conflict` and `check_traj_collision` methods.
Both require the nonzero generation-derived age to detect the fixture conflict;
zero age is negative control. Cached reevaluation increases correction.
The 7-metre/0.25-second wire segments estimate 28 m/s, demonstrating integer
coordinate quantization; the separate 27 m/s timing tests retain the exact
2.7 m / 4.05 m examples. No alternate collision model was added.

## Research reference and scope

Read-only research versions were found under
`/home/maksimovski/Desktop/artery_mcm/src/artery/application/`, with another local
checkout under `Desktop/artery`. The research service reconstructs generation
DeltaTime and passes generation-to-reception delay to its directional planner;
its CoordinationRequired path refreshes Request content. The refactored age
bug is not evidence that the dissertation simulations had the same defect.

Middleware updates follow configured `updateInterval` with initial jitter;
the supplied scenarios set 0.1 s. Trajectory point spacing is independently
0.25 s, with 20 points (5-second horizon). These are not universally the service
or message generation rates. DCC/LIMERIC/EDCA, priorities/traffic mapping, cost
thresholds, generation policy, background traffic, scenario configurations,
animation assets, horizon, point count and spacing were not edited.

Remaining limits: scenario-specific geometry and directional compensation;
original absolute-centisecond point-time range (0..30000, so long simulations
past its roughly 300-second range still require a separate design); nearest-
epoch ambiguity at long age; no multi-seed/network-delay sweep. Existing CV
lateral execution remains a state-only placeholder. Existing execution-container
full ASN.1 validation is still skipped because of the CooperationID descriptor
issue; UPER round trips are tested. The retransmission cache does not depend on
Vanetza's OER-based deep-copy operation.

## Focused tests and regressions

`mcm_timing_tests.cc` tests header-derived immediate/cached/fractional age,
normal/wrap/malformed/future/ambiguous timestamps, negative simulation generation
time, the unchanged-schema UPER codec (including negotiation/execution optional
participants), byte-identical Request/Offer retransmission and original age,
refusal of stale live generation and changed replay content, simulated multiple-
tick RV/CV DCC deferral, Confirm/Accept snapshots, immutable multi-CV proposals,
old-response invalidation, participant-count consistency, ID reuse protection,
execution agreement stability, and threshold/degenerate/first-point compensation.

The five 30-second scenarios test merging, safety-critical lane change,
rejection followed by a second Request, and both baselines. Added assertions
verify fresh generation/header alignment, complete retained trajectory sets on
retries, agreement hashes/cooperation type across the chain, and advancing
execution-prediction generation time. Lane-change assertions require the same
CV pair and preserve the actual target-slot ordering throughout the chain;
they do not force cache arrival order by editing the scenario or planner.

Final build/test results and exact Git review state follow below.

## Final verification results

- Build: `cmake --build build --target mcm_timing_tests -j 4` — passed.
- Focused CTest: `mcm_timing` — 1/1 passed.
- Dedicated `build/mcm_timing_tests --runtime-roundtrip` — passed; original-schema byte round trips, extracted live trajectory conflict/TTC checks and snapshot preservation.
- All five 30-second scenario regressions — passed; 38.412 seconds total wall time.

| Scenario | Result | Simulation run wall time |
| --- | --- | --- |
| Coordinated safety-critical lane change | PASS | 9.76 s |
| Coordinated highway merging | PASS | 3.54 s |
| Emergency baseline | PASS | 9.55 s |
| Merging baseline | PASS | 1.59 s |
| Rejection followed by second Request | PASS | 9.82 s |

Both main and Vanetza `git diff --check` pass with no output. Schema/header
comparison against submodule HEAD also passes with no output. Existing INET
static serializer shutdown warnings do not fail the focused test.

The requested invariant holds for newly generated live predictions. Exact
issued Request/Offer packets retain the original generation timestamp; copied
Confirm/Accept agreements remain snapshots. No timestamp extension is present.
Four independent existing-field codec-map corrections are the only remaining
Vanetza modifications. No README/scenario/animation assets were changed.

Nothing was staged, committed or pushed. Changes are ready for review.

## Final pre-commit scope review

All changed and untracked files were reviewed against the accepted timing task.
No category E (temporary/debug artifact) or F (unrelated/pre-existing change)
was found in either working tree. No repository file needed removal. Existing
MCM-WIRE timing/digest fields are retained because the required scenario tests
use them; warnings are limited to invalid timestamp/proposal events. No runtime
or test behavior was changed during this final scope review.

| Category | File (repository-relative) | Reason |
| --- | --- | --- |
| B | CMakeLists.txt | Registers the focused timing test target and CTest |
| A | src/artery/application/CMakeLists.txt | Builds the freshness implementation |
| A | src/artery/application/McService.cc | Generation-time refresh, immutable retries, receive-time reconstruction and trajectory extraction |
| A | src/artery/application/McService.h | Owns the immutable retransmission cache |
| A | src/artery/application/McService.ned | Documents the accepted generation timestamp contract for legacy configuration |
| A | src/artery/application/mcm/McApplication.cc | Integrates refresh and validates proposal/participant consistency |
| A | src/artery/application/mcm/McApplication.h | Timing metadata and immutable proposal state |
| A | src/artery/application/mcm/McApplicationCvDecision.cc | CV conflict checks use full Request age |
| A | src/artery/application/mcm/McApplicationLaneChange.cc | RV lane-change checks use generation-derived age and retain issued Request |
| A | src/artery/application/mcm/McApplicationMerging.cc | Merging age checks and current-state candidate bookkeeping |
| A | src/artery/application/mcm/McApplicationNegotiationCommon.cc | Agreement snapshots, alternative Request age and reset/invalidation |
| A | src/artery/application/mcm/McApplicationRetry.cc | Exact Request/Offer retry commands and timeout invalidation |
| A | src/artery/application/mcm/TrajPointMCM.h | Internal point/reference time distinction, with no wire field |
| A | src/artery/application/mcm/TrajectoryConflict.cc | Shared directional age compensation in real conflict/TTC checks |
| A | src/artery/application/mcm/TrajectoryGeneration.cc | Consistent planning reference and absolute sample times |
| A | src/artery/application/mcm/TrajectoryPlanner.cc | Propagates age into alternative-request conflict planning |
| A | src/artery/application/mcm/TrajectoryPlanner.h | Declares the corresponding planner age argument |
| B | tests/mcm/test_mcm_scenarios.py | Timing, retained proposal and multi-CV regression assertions |
| C | docs/mcm_trajectory_timing_audit.md (untracked) | Final contract, dependency evidence, tests and scope audit |
| A | src/artery/application/mcm/McApplicationFreshness.cc (untracked) | Live candidate/execution refresh and old Request invalidation |
| A | src/artery/application/mcm/McmReception.h (untracked) | Shared real receiver extraction entry point |
| A | src/artery/application/mcm/McmRetransmission.h (untracked) | Immutable complete-packet Request/Offer retention |
| A | src/artery/application/mcm/McmTiming.h (untracked) | Generation precision and timestamp validation/wrap semantics |
| A | src/artery/application/mcm/McmTrajectory.h (untracked) | Proposal comparison, generation invariant and compensation helpers |
| B | tests/mcm/mcm_timing_tests.cc (untracked) | Focused timing, runtime byte round trips, conflict/TTC and negotiation tests |
| D | extern/vanetza (dirty submodule) | Contains only the four validated descriptor corrections below |
| D | extern/vanetza/vanetza/asn1/its/MCMextra/IntentSharingContainer.c | Original optional map needed for received laneID |
| D | extern/vanetza/vanetza/asn1/its/MCMextra/ManeuverNegotiationContainer.c | Original optional map needed for runtime Request decoding |
| D | extern/vanetza/vanetza/asn1/its/MCMextra/ManeuverExecutionContainer.c | Original optional map needed for one-CV Execute decoding |
| D | extern/vanetza/vanetza/asn1/its/MCMextra/McmParameters.c | Original container map needed for runtime Execute decoding |

A = required runtime implementation; B = required test/build registration;
C = required documentation/audit; D = generated/codec correction.

The original ASN.1 schema and generated headers are unchanged. There are no
additional timing fields or wire-format redesign. Compatibility targets the
existing schema; correcting malformed codec presence handling does not imply
byte identity with the defective pre-fix codec output.

## Exact Git review state

The status listings include new files. Plain diff statistics omit untracked
files and do not expand a dirty submodule, so both working trees are shown.

### git status --short

```text
 M CMakeLists.txt
 m extern/vanetza
 M src/artery/application/CMakeLists.txt
 M src/artery/application/McService.cc
 M src/artery/application/McService.h
 M src/artery/application/McService.ned
 M src/artery/application/mcm/McApplication.cc
 M src/artery/application/mcm/McApplication.h
 M src/artery/application/mcm/McApplicationCvDecision.cc
 M src/artery/application/mcm/McApplicationLaneChange.cc
 M src/artery/application/mcm/McApplicationMerging.cc
 M src/artery/application/mcm/McApplicationNegotiationCommon.cc
 M src/artery/application/mcm/McApplicationRetry.cc
 M src/artery/application/mcm/TrajPointMCM.h
 M src/artery/application/mcm/TrajectoryConflict.cc
 M src/artery/application/mcm/TrajectoryGeneration.cc
 M src/artery/application/mcm/TrajectoryPlanner.cc
 M src/artery/application/mcm/TrajectoryPlanner.h
 M tests/mcm/test_mcm_scenarios.py
?? docs/mcm_trajectory_timing_audit.md
?? src/artery/application/mcm/McApplicationFreshness.cc
?? src/artery/application/mcm/McmReception.h
?? src/artery/application/mcm/McmRetransmission.h
?? src/artery/application/mcm/McmTiming.h
?? src/artery/application/mcm/McmTrajectory.h
?? tests/mcm/mcm_timing_tests.cc
```

### git diff --stat

```text
 CMakeLists.txt                                     |   5 +
 extern/vanetza                                     |   0
 src/artery/application/CMakeLists.txt              |   1 +
 src/artery/application/McService.cc                | 137 +++++++++-----
 src/artery/application/McService.h                 |   2 +
 src/artery/application/McService.ned               |   5 +-
 src/artery/application/mcm/McApplication.cc        |  77 +++++++-
 src/artery/application/mcm/McApplication.h         |  50 +++++-
 .../application/mcm/McApplicationCvDecision.cc     |   2 +-
 .../application/mcm/McApplicationLaneChange.cc     |   5 +-
 src/artery/application/mcm/McApplicationMerging.cc |   6 +-
 .../mcm/McApplicationNegotiationCommon.cc          |  32 +++-
 src/artery/application/mcm/McApplicationRetry.cc   |  58 +++---
 src/artery/application/mcm/TrajPointMCM.h          |   3 +-
 src/artery/application/mcm/TrajectoryConflict.cc   | 200 +++------------------
 src/artery/application/mcm/TrajectoryGeneration.cc |  16 +-
 src/artery/application/mcm/TrajectoryPlanner.cc    |  12 +-
 src/artery/application/mcm/TrajectoryPlanner.h     |   2 +-
 tests/mcm/test_mcm_scenarios.py                    |  93 ++++++++--
 19 files changed, 409 insertions(+), 297 deletions(-)
```

### git diff --check

```text
(no output; exit 0)
```

### git -C extern/vanetza status --short

```text
 M vanetza/asn1/its/MCMextra/IntentSharingContainer.c
 M vanetza/asn1/its/MCMextra/ManeuverExecutionContainer.c
 M vanetza/asn1/its/MCMextra/ManeuverNegotiationContainer.c
 M vanetza/asn1/its/MCMextra/McmParameters.c
```

### git -C extern/vanetza diff --stat

```text
 vanetza/asn1/its/MCMextra/IntentSharingContainer.c       | 3 ++-
 vanetza/asn1/its/MCMextra/ManeuverExecutionContainer.c   | 3 ++-
 vanetza/asn1/its/MCMextra/ManeuverNegotiationContainer.c | 3 ++-
 vanetza/asn1/its/MCMextra/McmParameters.c                | 3 ++-
 4 files changed, 8 insertions(+), 4 deletions(-)
```

### git -C extern/vanetza diff --check

```text
(no output; exit 0)
```

### git -C extern/vanetza diff --exit-code -- *.asn *.h

```text
(no output; exit 0)
```

