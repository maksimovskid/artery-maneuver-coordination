# Adaptive CAM generation: port and verification

## Baseline audited before the port

`CaService::checkTriggeringConditions()` originally had no `triggeringCondition` parameter or adaptive flags. The baseline is immediate standard CAM generation, with `fixedRate=false`, `minInterval=.1 s`, `maxInterval=1 s`, heading delta 4°, position delta 4 m and speed delta .5 m/s. Dynamics trigger a CAM after the facilities gate, set the dynamics interval to min(elapsed,maxInterval), and reset the low-dynamics counter. After three low-dynamics generations the interval returns to maxInterval. `fixedRate=true` generates whenever the facilities gate allows, using minInterval when facilities restriction is disabled.

`genCamDcc()` selects CA's primary network interface and asks its transmit-rate throttle for the DP2 interval. The result is converted to milliseconds and clamped to [minInterval,maxInterval]. `withDccRestriction=false` bypasses that facilities query and uses minInterval. These existing semantics and clamps remain unchanged.

The CAM wire generation timestamp comes from `VehicleDataProvider::updated()` through the facilities Timer, modulo 16 bits. `mLastCamTimestamp` records application generation time, initially the vehicle/service insertion time. Low-frequency containers retain their 500 ms cadence. `sendCam()` emits CamSent, then submits the packet through BTP/GeoNetworking SHB using DP2. Application generation and actual radio transmission are distinct events.

No baseline scenario INI/traffic/trajectory settings were edited. All adaptive flags default false, and the default triggeringCondition is SameAsCAM. The original dynamics/fixedRate block and CAM content/timestamp/transport construction remain intact; the new periodic branch is bypassed in the default configuration. Disabled adaptive features do not subscribe to CBR/maneuver signals, scan MCM awareness, or consume RNG draws.

## Historical source and deliberate differences

The historical implementation is `reference/mcm_old/CaService.cc/.h/.ned`. Those files were read only. Its `trigger()` contains the adaptation; `receiveSignal()` stores CBR and maneuver mode; `checkTriggeringConditions()` selects dynamics/periodic generation. No enabled adaptive CAM INI examples were found in the supplied reference scenario configuration.

| Feature | Reference | Current before port | Current after port / interpretation |
|---|---|---|---|
| Baseline policy | Mutable string; SameAsCAM plus periodic branches | Standard dynamics/fixedRate only | Immutable configured enum plus temporary reductions; SameAsCAM default preserves current baseline |
| Own negotiation | `trigger()` lines 158–164: mode 2 → 1 Hz; exit restores SameAsCAM if shared reduction state clear | Absent | Existing current signal decoded to typed `operationMode`; own reason independent; restores configured baseline rather than overwriting it |
| Observed negotiation | Lines 167–195: any negotiation container, ego differs from ID1/ID2; CBR >=.50, high >=.65 | Absent | Same thresholds/container recognition; optional ID2 checked safely; expired entries ignored; independent hold |
| Observed hold | Moderate U(.5,1.5), high U(1,2); one draw when entering shared negotiation state | Absent | Same ranges and one draw per entry; no redraw when CBR changes band during a hold |
| Recovery | Lines 204–217: elapsed > hold AND CBR <.50; legacy also tests startTime>0 | Absent | Same strict elapsed/CBR comparisons; explicit active state fixes time-zero ambiguity |
| Intent | Lines 223–236: requires newGenCamRules; SameAsCAM and no background reduction; >.65 → U(1,5), else >.50 → U(1,2) | Absent | Same thresholds/ranges/master dependency; actually congestion-only, not a test for an MCM intention subtype |
| MCO | Lines 242–254: independent flag, >.50 → U(.1,1); recovery as above | Absent | Independent flag and exact ranges/boundaries retained; explicitly supersedes Intent when both flags enabled, as requested |
| Combined Intent/MCO | Legacy executes Intent first; shared state can prevent MCO entry when both enabled | Absent | Intentionally modernized: one selected background rule, MCO wins; never draws both holds |
| Overlap | Own/observed/Intent mutate one policy string and shared frequency state | Absent | Explicit own, observed and selected background reasons; ending one cannot cancel another |
| CBR | Host subscription to PowerLevelRx ChannelLoad, single-channel assumption | No CAM CBR subscription | Current RadioDriverBase ChannelLoad for CAM primary radio; no synthetic CBR |
| Awareness age | CaService does not check per-entry age; assumes legacy map maintenance | Current MCM map has generation-based 2 s expiry | Uses current map expiry explicitly, even before dropExpired runs; reference map source unavailable |
| Deferred normal send | SameAsCAM uses scheduleSendCam(.025) | Immediate send | Deliberately not ported; no historical-delay option or configuration requires it |
| Other policies | NED declares SameAsCAM, PeriodicFixed1Hz, PeriodicFixed2Hz; code also has unused 5Hz/.5Hz cases | No policy parameter | Supports the three declared policies; unused extra legacy branches not resurrected |
| Timestamp/statistics | Legacy timeGenCam choice and unrelated counters | VDP updated timestamp/current statistics | Preserved current timestamps/statistics; timeGenCam and obsolete measurement plumbing not ported |

The historical Intent name is ambiguous: its condition checks congestion and current policy, not MCM-intention content. The implementation and parameter documentation make that distinction explicit.

## Policy and precedence established for the port

| Source | Entry / eligibility | Combination / recovery |
|---|---|---|
| Configured baseline | SameAsCAM, PeriodicFixed1Hz or PeriodicFixed2Hz | Never overwritten; SameAsCAM retains fixedRate behavior |
| Own negotiation | newGenCamRulesMyNegotiation AND current typed negotiation mode | Independent reason; ends when ego leaves negotiation |
| Observed negotiation | newGenCamRules AND fresh eligible negotiation container AND CBR >=.50 | Independent hold; wins new background entry; recovery needs hold expired AND CBR <.50 |
| Intent background | newGenCamRules AND newGenCamRulesIntent, MCO off, SameAsCAM baseline, no own/observed/active background reason at entry, CBR >.50 | One hold; existing hold remains independent if another reason later starts |
| MCO background | newGenCamRulesIntent1Hz_MCO, SameAsCAM baseline, no own/observed/active background reason at entry, CBR >.50 | Supersedes Intent; same independent recovery semantics |

All active reasons combine with OR for effective 1 Hz. Transition-log reason bits are own=1, observed=2, Intent=4 and MCO=8. An already active background hold is not discarded when observed negotiation begins. Own/observed reasons can coexist. As in the reference, no new Intent/MCO hold starts while own negotiation already selects 1 Hz. There is no high-band escalation/redraw inside an existing hold; the entry CBR selects that hold's range. Leaving/re-entering after recovery permits one new draw.

Exact state-entry expressions in the final code (evaluated after hold recovery and with own/observed state updated):

```text
G = newGenCamRules
Y = newGenCamRulesMyNegotiation
I = newGenCamRulesIntent
M = newGenCamRulesIntent1Hz_MCO
N = ego mode is ManeuverNegotiationMode
O = a nonexpired observed negotiation exists and ego is not ID1/present ID2
B = configured baseline is SameAsCAM
H = !background.active && !observed.active && !own

Own active:      Y && N
Observed entry:  G && !observed.active && O && CBR >= .50
Intent entry:    G && I && !M && B && H && CBR > .50
MCO entry:       M && B && H && CBR > .50
```

`enabled()` is only a processing/subscription guard; its OR of configuration flags does not make Intent independent of G. Timed reasons persist after entry until their own recovery conditions hold; entry expressions are not continuous cancellation conditions.

Boundary details: observed enters at exactly .50 and uses its high band at exactly .65. Intent/MCO do not enter at exactly .50. Intent at exactly .65 uses the moderate band; its high band requires >.65. No timed reason recovers at exactly .50. Recovery requires `now > until`, preserving the legacy strict hold-elapsed comparison.

`CamGenerationPolicy` stores explicit active flags and simulation-time deadlines. Draws call `CaService::uniform(low,high)`, hence the module's OMNeT++ RNG/seed mapping, not wall-clock or C++ random facilities. Entry at simulation time zero works. Thresholds and hold ranges remain the verified historical constants, avoiding a large new configuration surface.

## Current architecture integration

### Maneuver operating mode

McService already emits `currentMCSoperatingMode` on mode changes. Its existing double encoding is Unknown=0, IntentionSharingMode=1, ManeuverNegotiationMode=2, ManeuverExecutionMode=3. The encoder, signal name and inverse decoder are now shared in `mcm/McmOperatingMode.h`; McService's emitted values/when it emits are unchanged. CaService subscribes on its own host only when adaptation is enabled and stores the decoded current typed enum. It has no dependency on obsolete McmService headers. Before the first mode update the state is Unknown, with no own reduction. Policy evaluation occurs on each existing Middleware service trigger (.1 s in the validated scenarios), so an observed mode change is reflected by the next CAM check.

### CBR source and channel

Current INET `PowerLevelRx` samples channel occupancy and emits a ratio [0,1] every 100 ms (default async phase; CBR threshold -85 dBm). InetRadioDriver forwards it as `RadioDriverBase::ChannelLoadSignal` (`ChannelLoad`), the interface also used by current DCC/McService. CaService selects the driver's host-relative radioModule whose channelNumber matches CA's `MultiChannelPolicy` primary channel. The validated configuration uses channel 180 and `World.node[*].radioDriver[0]`.

Before the first valid sample CBR is 0. Invalid/out-of-range samples are ignored. The inspected legacy constructor/initialize path did not explicitly initialize its CBR/mode fields; the port deliberately uses CBR=0 and typed Unknown at startup. For a single-radio backend lacking radioModule, the single driver is used; an unidentified multi-radio backend is not guessed and leaves CBR=0, with cbrSource=unavailable in the opt-in configuration log. This port validates the current fixed-channel INET scenario; dynamic retuning/backend-specific multi-radio discovery is outside its verification scope.

### MCM awareness and exclusions

Middleware registers current LocalDynamicMapMCM as an optional mutable facility and drops expired entries each update. McService updates it from validated received MCMs. Entries are keyed by sender station and replaced by the sender's latest packet; expiry is reconstructed generation time +2 s. `CamMcmAwareness.h` recognizes a nonexpired negotiation container, matching the legacy container-based rule, not an invented Request-only test. A recent Cancel/Reject container can therefore remain visible until replaced/expired, just as other negotiation containers.

The observed-negotiation reduction is eligible only when the ego station is **not one of the configured negotiation participants**: ego != negotiationVehicleID1 AND (negotiationVehicleID2 is absent OR ego != *negotiationVehicleID2). The rule excludes ego if it equals ID1 or a **present** ID2. It does not invent an RV/sender exclusion or interpret absent ID2 as zero. Missing map → no observed negotiation. Own-negotiation and eligible congestion-only rules remain safe independently of map availability. An absent McService supplies neither negotiations nor own-mode updates.

### Facilities versus access DCC

Effective periodic generation requires elapsed >= max(effective CAM period, current `genCamDcc()` interval). Under a reduction the CAM period is 1 s. With facilities restriction off the other term remains minInterval. The existing DCC interval clamp to configured min/max remains unchanged; this is not an unbounded raw access-layer delay.

Access DCC/LIMERIC/reactive control, DP2, SHB, EDCA and MAC queues are unchanged. Generation at 1 Hz does **not** guarantee an on-air transmission every second: lower layers can still delay/drop/control the packet. Holds are minimum recovery timers, not MAC backoffs, packet delays or substitutes for the 1 s generation interval.

## Parameters and actual INI examples

Added to CaService.ned:

- triggeringCondition = "SameAsCAM"
- newGenCamRules = false
- newGenCamRulesMyNegotiation = false
- newGenCamRulesIntent = false
- newGenCamRulesIntent1Hz_MCO = false
- logAdaptiveGeneration = false (optional cadence verification only)

The actual XML-created CAM module name is **CaService** (the service has no explicit XML name), unlike MCM's explicit **MC** name. Examples below are alternatives; no validated scenario INI was edited.

```ini
# A: unchanged default behavior
*.node[*].middleware.CaService.triggeringCondition = "SameAsCAM"
*.node[*].middleware.CaService.newGenCamRules = false
*.node[*].middleware.CaService.newGenCamRulesMyNegotiation = false
*.node[*].middleware.CaService.newGenCamRulesIntent = false
*.node[*].middleware.CaService.newGenCamRulesIntent1Hz_MCO = false
# existing defaults: fixedRate=false; withDccRestriction=true

# B: ego negotiation only (independent of general rules)
*.node[*].middleware.CaService.newGenCamRulesMyNegotiation = true

# C: observed negotiation plus congestion
*.node[*].middleware.CaService.newGenCamRules = true

# D: add congestion-only/Intent reduction; needs the master flag
*.node[*].middleware.CaService.newGenCamRules = true
*.node[*].middleware.CaService.newGenCamRulesIntent = true

# E: MCO congestion reduction, independently enabled
*.node[*].middleware.CaService.newGenCamRulesIntent1Hz_MCO = true

# F: own + observed + one selected background rule
*.node[*].middleware.CaService.newGenCamRulesMyNegotiation = true
*.node[*].middleware.CaService.newGenCamRules = true
*.node[*].middleware.CaService.newGenCamRulesIntent = true
# Optional: use MCO instead of Intent's background hold (never both holds)
*.node[*].middleware.CaService.newGenCamRulesIntent1Hz_MCO = true

# Optional diagnostics; transition logs already appear when adaptation is enabled
*.node[*].middleware.CaService.logAdaptiveGeneration = true
```

Configured periodic policies are `triggeringCondition="PeriodicFixed1Hz"` or `"PeriodicFixed2Hz"`. SameAsCAM + fixedRate=true continues to mean minInterval, optionally facilities-DCC restricted; it does not mean 1 Hz unless a reduction applies. All logging switches default off except concise opt-in configuration/reduction-transition logs. No legacy statistics were added.

## Tests and evidence

Build and focused checks:

```sh
cmake --build build --target cam_policy_tests mcm_timing_tests -j 4
build/cam_policy_tests
build/mcm_timing_tests
python3 -m unittest discover -s tests/cam -p 'test_*.py'
python3 -m unittest discover -s tests/mcm -p 'test_*.py'
```

`cam_policy_tests` passes 93 assertions covering disabled flags/no RNG, own enter/exit, time-zero holds, threshold boundaries, correct hold ranges, no redraw, reentry, both recovery conditions, overlapping reasons, MCO precedence/master independence, baseline restoration, DCC interval composition, optional ID2 including station zero, missing map, expiry boundaries and current typed mode round trips. The required DCC>1 s composition is checked directly; actual default facilities clamping remains unchanged.

The Python integration tests inherit all coordinated-merging protocol/completion assertions, enable own-negotiation adaptation explicitly with facilities DCC both ON and OFF, verify primary-channel CBR subscription, reduction and recovery transitions for every negotiating station, real CAM suppression, and the 1 s gate for any generation while reduced. It also runs fixedRate/SameAsCAM and configured 1/2 Hz cases with facilities restriction disabled. First generation may be quantized by vehicle insertion/Middleware trigger phase; subsequent intervals are checked exactly.

All five original MCM scenario regressions pass after the port: coordinated merging, merging baseline, coordinated lane change, emergency baseline, forced rejection/second Request. Existing timing, immutable Request/Offer retransmission and trajectory freshness assertions remain active. Focused MCM timing tests pass.

**Default compatibility comparison:** Before editing CaService, a 30 s merging run recorded CAM generationDeltaTime vectors. The same run/seed after the port, with all adaptive flags false, matches **all 990 CAM samples across nine stations exactly**, including event number, simulation generation time and wire generationDeltaTime. This is deterministic scenario evidence plus an unchanged baseline code path, not a claim of exhaustive testing of every possible configuration.

Evidence preserved outside the repository:

- `/tmp/cam-baseline-comparison.json` and `/tmp/compare_cam_baseline.py`
- Before: `/tmp/mcm-regression-envmod-19CAVs-merging-x2hf0bmi/results/`
- After: `/tmp/mcm-regression-envmod-19CAVs-merging-8bg6y8_o/results/`
- Adaptive coexistence: `/tmp/mcm-regression-envmod-19CAVs-merging-fdohpjo2/simulation.log`
- `/tmp/cam-focused-result.log`, `/tmp/cam-port-mcm-focused.log`
- Existing regression logs: `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-j4caxtzo/`, `merging-lr79qn5k/`, `emergency-lane-change-baseline-pcas48li/`, `merging-baseline-zta6a2eu/`, `second-request-smoke-v37d8ijq/` (each under `/tmp/mcm-regression-envmod-19CAVs-` where abbreviated).

In the adaptive run RV1 enters own reduction at 7.184426574409 s, exits at 7.384426574409 s; its preceding CAM is at 7.084426574409 s. CV169/309 enter at 7.189177300176/7.194466891862 and exit at 7.489177300176/7.494466891862. Subsecond negotiation windows suppress CAMs rather than forcing a new CAM on entry; when a reduction lasts longer, generation waits until >=1 s since the previous CAM and passes facilities DCC. All three merging negotiations still complete. CBR is naturally below congestion thresholds in this scenario, so own-negotiation integration plus deterministic CBR/state sweeps verifies the two aspects separately; no fake CBR was injected into runtime.

## Scope

Changed runtime files: CaService.cc/.h/.ned, new CamGenerationPolicy.h and CamMcmAwareness.h, new shared mcm/McmOperatingMode.h, and McService.cc's behavior-preserving use of the shared signal encoder/name. Build/test changes add cam_policy_tests and tests/cam, allow explicit extra CLI overrides in the existing scenario runner, and let the shared log parser recognize CAM tags alongside MCM tags.

No changes to reference/, scenario INIs/XML, traffic/departures, trajectories, MCM timing/priorities, search tiers, cost formulas, DCC settings or EDCA. No commit or push.

Final uninstrumented default regressions (same final runtime build) passed in 37.985 seconds:

- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-cv5xttk0/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-m9b37o2s/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-emergency-lane-change-baseline-h_z8e2jf/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-merging-baseline-ppp4fsoz/simulation.log`
- `/tmp/mcm-regression-envmod-19CAVs-second-request-smoke-yhf5jrw_/simulation.log`

Final adaptive/baseline-policy integration suite: **3 tests pass** (including the two DCC modes and three periodic-policy subcases), 10.047 s wall time:

- Facilities DCC ON: `/tmp/mcm-regression-envmod-19CAVs-merging-n1qk8zem/simulation.log`
- Facilities DCC OFF: `/tmp/mcm-regression-envmod-19CAVs-merging-vdxpuwf2/simulation.log`
- fixedRate/SameAsCAM: `/tmp/mcm-regression-envmod-19CAVs-merging-kfg5en_g/simulation.log`
- Fixed1Hz: `/tmp/mcm-regression-envmod-19CAVs-merging-n3h_1b9u/simulation.log`
- Fixed2Hz: `/tmp/mcm-regression-envmod-19CAVs-merging-9ud32vou/simulation.log`

`git diff --check` and whitespace checks for every new file pass. Reference and validated scenario files have no diff. The work remains uncommitted for review.
