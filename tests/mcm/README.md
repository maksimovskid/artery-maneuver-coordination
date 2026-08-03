# Maneuver-Coordination Simulation Regression Tests

This directory contains five deterministic, headless regression tests for the supplied maneuver-coordination validation scenarios. They run complete Artery/SUMO simulations and inspect stable diagnostic events; they are not isolated C++ unit tests.

Run the suite from the repository root:

```bash
python3 -m unittest discover -s tests/mcm -p 'test_*.py'
```

The equivalent convenience target is:

```bash
make test_mcm
```

## Covered Scenarios

Each test runs configuration run `0` with `Cmdenv`, a 30-second limit, and express mode disabled:

1. `envmod-19CAVs-merging`: coordinated merging, two-CV negotiation, execution, restoration, and completion.
2. `envmod-19CAVs-merging-baseline`: suppression of maneuver coordination.
3. `envmod-19CAVs-emergency-lane-change`: emergency Abort, follower trigger, HighPriority negotiation, lane-change execution, and completion.
4. `envmod-19CAVs-emergency-lane-change-baseline`: source braking with emergency MCM and follower coordination suppressed.
5. `envmod-19CAVs-second-request-smoke`: first-Request rejection, replacement proposal/identity, final agreement, and execution.

## Regression Evidence

The parser matches tags and stable `key=value` fields instead of comparing complete logs. Covered behavior includes:

- Request, Offer, Confirm, Accept, and Reject progression;
- initial and repeated execution-container Execute;
- cooperation ID and one/two-participant mapping;
- CV execution arming and duplicate-entry guards;
- `EmergencyPriority` execution-container Abort;
- requested and negotiated trajectory lifecycle;
- completion through the isolated negotiation-container Cancel workaround;
- RV/CV reset and failure-state evidence;
- runtime configuration defaults;
- second-Request replacement; and
- baseline suppression.

Important diagnostic families are:

- `[MCM-WIRE]`
- `[MCM-TRAJECTORY]`
- `[MCM-FAILURE]`
- `[MCM-STATE]`
- `[MCM-CONFIG]`
- `[MCM-NEGOTIATION]`
- `[MCM-NEGOTIATION-RETRY]`
- `[MCM-MERGE-TARGET]`
- `[MCM-GAP-DIAG]`
- `[MCM-EMERGENCY]`
- `[MCM-LC-TRIGGER]`
- `[MCM-LC-3VEH]`
- `[MCM-LC-EXEC]`
- `[MCM-BASELINE]`

Exact total message counts are avoided where periodic transmission or retry scheduling can vary without changing the protocol outcome. Assertions prefer sequence evidence, identity, participant IDs, container type, state transitions, and required presence/absence.

## Temporary Outputs

Each run creates a unique `/tmp/mcm-regression-*` directory. The harness routes the combined log, OMNeT++ results, SUMO trip information, and SUMO statistics into that directory.

Successful tests remove their temporary directory, leaving a previously clean checkout clean. Failed validation preserves the directory and includes the log path in the assertion error so diagnostic evidence remains available.

## Coverage Boundaries

The suite does not currently provide deterministic cases for ordinary timeout, final rejection, forced fallback, invalid NED configuration, pre-execution Cancel rollback, direct TraCI read/control failure, malformed messages, or arbitrary-map geometry. Those are documented coverage gaps, not claims about the behavior exercised by the five passing scenarios. Multi-seed statistical evaluation is also separate from this regression suite.
