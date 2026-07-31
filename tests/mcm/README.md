# Maneuver-Coordination Regression Tests

This directory contains deterministic headless regression tests for the validated
maneuver-coordination scenarios.

Run the suite from the repository root:

```bash
python3 -m unittest discover -s tests/mcm -p "test_*.py"
```

The equivalent convenience target is:

```bash
make test_mcm
```

## Covered Scenarios

The tests run these OMNeT++ configurations with `Cmdenv`, run `0`,
`--sim-time-limit=30s`, and `--cmdenv-express-mode=false`:

* `envmod-19CAVs-merging`
* `envmod-19CAVs-merging-baseline`
* `envmod-19CAVs-emergency-lane-change`
* `envmod-19CAVs-emergency-lane-change-baseline`
* `envmod-19CAVs-second-request-smoke`

## Assertions

The suite parses stable MCM log tags and key-value fields instead of comparing
complete logs. It checks protocol progression, baseline suppression,
emergency-braking behavior, second-request construction, and completion evidence.

Important tags include:

* `[MCM-NEGOTIATION]`
* `[MCM-MERGE-TARGET]`
* `[MCM-GAP-DIAG]`
* `[MCM-EMERGENCY]`
* `[MCM-LC-TRIGGER]`
* `[MCM-LC-3VEH]`
* `[MCM-LC-EXEC]`
* `[MCM-BASELINE]`
* `[MCM-NEGOTIATION-RETRY]`

Exact total message counts are generally avoided because repeated MCM generation
and retry timing can change without invalidating the protocol flow. Tests prefer
presence, absence, participant IDs, request IDs, and completion markers.

## Temporary Outputs

Each scenario run writes its combined stdout/stderr log and any OMNeT++ result
files into a temporary `/tmp/mcm-regression-*` directory. Successful tests remove
their temporary directory. If a run fails validation, the directory is preserved
and the assertion error reports the log path.

## Limitations

These are smoke/regression tests for the supplied 19-CAV scenarios. They do not
replace unit tests for malformed messages, arbitrary maps, empty trajectories,
vehicle disappearance, or multi-seed statistical evaluation.
