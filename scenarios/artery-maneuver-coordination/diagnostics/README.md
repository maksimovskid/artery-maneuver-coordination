# 200-CAV QoS Diagnostic

This diagnostic measures the first part of the high-load QoS chain for the
current 200-CAV maneuver-coordination scenario:

```text
active CAV population over time
-> peak simultaneous active CAV count
-> local Channel Busy Ratio
-> MCM activity
-> maneuver-negotiation outcomes
```

It is a diagnostic workflow, not a multi-seed high-load evaluation. It does not
measure CAM/MCM PDR, actual MAC transmission intervals, complete AoI, or 500-CAV
behavior.

## Scenario Configuration

The quick OMNeT++ configuration is:

```text
envmod-200CAVs-qos-diagnostic
```

It extends `envmod-200CAVs-qos-baseline-freespace`, keeps the same 200-CAV SUMO
traffic demand, runs for `10 s`, and writes generated outputs under:

```text
scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/
```

The longer normal-window diagnostic is:

```text
envmod-200CAVs-qos-diagnostic-30s
```

It extends the same 200-CAV QoS baseline configuration, runs for `30 s`, and
writes separate generated outputs under:

```text
scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos_30s/
```

The 10-second profile is intended as a quick active-population and CBR check.
The 30-second profile covers the normal maneuver-coordination window and can
capture negotiation and execution results. Neither profile is a statistical
evaluation; both describe run `0` with seed `10`.

Generated diagnostic outputs are ignored by Git.

## Active Population

Active population is derived from SUMO FCD output by counting `<vehicle>`
elements at each FCD timestep. In this scenario, all SUMO vehicles are mapped to
equipped CAV nodes by the active Artery mapper, so the count is treated as the
active equipped CAV population. Infrastructure nodes are not included.

## CBR Aggregation

`ChannelLoad` remains a local per-station measurement. The analysis aggregates
recorded station samples per simulation timestamp and reports mean, median,
95th-percentile, maximum CBR, and the number of contributing samples. These
aggregates are diagnostic summaries of local observations, not a single global
physical CBR.

## Communication and Channel Assumptions

The diagnostic configurations inherit from
`envmod-200CAVs-qos-baseline-freespace`. The active network is
`artery.envmod.World`, which inherits the INET radio medium
`Ieee80211ScalarRadioMedium`. The diagnostic override sets the path-loss type to
`FreeSpacePathLoss`. The scenario still enables the physical environment and
obstacle loading, but this path-loss model does not use the GEMV2 obstacle,
shadowing, or fading logic. The result is a broad, unobstructed sensing and
interference model compared with obstacle-aware or fading models.

Vehicles use `artery.envmod.Car` with two `VanetNic` radios configured on
channel `180`, carrier frequency `5.9 GHz`, bandwidth `10 MHz`, bitrate
`6 Mbps`, and transmitter power `200 mW`. The receivers are `VanetReceiver`
instances using INET scalar SNIR reception logic with a `6 dB` capture
threshold; other receiver thresholds are inherited from the INET/Artery radio
defaults unless explicitly configured elsewhere. `ChannelLoad` is emitted by
the receiver path every `100 ms` from an 8 microsecond busy/free sampler.

All mapped SUMO vehicles use `services-mco-envmod.xml`, which assigns both
`CaService` and `McService`. This means all active equipped CAVs are expected to
generate CAM traffic and Intent MCM traffic. The focal maneuver vehicles can
also generate negotiation, execution, and emergency MCM containers when the
scenario logic reaches those states. Background vehicles mainly act as
communication-load CAVs; their fallback prerecorded trajectories populate
Intent MCMs, but they are not intended to be focal maneuver participants.

`CaService` uses ETSI-style CAM triggering with `minInterval = 0.1 s`,
`maxInterval = 1.0 s`, heading-change threshold `4 deg`, position-change
threshold `4 m`, speed-change threshold `0.5 m/s`, and `withDccRestriction =
true` by default. CAM generation, DCC eligibility, actual MAC transmission, and
reception are separate concepts. This diagnostic does not record a direct CAM
sent counter or actual CAM MAC transmission timestamps, so CAM rates are not
reported.

`McService` uses `triggeringCondition = "SameAsCAM"`, `fixedRate = true`,
`fixedRateInterval = 0.1 s`, `minInterval = 0.1 s`, and `maxInterval = 1.0 s`.
For these diagnostic configurations, `newGenMcmRules`,
`newGenMcmRulesIntent`, and `newGenMcmRulesIntent1Hz_MCO` are disabled. The
nominal Intent MCM generation/submission interval is therefore `0.1 s`
(`10 Hz`) once a vehicle is active and the service has started. Application-level
MCM DCC waiting is disabled by `McService.withDccRestriction = false`, so
`dccTimeWaitNextMcm` is expected to be unavailable. Vanetza DCC is still present
as `LimericDccEntity` with dual alpha enabled.

CAM and MCM traffic use the default multi-channel policy (`CCH`, channel `180`
in this setup) and therefore share channel capacity. CAMs use BTP port `2001`,
ITS-AID `CA`, and DCC profile `DP2`. MCMs use BTP port `2027`, experimental
ITS-AID `650`, and `DP2` because `McService.dccProfiles = false`. The observed
CBR therefore reflects the combined local channel occupancy of CAM traffic,
Intent MCM traffic, maneuver negotiation/execution MCMs, and any other wireless
transmissions on the same configured channel. It should not be attributed only
to MCM traffic.

## Interpreting the Message Counts

The analyzer derives active-CAV vehicle-seconds by integrating the FCD active
vehicle count over simulation time. A per-active-CAV-second MCM rate is then:

```text
service-level MCM counter / active-CAV vehicle-seconds
```

For the 30-second run, the active population integral is `4757.2`
active-CAV-seconds. The run records `45401` service-level MCM sent events and
`45060` service-level Intent MCM sent events, giving approximately `9.54` MCMs
and `9.47` Intent MCMs per active-CAV-second. That is consistent with the
configured nominal `10 Hz` Intent MCM generation/submission path, with startup
and insertion timing included in the measured active-vehicle integral.

The MCM counters are emitted by `McService` at the application/service layer
before handing the packet to lower layers. They are useful for MCM activity and
load-generation diagnostics, but they are not packet-delivery counters and do
not prove actual MAC transmission timing. DCC-delayed, queued, or suppressed
packets must not be counted as radio packet errors without lower-layer
transmission and reception evidence.

## Running

```bash
make diagnose_200cav_qos
make diagnose_200cav_qos_10s
make diagnose_200cav_qos_30s
```

`diagnose_200cav_qos` is an alias for the faster 10-second run. Both concrete
targets run headlessly with run `0`, seed `10`, `Cmdenv`, and express mode
disabled. When `/usr/bin/time` is available, wall-clock runtime and peak
resident memory are captured in `runtime.txt`.

Runtime grows significantly as the active population increases. The 30-second
target is expected to take substantially longer than the 10-second target. The
500-CAV scenario is not executed by these targets.

The analysis can also be run directly after a diagnostic simulation:

```bash
python3 tools/analyze_high_load_diagnostic.py \
  --runtime-log scenarios/artery-maneuver-coordination/results_diagnostic/200cav_qos/runtime.txt
```

## Output Files

The analyzer writes:

```text
diagnostic_summary.csv
active_population_timeseries.csv
cbr_timeseries.csv
mcm_qos_summary.csv
coordination_period_summary.csv
```

The 30-second target also writes:

```text
scenarios/artery-maneuver-coordination/results_diagnostic/diagnostic_10s_vs_30s.csv
```

Unavailable metrics are written as `not_available` rather than inferred.
