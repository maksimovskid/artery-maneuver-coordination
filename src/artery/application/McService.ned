package artery.application;

simple McService like ItsG5Service
{
    parameters:
        // Test hook: when enabled and no application command is pending, submit a minimal negotiation packet with placeholder IDs/trajectories.
        bool sendNegotiationTestMcm = default(false);
        // Smoke hook: asks McApplication to force a selected CV's first high-priority lane-change response to Reject.
        bool forceFirstCvRejectForSecondRequestSmoke = default(false);
        // Exact CV station ID targeted by the smoke rejection; 0 normally matches no simulated station and disables the forced response.
        int forceFirstCvRejectStationId = default(0);

        // Baseline scenario switch: retain emergency-source braking but suppress MCM coordination behavior.
        bool emergencyBrakingOnlyBaseline = default(false);
        // SUMO vehicle ID treated as the emergency source; must be nonempty.
        string emergencySourceVehicleId = default("car_hl0_Emergency");
        // Simulation time at which source braking/emergency behavior starts (s); must be nonnegative.
        double emergencyStartTime @unit(s) = default(12s);
        // Duration of source emergency behavior (s); must be nonnegative.
        double emergencyDuration @unit(s) = default(15s);
        // Minimum interval between source emergency MCM queue attempts (s); must be positive.
        double emergencyBroadcastInterval @unit(s) = default(0.1s);
        // Source vehicle's expected pre-event speed (m/s), used by emergency scenario logic; must be nonnegative.
        double emergencyNormalSpeed @unit(mps) = default(27.77mps);
        // Emergency source target speed (m/s); must be nonnegative.
        double emergencyTargetSpeed @unit(mps) = default(0.1mps);

        // Scenario-specific route ID whose vehicle may request medium-priority merging coordination.
        string mergingRequestingRouteId = default("route_merging_1");
        // Scenario-map SUMO X trigger coordinate for the requesting vehicle (m).
        double mergingValidationTriggerX @unit(m) = default(216554m);
        // Scenario-map SUMO Y trigger coordinate for the requesting vehicle (m).
        double mergingValidationTriggerY @unit(m) = default(452461m);
        // Time-gap factor passed to merge-gap measurement at the trigger (s).
        double mergingTriggerGapTimeFactor @unit(s) = default(0.5s);
        // Required current lane index of a candidate cooperating vehicle; nonnegative.
        int mergingCooperatingLaneIndex = default(0);
        // Strict lower Y bound for candidate trajectory validation (SUMO m).
        double mergingValidationLaneMinY @unit(m) = default(452474m);
        // Strict upper Y bound for candidate trajectory validation (SUMO m).
        double mergingValidationLaneMaxY @unit(m) = default(452574m);
        // Maximum generation-derived age accepted for a candidate intent snapshot (s); must be positive.
        double mergingSnapshotFreshness @unit(s) = default(0.6s);
        // Desired temporal gap used while ranking/selecting cooperating vehicles (s).
        double mergingTargetSelectionTimeGap @unit(s) = default(1.2s);
        // Lower speed bound (m/s) used when forming the candidate gap-selection window.
        double mergingMinimumGapSpeed @unit(mps) = default(1.0mps);
        // Multiplier on the target-selection speed/time window; unitless.
        double mergingGapWindowFactor = default(2.5);

        // Medium-priority merging RV execution target speed (m/s), consumed by execution control.
        double mergingRvExecutionSpeed @unit(mps) = default(22.22mps);
        // Medium-priority merging CV acceleration target speed (m/s), consumed by CV execution control.
        double mergingCvAccelerationTargetSpeed @unit(mps) = default(33.33mps);

        // Scenario route ID for the safety-critical lane-change requesting follower.
        string laneChangeFollowerRouteId = default("route_highway_1");
        // Scenario route ID used to identify eligible target-lane cooperating vehicles.
        string laneChangeTargetCvRouteId = default("route_highway_2");
        // Planned lateral path displacement (m) used to generate the RV Request candidate.
        double laneChangePlannedLateralShift @unit(m) = default(3.0m);
        // Scenario-map SUMO Y threshold for correcting the lane-change lane-index interpretation (m).
        double laneChangeValidationLaneCorrectionThresholdY @unit(m) = default(452365m);
        // Number of execution path samples/steps; positive integer, distinct from service update cadence.
        int laneChangeExecutionStepCount = default(10);
        // Lateral displacement per execution step (m) in the existing lane-change execution controller.
        double laneChangeExecutionLateralShiftPerStep @unit(m) = default(0.32m);
        // Desired minimum inter-vehicle gap used in candidate/coordination assessment (s).
        double laneChangeDesiredMinimumTimeGap @unit(s) = default(1.0s);
        // Time-gap threshold used by lane-change trajectory conflict checks (s).
        double laneChangeCoordinationConflictTimeGap @unit(s) = default(1.5s);
        // Initial gap used by the scenario's paper/validation safety check (s).
        double laneChangePaperInitialTimeGap @unit(s) = default(1.22s);
        // Minimum front distance gate during lane-change execution (m).
        double laneChangeExecutionMinFrontDistance @unit(m) = default(5.0m);
        // Minimum execution time gap (s).
        double laneChangeExecutionMinTimeGap @unit(s) = default(0.3s);
        // Minimum execution time-to-collision threshold (s).
        double laneChangeExecutionMinTtc @unit(s) = default(1.0s);
        // Safe fallback speed (m/s) if the coordinated lane-change execution cannot continue.
        double laneChangeFallbackSpeed @unit(mps) = default(0.1mps);
        // Duration over which fallback deceleration is applied (s); must be positive.
        double laneChangeFallbackDecelerationTime @unit(s) = default(1.0s);

        // Minimum front distance required before restoring normal speed (m).
        double executionRestoreMinFrontDistance @unit(m) = default(10.0m);
        // Minimum leader-aware gap required before restoring normal speed (s).
        double executionRestoreMinTimeGap @unit(s) = default(1.0s);
        // Minimum TTC required before restoring normal speed (s).
        double executionRestoreMinTtc @unit(s) = default(2.0s);

        // Intent policy: SameAsCAM, SameAsCAMreduced2Hz, SameAsCAMreduced1Hz,
        // PeriodicFixed, PeriodicFixed0.5Hz, PeriodicFixed1Hz/2Hz/3Hz/5Hz.
        // Unknown values fall back to SameAsCAM. Frequency policies use intervals below.
        string triggeringCondition = default("SameAsCAM");
        // Coordination policy: SameAsCAM, PeriodicFixed, PeriodicFixedNoDCC.
        // The fixed modes use maxIntervalNegMcm and coordination takes precedence over intent.
        string triggeringConditionCoordination = default("SameAsCAM");
        // Enables McService facilities generation interval admission from the access-layer DCC throttle.
        // Does not disable or bypass DCC at the ITS access/MAC layers.
        bool withDccRestriction = default(false);
        // Minimum facilities interval (s), also the lower clamp for queried DCC wait and dynamics pacing.
        double minInterval @unit(s) = default(0.1s);
        // Maximum facilities interval (s), upper clamp for queried DCC wait and low-dynamics backoff.
        double maxInterval @unit(s) = default(1.0s);
        // SameAsCAM policy selector: true uses fixedRateInterval; false uses dynamics thresholds/backoff.
        bool fixedRate = default(true);
        // Fixed interval (s) for SameAsCAM fixed-rate intent/coordination; other fixed policy names use their own intervals.
        double fixedRateInterval @unit(s) = default(0.1s);
        // Compatibility parameter: parsed/logged but not used to decide message timing.
        double fixedInterval @unit(s) = default(0.1s);
        // Fixed coordination interval (s) for PeriodicFixed and PeriodicFixedNoDCC; nonpositive falls back to minInterval.
        double maxIntervalNegMcm @unit(s) = default(0.1s);

        // McApplication retry spacing between queued negotiation attempts (s), not the overall negotiation timeout.
        double negotiationRetryInterval @unit(s) = default(0.1s);
        // Overall timeout (s) for medium-priority merging negotiations.
        double negotiationLimitMerging @unit(s) = default(1.0s);
        // Overall timeout (s) for high-priority lane-change negotiations.
        double negotiationLimitLaneChange @unit(s) = default(1.0s);

        // Enables differentiated Vanetza profile selection; false selects DP2 for every MCM. With dccOnlyMcm=false: negotiation high DP0/other priorities DP1; execution emergency DP0/high DP1/other DP2; intent DP2.
        bool dccProfiles = default(false);
        // When differentiated mapping is enabled: negotiation high/medium/low map DP0/DP1/DP2; execution emergency/high/medium/low map DP0/DP1/DP2/DP3; other modes DP3. Also selects adaptive thresholds/recovery-delay ranges.
        bool dccOnlyMcm = default(false);

        // Compatibility string parsed as DataProvider or CurrentTime (unknown values fall back to DataProvider).
        // It is logged but does not select the live prediction/header origin: generationDeltaTime is authoritative.
        string timeGenMcm = default("DataProvider");

        // General adaptive rule master flag. Requires newGenMcmRulesIntent, starts only from SameAsCAM, and is overridden by the independent MCO 1 Hz rule.
        bool newGenMcmRules = default(false);
        // Enables general CBR-based intent-rate reduction when the master flag is true; a configured fixed/other policy is not rewritten.
        bool newGenMcmRulesIntent = default(false);
        // Independent MCO intent rule; takes precedence over the general rule and reduces SameAsCAM to 1 Hz above its threshold.
        bool newGenMcmRulesIntent1Hz_MCO = default(false);
        // General rule entry/recovery threshold when dccOnlyMcm=false; strict >/< comparisons and 1–2 s randomized recovery. Unitless CBR ratio; no Important Intent exemption exists yet.
        double freqReduceCBRmin = default(0.5);
        // Reserved parsed/logged threshold; no current decision reads it (unitless CBR ratio).
        double freqReduceCBRmedium = default(0.6);
        // General rule entry/recovery threshold when dccOnlyMcm=true; strict >/< comparisons and 0.5–1.5 s randomized recovery. Unitless CBR ratio.
        double freqReduceCBRmax = default(0.65);
        // Entry/recovery threshold for the independent MCO intent rule; strict >/< comparisons and 1–2 s recovery (0.1–1 s with dccOnlyMcm). Unitless CBR ratio; no Important Intent exemption exists yet.
        double freqReduceCBRmco = default(0.5);

        // Enables using route CSV geometry to form a fresh intent prediction. When false the outgoing intent trajectory is a zero placeholder; negotiation candidates still come from McApplication.
        bool usePrerecordedIntentTrajectory = default(false);
        // CSV file path containing route-specific <routeId>_x/<routeId>_y columns, or others_x/others_y fallback columns.
        string prerecordedTrajectoryCsv = default("");
        // Number of points requested from the route planner, clamped to 1..20; independent of service/MCM cadence.
        int prerecordedTrajectorySteps = default(20);
        // Spacing between trajectory samples (s), not the service update interval or packet generation interval.
        double prerecordedTrajectoryDt @unit(s) = default(0.25s);
        // Heading-change threshold for dynamics intent triggering (degrees).
        double headingDelta = default(4.0);
        // Position-change threshold for dynamics intent triggering (m).
        double positionDelta @unit(m) = default(4.0m);
        // Speed-change threshold for dynamics intent triggering (m/s).
        double speedDelta @unit(mps) = default(0.5mps);

        // Semantic packet objects emitted at service-stack handoff/indication, not radio success notifications.
        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);
        // Service handling counters; sent means handed down by McService, received means decoded/accepted by this service.
        @signal[McmReceivedCounter](type=long);
        @signal[McmSentCounter](type=long);
        @signal[McmIntentionReceivedCounter](type=long);
        @signal[McmIntentionSentCounter](type=long);
        @signal[McmNegotiationReceivedCounter](type=long);
        @signal[McmNegotiationSentCounter](type=long);
        @signal[McmExecutionReceivedCounter](type=long);
        @signal[McmExecutionSentCounter](type=long);
        @signal[McmExecutionEmergencyReceivedCounter](type=long);
        @signal[McmExecutionEmergencySentCounter](type=long);
        // Encoded receive size in bytes; generation-to-reception delay/AoI in simulation seconds.
        @signal[msgsizeReceived](type=unsigned long);
        @signal[EteDelayMcm](type=simtime_t);
        @signal[EteDelayMcmNegotiation](type=simtime_t);
        @signal[EteDelayMcmExecution](type=simtime_t);
        @signal[EteDelayMcmEmergency](type=simtime_t);
        @signal[CoopVehicleAgeOfInformation](type=simtime_t);
        // DCC-derived facilities wait interval in seconds and latest channel-busy ratio in [0,1].
        @signal[dccTimeWaitNextMcm](type=simtime_t);
        @signal[coopCBR](type=double);
        // NegotiationStarted is first Request submission; completed is McApplication's final-Accept/Execute completion evidence. negotiationTime spans those RV-side events and excludes maneuver execution. The two/three-vehicle buckets use numberOfVehicles <=2/>2. Execution counters mark first non-emergency execution-container submission and Cancel submission while executing; no execution-duration signal is emitted.
        @signal[NegotiationStartedCounter](type=long);
        @signal[NegotiationCompletedCounter](type=long);
        @signal[negotiationTime](type=simtime_t);
        @signal[ExecutionStartedCounter](type=long);
        @signal[ExecutionCompletedCounter](type=long);
        @signal[CounterNegotiationRejected](type=long);
        @signal[CounterNegotiationTwoVehicles](type=long);
        @signal[CounterNegotiationThreeVehicles](type=long);
        // Planner outputs: costs are planner-defined scalar values with no declared physical unit; the rest count candidate/trajectory classes/events.
        @signal[TrajectoryCost](type=double);
        @signal[TrajectoryCostRV](type=double);
        @signal[SecondRequestStartedCounter](type=long);
        @signal[SecondRequestCompletedCounter](type=long);
        @signal[SecondRequestRejectedCounter](type=long);
        @signal[CounterCoordPossiblePriorityLow](type=long);
        @signal[CounterCoordPossiblePriorityMedium](type=long);
        @signal[CounterCoordPossiblePriorityHigh](type=long);
        @signal[CounterTrajectoryType0](type=long);
        @signal[CounterTrajectoryType1](type=long);
        @signal[CounterTrajectoryType2](type=long);
        @signal[CounterTrajectoryType4](type=long);
        @signal[CounterTrajectoryType5](type=long);
        @signal[CounterTrajectoryType6](type=long);
        // Numeric application mode (1 intent, 2 negotiation, 3 execution), emitted only on change.
        @signal[currentMCSoperatingMode](type=double);

        // Count/vector recorders on Mcm*Counter values; service submit counters do not mean radio transmission success.
        @statistic[reception](source=McmReceived;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[McmReceivedCounter](source=McmReceivedCounter;record=count,vector?);
        @statistic[McmSentCounter](source=McmSentCounter;record=count,vector?);
        @statistic[McmIntentionReceivedCounter](source=McmIntentionReceivedCounter;record=count,vector?);
        @statistic[McmIntentionSentCounter](source=McmIntentionSentCounter;record=count,vector?);
        @statistic[McmNegotiationReceivedCounter](source=McmNegotiationReceivedCounter;record=count,vector?);
        @statistic[McmNegotiationSentCounter](source=McmNegotiationSentCounter;record=count,vector?);
        @statistic[McmExecutionReceivedCounter](source=McmExecutionReceivedCounter;record=count,vector?);
        @statistic[McmExecutionSentCounter](source=McmExecutionSentCounter;record=count,vector?);
        @statistic[McmExecutionEmergencyReceivedCounter](source=McmExecutionEmergencyReceivedCounter;record=count,vector?);
        @statistic[McmExecutionEmergencySentCounter](source=McmExecutionEmergencySentCounter;record=count,vector?);
        // stats/vector recorders below capture byte size, delays, CBR and event duration/cost values.
        @statistic[msgsizeReceived](source=msgsizeReceived;record=stats,vector);
        @statistic[EteDelayMcm](source=EteDelayMcm;record=stats,vector);
        @statistic[EteDelayMcmNegotiation](source=EteDelayMcmNegotiation;record=stats,vector);
        @statistic[EteDelayMcmExecution](source=EteDelayMcmExecution;record=stats,vector);
        @statistic[EteDelayMcmEmergency](source=EteDelayMcmEmergency;record=stats,vector);
        @statistic[CoopVehicleAgeOfInformation](source=CoopVehicleAgeOfInformation;record=stats,vector);
        @statistic[dccTimeWaitNextMcm](source=dccTimeWaitNextMcm;record=stats,vector);
        @statistic[coopCBR](source=coopCBR;record=stats,vector);
        @statistic[NegotiationStartedCounter](source=NegotiationStartedCounter;record=count,vector?);
        @statistic[NegotiationCompletedCounter](source=NegotiationCompletedCounter;record=count,vector?);
        @statistic[negotiationTime](source=negotiationTime;record=stats,vector);
        @statistic[ExecutionStartedCounter](source=ExecutionStartedCounter;record=count,vector?);
        @statistic[ExecutionCompletedCounter](source=ExecutionCompletedCounter;record=count,vector?);
        @statistic[CounterNegotiationRejected](source=CounterNegotiationRejected;record=count,vector?);
        @statistic[CounterNegotiationTwoVehicles](source=CounterNegotiationTwoVehicles;record=count,vector?);
        @statistic[CounterNegotiationThreeVehicles](source=CounterNegotiationThreeVehicles;record=count,vector?);
        @statistic[TrajectoryCost](source=TrajectoryCost;record=stats,vector);
        @statistic[TrajectoryCostRV](source=TrajectoryCostRV;record=stats,vector);
        @statistic[SecondRequestStartedCounter](source=SecondRequestStartedCounter;record=count,vector?);
        @statistic[SecondRequestCompletedCounter](source=SecondRequestCompletedCounter;record=count,vector?);
        @statistic[SecondRequestRejectedCounter](source=SecondRequestRejectedCounter;record=count,vector?);
        @statistic[CounterCoordPossiblePriorityLow](source=CounterCoordPossiblePriorityLow;record=count,vector?);
        @statistic[CounterCoordPossiblePriorityMedium](source=CounterCoordPossiblePriorityMedium;record=count,vector?);
        @statistic[CounterCoordPossiblePriorityHigh](source=CounterCoordPossiblePriorityHigh;record=count,vector?);
        @statistic[CounterTrajectoryType0](source=CounterTrajectoryType0;record=count,vector?);
        @statistic[CounterTrajectoryType1](source=CounterTrajectoryType1;record=count,vector?);
        @statistic[CounterTrajectoryType2](source=CounterTrajectoryType2;record=count,vector?);
        @statistic[CounterTrajectoryType4](source=CounterTrajectoryType4;record=count,vector?);
        @statistic[CounterTrajectoryType5](source=CounterTrajectoryType5;record=count,vector?);
        @statistic[CounterTrajectoryType6](source=CounterTrajectoryType6;record=count,vector?);
        @statistic[currentMCSoperatingMode](source=currentMCSoperatingMode;record=stats,vector);
}
