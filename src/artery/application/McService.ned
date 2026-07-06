package artery.application;

simple McService like ItsG5Service
{
    parameters:
        // Disabled-by-default legacy test hook for emitting negotiation containers.
        bool sendNegotiationTestMcm = default(false);

        // MCM generation-rule hooks restored from the old research service.
        // The current default path still preserves the existing fixed-rate
        // behavior; the named policies will be implemented incrementally.
        string triggeringCondition = default("SameAsCAM"); // intent sharing: SameAsCAM, SameAsCAMreduced2Hz, SameAsCAMreduced1Hz, PeriodicFixed*
        string triggeringConditionCoordination = default("SameAsCAM"); // negotiation/execution: SameAsCAM, PeriodicFixed, PeriodicFixedNoDCC

        // DCC restriction and interval limits. withDccRestriction currently
        // gates generation through genMcmDcc(); other DCC-related parameters
        // are configuration hooks for staged congested-scenario work.
        bool withDccRestriction = default(false);
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);
        bool fixedRate = default(true);
        double fixedRateInterval @unit(s) = default(0.1s);
        double fixedInterval @unit(s) = default(0.1s); // compatibility alias for old NED configs; not active by default
        double maxIntervalNegMcm @unit(s) = default(0.1s); // coordination/negotiation max interval hook

        // Negotiation retry limits consumed by McApplication.
        double negotiationRetryInterval @unit(s) = default(0.1s);
        double negotiationLimitMerging @unit(s) = default(1.0s);
        double negotiationLimitLaneChange @unit(s) = default(1.0s);

        // DCC profile mapping hooks. dccProfiles=false preserves the current
        // DP2 traffic class. dccOnlyMcm affects the old profile mapping once
        // profile selection is activated.
        bool dccProfiles = default(false);
        bool dccOnlyMcm = default(false);

        // Generation timestamp source hook. DataProvider preserves current
        // behavior; CurrentTime exists for old-configuration compatibility.
        string timeGenMcm = default("DataProvider");

        // Adaptive generation-rule hooks from the old congested-scenario code.
        // They are parsed and logged but behavior is intentionally staged.
        bool newGenMcmRules = default(false);
        bool newGenMcmRulesIntent = default(false);
        bool newGenMcmRulesIntent1Hz_MCO = default(false);
        double freqReduceCBRmin = default(0.5); // future adaptive-frequency threshold hook; inactive by default
        double freqReduceCBRmedium = default(0.6); // future adaptive-frequency threshold hook; inactive by default
        double freqReduceCBRmax = default(0.65); // future adaptive-frequency threshold hook; inactive by default
        double freqReduceCBRmco = default(0.5); // future MCO 1 Hz intent threshold hook; inactive by default

        bool usePrerecordedIntentTrajectory = default(false);
        string prerecordedTrajectoryCsv = default("");
        int prerecordedTrajectorySteps = default(20);
        double prerecordedTrajectoryDt @unit(s) = default(0.25s);
        double headingDelta = default(4.0);
        double positionDelta @unit(m) = default(4.0m);
        double speedDelta @unit(mps) = default(0.5mps);

        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);
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
        @signal[msgsizeReceived](type=unsigned long);
        @signal[EteDelayMcm](type=simtime_t);
        @signal[EteDelayMcmNegotiation](type=simtime_t);
        @signal[EteDelayMcmExecution](type=simtime_t);
        @signal[EteDelayMcmEmergency](type=simtime_t);
        @signal[dccTimeWaitNextMcm](type=simtime_t);
        @signal[coopCBR](type=double);
        @signal[NegotiationStartedCounter](type=long);
        @signal[NegotiationCompletedCounter](type=long);
        @signal[ExecutionStartedCounter](type=long);
        @signal[ExecutionCompletedCounter](type=long);
        @signal[CounterNegotiationRejected](type=long);
        @signal[CounterNegotiationTwoVehicles](type=long);
        @signal[CounterNegotiationThreeVehicles](type=long);
        @signal[currentMCSoperatingMode](type=double);

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
        @statistic[msgsizeReceived](source=msgsizeReceived;record=stats,vector);
        @statistic[EteDelayMcm](source=EteDelayMcm;record=stats,vector);
        @statistic[EteDelayMcmNegotiation](source=EteDelayMcmNegotiation;record=stats,vector);
        @statistic[EteDelayMcmExecution](source=EteDelayMcmExecution;record=stats,vector);
        @statistic[EteDelayMcmEmergency](source=EteDelayMcmEmergency;record=stats,vector);
        @statistic[dccTimeWaitNextMcm](source=dccTimeWaitNextMcm;record=stats,vector);
        @statistic[coopCBR](source=coopCBR;record=stats,vector);
        @statistic[NegotiationStartedCounter](source=NegotiationStartedCounter;record=count,vector?);
        @statistic[NegotiationCompletedCounter](source=NegotiationCompletedCounter;record=count,vector?);
        @statistic[ExecutionStartedCounter](source=ExecutionStartedCounter;record=count,vector?);
        @statistic[ExecutionCompletedCounter](source=ExecutionCompletedCounter;record=count,vector?);
        @statistic[CounterNegotiationRejected](source=CounterNegotiationRejected;record=count,vector?);
        @statistic[CounterNegotiationTwoVehicles](source=CounterNegotiationTwoVehicles;record=count,vector?);
        @statistic[CounterNegotiationThreeVehicles](source=CounterNegotiationThreeVehicles;record=count,vector?);
        @statistic[currentMCSoperatingMode](source=currentMCSoperatingMode;record=stats,vector);
}
