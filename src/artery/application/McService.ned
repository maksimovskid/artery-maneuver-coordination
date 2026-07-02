package artery.application;

simple McService like ItsG5Service
{
    parameters:
        bool sendNegotiationTestMcm = default(false);
        bool withDccRestriction = default(false);
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);
        bool fixedRate = default(true);
        double fixedRateInterval @unit(s) = default(0.1s);
        double negotiationRetryInterval @unit(s) = default(0.1s);
        double negotiationLimitMerging @unit(s) = default(1.0s);
        double negotiationLimitLaneChange @unit(s) = default(1.0s);
        bool usePrerecordedIntentTrajectory = default(false);
        string prerecordedTrajectoryCsv = default("");
        int prerecordedTrajectorySteps = default(20);
        double prerecordedTrajectoryDt @unit(s) = default(0.25s);
        double headingDelta = default(4.0);
        double positionDelta @unit(m) = default(4.0m);
        double speedDelta @unit(mps) = default(0.5mps);

        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);

        @statistic[reception](source=McmReceived;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
}
