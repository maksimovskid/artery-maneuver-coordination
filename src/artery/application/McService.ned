package artery.application;

simple McService like ItsG5Service
{
    parameters:
        bool sendNegotiationTestMcm = default(false);

        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);

        @statistic[reception](source=McmReceived;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=count,vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
}
