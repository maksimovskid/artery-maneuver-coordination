package artery.application;

simple McService like ItsG5Service
{
    parameters:
        @signal[McmReceived](type=McObject);
        @signal[McmSent](type=McObject);

        @statistic[reception](source=McmReceived;record=vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
        @statistic[transmission](source=McmSent;record=vector(mcmStationId)?,vector(mcmGenerationDeltaTime)?);
}
