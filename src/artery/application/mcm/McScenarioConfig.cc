#include "artery/application/mcm/McScenarioConfig.h"

namespace artery
{
namespace mcm
{
namespace scenario
{

const char* const scMergingRouteId = "route_merging_1";
const char* const scHighwayMergingRouteId = "route_highway_0";
const char* const scSafetyCriticalLaneChangeRouteId = "route_highway_1";
const char* const scTargetLaneChangeRouteId = "route_highway_2";
const char* const scEmergencyVehicleId = "car_hl0_Emergency";

const double scEmergencyStartTime = 12.0;
const double scEmergencyBroadcastDuration = 15.0;
const double scEmergencyBroadcastInterval = 0.1;
const double scEmergencyMaxSpeed = 0.1;
const double scNormalHighwaySpeed = 27.77;

const double scLaneChangeShiftX = 3.0;
const double scSafetyCriticalTimeGap = 1.0;
const double scEmergencyCoordinationTimeGap = 1.5;
const double scInitialPaperTimeGap = 1.22;

const double scMergeStartX = 216554.0;
const double scMergeStartY = 452461.0;
const double scHighwayLane0MinY = 452474.0;
const double scHighwayLane0MaxY = 452574.0;
const double scMergingTimeGap = 1.2;
const double scMergeTargetMaxSnapshotAge = 0.6;

const int scRequestTrajectorySteps = 20;
const double scRequestTrajectoryDt = 0.25;
const std::size_t scMaxReceivedMcmCache = 256;

} // namespace scenario
} // namespace mcm
} // namespace artery
