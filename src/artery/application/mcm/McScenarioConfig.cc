#include "artery/application/mcm/McScenarioConfig.h"

namespace artery
{
namespace mcm
{
namespace scenario
{

const char* const scHighwayMergingRouteId = "route_highway_0";
const double scNormalHighwaySpeed = 27.77;

const double scSafetyCriticalTimeGap = 1.0;

const double scMergingTimeGap = 1.2;
const double scExecutionRestoreMinFrontDistance = 10.0;
const double scExecutionRestoreMinTtc = 2.0;

const int scRequestTrajectorySteps = 20;
const double scRequestTrajectoryDt = 0.25;
const std::size_t scMaxReceivedMcmCache = 256;

} // namespace scenario
} // namespace mcm
} // namespace artery
