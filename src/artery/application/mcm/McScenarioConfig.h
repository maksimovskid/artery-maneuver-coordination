#ifndef ARTERY_MCM_MCSCENARIOCONFIG_H_
#define ARTERY_MCM_MCSCENARIOCONFIG_H_

#include <cstddef>

namespace artery
{
namespace mcm
{
namespace scenario
{

// Validation route IDs from the SUMO route files. These are scenario selectors,
// not protocol concepts; they should eventually become omnetpp.ini parameters.
extern const char* const scHighwayMergingRouteId;

// Physical speed-control calibration shared by the current validation scenarios.
extern const double scNormalHighwaySpeed;

// Cooperation time-gap calibration shared by merging and lane-change planner paths.
extern const double scMergingTimeGap;

// Request trajectory generation and local cache limits used by the validation
// implementation. These are implementation/scenario controls and should move to
// configuration once the behavior is no longer tied to the paper scenario.
extern const int scRequestTrajectorySteps;
extern const double scRequestTrajectoryDt;
extern const std::size_t scMaxReceivedMcmCache;

} // namespace scenario
} // namespace mcm
} // namespace artery

#endif
