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
extern const char* const scMergingRouteId;
extern const char* const scHighwayMergingRouteId;
extern const char* const scSafetyCriticalLaneChangeRouteId;
extern const char* const scTargetLaneChangeRouteId;

// The emergency vehicle is intentionally singled out only in the emergency
// lane-change validation scenario. The MCM protocol itself does not depend on
// this vehicle name.
extern const char* const scEmergencyVehicleId;

// Timing and speed values are scenario parameters, not ASN.1/protocol constants.
extern const double scEmergencyStartTime;
extern const double scEmergencyBroadcastDuration;
extern const double scEmergencyBroadcastInterval;
extern const double scEmergencyMaxSpeed;
extern const double scNormalHighwaySpeed;

// Safety-critical lane-change validation knobs. The lane shift is tied to the
// current SUMO lane geometry; the time gaps preserve the old implementation's
// high-priority/emergency thresholds.
extern const double scLaneChangeShiftX;
extern const double scSafetyCriticalTimeGap;
extern const double scEmergencyCoordinationTimeGap;
extern const double scInitialPaperTimeGap;

// Medium-priority merging validation geometry. The trigger point and highway
// lane-0 coordinate window come from the current SUMO map/route setup.
extern const double scMergeStartX;
extern const double scMergeStartY;
extern const double scHighwayLane0MinY;
extern const double scHighwayLane0MaxY;
extern const double scMergingTimeGap;
extern const double scMergeTargetMaxSnapshotAge;

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
