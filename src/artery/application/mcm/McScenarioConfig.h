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
extern const char* const scSafetyCriticalLaneChangeRouteId;
extern const char* const scTargetLaneChangeRouteId;

// Physical speed-control calibration shared by the current validation scenarios.
extern const double scNormalHighwaySpeed;

// Safety-critical lane-change validation knobs. The lane shift is tied to the
// current SUMO lane geometry; the time gaps preserve the calibrated scenario
// high-priority/emergency thresholds.
extern const double scLaneChangeShiftX;
extern const double scSafetyCriticalTimeGap;
extern const double scEmergencyCoordinationTimeGap;
extern const double scInitialPaperTimeGap;

// Current validation-map lane correction and physical lane-change execution
// calibration. These are scenario workarounds, not general protocol rules.
extern const double scValidationMapLaneIndexCorrectionThresholdY;
extern const std::size_t scLaneChangeExecutionStepCount;
extern const double scLaneChangeExecutionLateralShiftPerStep;
extern const double scLaneChangeExecutionMinFrontDistance;
extern const double scLaneChangeExecutionMinTimeGap;
extern const double scLaneChangeExecutionMinTtc;
extern const double scLaneChangeEmergencyFallbackSpeed;
extern const double scLaneChangeEmergencyFallbackDecelerationTime;

// Cooperation time-gap calibration shared by merging and lane-change planner paths.
extern const double scMergingTimeGap;

// Generic execution-restoration safety calibration.
extern const double scExecutionRestoreMinFrontDistance;
extern const double scExecutionRestoreMinTtc;

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
