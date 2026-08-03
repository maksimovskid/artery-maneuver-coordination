#include "artery/application/mcm/McApplication.h"

#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>

#include <cmath>
#include <exception>
#include <string>

/*
 * Implements merging-gap diagnostics for McApplication.
 *
 * This file samples and reports diagnostic gap measurements around merging
 * execution. The diagnostics observe McApplication state and vehicle positions;
 * they do not own separate maneuver state or change the protocol flow.
 *
 * The source split is organizational only. All definitions are member functions
 * of the single McApplication class declared in McApplication.h and operate on
 * the same shared application state.
 */

namespace artery
{
namespace mcm
{

namespace
{
}

void McApplication::resetMergingGapDiagnostics()
{
    mMergingGapDiagActive = false;
    mMergingGapDiagExecutionStart = omnetpp::SimTime::ZERO;
    mMergingGapDiagTargetCvStationId = 0;
    mMergingGapDiagMinDistance = 0.0;
    mMergingGapDiagMinTimeGap = 0.0;
    mMergingGapDiagMinDistanceAt = omnetpp::SimTime::ZERO;
    mMergingGapDiagMinTimeGapAt = omnetpp::SimTime::ZERO;
    mMergingGapDiagHasMinDistance = false;
    mMergingGapDiagHasMinTimeGap = false;
    mMergingGapDiagMinRvLaneId.clear();
    mMergingGapDiagMinCvLaneId.clear();
    mMergingGapDiagMinRvLaneIndex = -1;
    mMergingGapDiagMinCvLaneIndex = -1;
    mMergingGapDiagMinRvLanePosition = 0.0;
    mMergingGapDiagMinCvX = 0.0;
    mMergingGapDiagMinCvY = 0.0;
    mMergingGapDiagMinRvX = 0.0;
    mMergingGapDiagMinRvY = 0.0;
}

/*
 * Samples the current RV-to-target-CV gap during merging execution. The
 * diagnostics use the latest received CV trajectory sample and TraCI lane data
 * for reporting only; the sampled values do not drive maneuver decisions.
 */
void McApplication::sampleMergingGapDiagnostics(const char* phase)
{
    EV_STATICCONTEXT;

    if (!mMergingGapDiagActive || !mHasEgoContext || !mVehicleController ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            mEgoContext.routeId != mMergingCoordinationConfig.requestingRouteId) {
        return;
    }

    const std::string rvVehicleId = mVehicleController->getVehicleId();
    std::string rvLaneId = "unavailable";
    int rvLaneIndex = mEgoContext.laneIndex;
    double rvLanePosition = 0.0;
    bool hasRvLanePosition = false;

    try {
        auto traci = mVehicleController->getTraCI();
        if (traci) {
            rvLaneId = traci->vehicle.getLaneID(rvVehicleId);
            rvLaneIndex = traci->vehicle.getLaneIndex(rvVehicleId);
            rvLanePosition = traci->vehicle.getLanePosition(rvVehicleId);
            hasRvLanePosition = true;
        }
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-GAP-DIAG]"
            << " simTime=" << mEgoContext.now
            << " side=RV"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << rvVehicleId
            << " route=" << mEgoContext.routeId
            << " phase=" << phase
            << " rvTraCIQuery=failed"
            << " reason=\"" << e.what() << "\"\n";
    }

    const uint32_t targets[] = { mRvTargetVehicle1, mRvTargetVehicle2 };
    for (uint32_t targetStationId : targets) {
        if (targetStationId == 0) {
            continue;
        }

        const ReceivedMcm* latestTargetMcm = nullptr;
        for (auto it = mReceivedMcmCache.rbegin(); it != mReceivedMcmCache.rend(); ++it) {
            if (it->data.stationId == targetStationId) {
                latestTargetMcm = &*it;
                break;
            }
        }

        if (!latestTargetMcm || latestTargetMcm->data.plannedTrajectory.empty()) {
            EV_INFO << "[MCM-GAP-DIAG]"
                << " simTime=" << mEgoContext.now
                << " side=RV"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << rvVehicleId
                << " route=" << mEgoContext.routeId
                << " phase=" << phase
                << " targetCvStation=" << targetStationId
                << " targetCvSample=unavailable"
                << " reason=no-latest-target-mcm-plannedTrajectory\n";
            continue;
        }

        const auto& snapshot = latestTargetMcm->data;
        const auto& cvPoint = snapshot.plannedTrajectory.front();
        const double dx = cvPoint.mX - mEgoContext.x;
        const double dy = cvPoint.mY - mEgoContext.y;
        const double euclideanGap = std::sqrt(dx * dx + dy * dy);
        const double signedYGap = cvPoint.mY - mEgoContext.y;
        const double cvSpeed = snapshot.speedValue >= 0 && snapshot.speedValue < 16383 ?
            static_cast<double>(snapshot.speedValue) / 100.0 : -1.0;
        const double rvSpeedTimeGap = mEgoContext.speed > 0.1 ?
            euclideanGap / mEgoContext.speed : -1.0;
        const double closingSpeed = cvSpeed >= 0.0 ? mEgoContext.speed - cvSpeed : 0.0;
        const double closingTimeGap = closingSpeed > 0.1 ? euclideanGap / closingSpeed : -1.0;
        const int cvLaneIndex = snapshot.hasLaneId ? static_cast<int>(snapshot.laneId) : -1;
        const std::string cvLaneId = snapshot.hasLaneId ? std::to_string(snapshot.laneId) : "unavailable";

        if (!mMergingGapDiagHasMinDistance || euclideanGap < mMergingGapDiagMinDistance) {
            mMergingGapDiagHasMinDistance = true;
            mMergingGapDiagMinDistance = euclideanGap;
            mMergingGapDiagMinDistanceAt = mEgoContext.now;
            mMergingGapDiagTargetCvStationId = targetStationId;
            mMergingGapDiagMinRvLaneId = rvLaneId;
            mMergingGapDiagMinCvLaneId = cvLaneId;
            mMergingGapDiagMinRvLaneIndex = rvLaneIndex;
            mMergingGapDiagMinCvLaneIndex = cvLaneIndex;
            mMergingGapDiagMinRvLanePosition = rvLanePosition;
            mMergingGapDiagMinRvX = mEgoContext.x;
            mMergingGapDiagMinRvY = mEgoContext.y;
            mMergingGapDiagMinCvX = cvPoint.mX;
            mMergingGapDiagMinCvY = cvPoint.mY;
        }

        if (rvSpeedTimeGap >= 0.0 &&
                (!mMergingGapDiagHasMinTimeGap || rvSpeedTimeGap < mMergingGapDiagMinTimeGap)) {
            mMergingGapDiagHasMinTimeGap = true;
            mMergingGapDiagMinTimeGap = rvSpeedTimeGap;
            mMergingGapDiagMinTimeGapAt = mEgoContext.now;
        }

        EV_INFO << "[MCM-GAP-DIAG]"
            << " simTime=" << mEgoContext.now
            << " side=RV"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << rvVehicleId
            << " route=" << mEgoContext.routeId
            << " phase=" << phase
            << " targetCvStation=" << targetStationId
            << " targetCvSample=latest-mcm-plannedTrajectory-front"
            << " rvLaneId=" << rvLaneId
            << " rvLaneIndex=" << rvLaneIndex
            << " rvLanePosition=" << (hasRvLanePosition ? rvLanePosition : -1.0)
            << " cvLaneId=" << cvLaneId
            << " cvLaneIndex=" << cvLaneIndex
            << " cvLanePosition=unavailable"
            << " rvX=" << mEgoContext.x
            << " rvY=" << mEgoContext.y
            << " cvApproxX=" << cvPoint.mX
            << " cvApproxY=" << cvPoint.mY
            << " rvSpeed=" << mEgoContext.speed
            << " cvSpeed=" << cvSpeed
            << " approxEuclideanGap=" << euclideanGap
            << " approxSignedYGap=" << signedYGap
            << " approxTimeGapByRvSpeed=" << rvSpeedTimeGap
            << " approxTimeGapByClosingSpeed=" << closingTimeGap
            << " currentMinApproxEuclideanGap=" << mMergingGapDiagMinDistance
            << " currentMinApproxTimeGapByRvSpeed="
            << (mMergingGapDiagHasMinTimeGap ? mMergingGapDiagMinTimeGap : -1.0)
            << '\n';
    }
}

/*
 * Reports the best observed merging gap at execution completion. Missing
 * samples are reported explicitly so validation logs distinguish no-data cases
 * from a measured small gap.
 */
void McApplication::logMergingGapSummary(omnetpp::SimTime completionTime) const
{
    EV_STATICCONTEXT;

    if (!mMergingGapDiagHasMinDistance) {
        EV_INFO << "[MCM-GAP-DIAG]"
            << " summary=rv-completion"
            << " rvStation=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " route=" << mMergingCoordinationConfig.requestingRouteId
            << " executionStart=" << mMergingGapDiagExecutionStart
            << " completionTime=" << completionTime
            << " targetCvStation=" << mMergingGapDiagTargetCvStationId
            << " result=no-gap-samples\n";
        return;
    }

    EV_INFO << "[MCM-GAP-DIAG]"
        << " summary=rv-completion"
        << " rvStation=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " route=" << mMergingCoordinationConfig.requestingRouteId
        << " executionStart=" << mMergingGapDiagExecutionStart
        << " completionTime=" << completionTime
        << " targetCvStationAtMin=" << mMergingGapDiagTargetCvStationId
        << " minApproxEuclideanGap=" << mMergingGapDiagMinDistance
        << " minGapAt=" << mMergingGapDiagMinDistanceAt
        << " minApproxTimeGapByRvSpeed="
        << (mMergingGapDiagHasMinTimeGap ? mMergingGapDiagMinTimeGap : -1.0)
        << " minTimeGapAt="
        << (mMergingGapDiagHasMinTimeGap ? mMergingGapDiagMinTimeGapAt : omnetpp::SimTime::ZERO)
        << " rvLaneIdAtMin=" << mMergingGapDiagMinRvLaneId
        << " rvLaneIndexAtMin=" << mMergingGapDiagMinRvLaneIndex
        << " rvLanePositionAtMin=" << mMergingGapDiagMinRvLanePosition
        << " cvLaneIdAtMin=" << mMergingGapDiagMinCvLaneId
        << " cvLaneIndexAtMin=" << mMergingGapDiagMinCvLaneIndex
        << " rvXAtMin=" << mMergingGapDiagMinRvX
        << " rvYAtMin=" << mMergingGapDiagMinRvY
        << " cvApproxXAtMin=" << mMergingGapDiagMinCvX
        << " cvApproxYAtMin=" << mMergingGapDiagMinCvY
        << " cvPositionSourceAtMin=latest-mcm-plannedTrajectory-front\n";
}

}  // namespace mcm

}  // namespace artery
