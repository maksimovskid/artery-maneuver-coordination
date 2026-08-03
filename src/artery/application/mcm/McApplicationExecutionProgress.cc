#include "artery/application/mcm/McApplication.h"

#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/traci/VehicleController.h"

#include <cmath>

/*
 * Implements execution-progress signaling for McApplication.
 *
 * This file queues repeated Execute messages and detects RV/CV completion of
 * the active negotiated trajectory. It intentionally remains separate from
 * low-level SUMO control while sharing the same McApplication state.
 *
 * The source split is organizational only; it does not create a separate
 * runtime component.
 */

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scMergingRouteId;

} // namespace

// Successful completion has no dedicated ASN.1 McmCategory yet. These are the
// single intended role-specific construction points for the backward-compatible
// Cancel encoding; a future migration should replace it with execution-container
// Complete.
PendingMcmCommand McApplication::buildRvExecutionCompletionWorkaroundCommand() const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Cancel;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;
    return command;
}

PendingMcmCommand McApplication::buildCvExecutionCompletionWorkaroundCommand() const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Cancel;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mCvRequestId;
    command.numberOfVehicles = 1;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;
    return command;
}

/*
 * Queues an additional Execute while the RV remains in execution mode. The
 * repeated message carries the live planned trajectory from the current ego
 * context rather than resending a frozen copy of the original request.
 */
void McApplication::queueRepeatedExecute()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            !mHasRvNegotiatedTrajectory) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Execution;
    command.subtype = mcmSubtype::Execute;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;

    // During execution, the negotiated trajectory is no longer sent as a separate
    // fixed trajectory. The live plannedTrajectory/intent is updated and sent.
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mPendingMcmCommand = command;
    EV_INFO << "[MCM-WIRE]"
        << " direction=queued"
        << " station=" << mEgoContext.stationId
        << " subtype=Execute"
        << " kind=Execution"
        << " container=Execution"
        << " origin=repeated-execute"
        << " requestId=-1"
        << " cooperationId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " hasTarget2=" << command.hasTargetVehicle2
        << " priority=" << priorityName(static_cast<long>(command.priority))
        << '\n';
    EV_INFO << "[MCM-TRAJECTORY]"
        << " event=repeated-execute-retains-negotiated-trajectory"
        << " station=" << mEgoContext.stationId
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " trajectoryRole=negotiated"
        << " trajectoryPoints=" << mRvNegotiatedTrajectory.size()
        << " liveTrajectoryPoints=" << mEgoContext.plannedTrajectory.size()
        << '\n';
    mLastExecuteQueuedAt = mEgoContext.now;
    mHasLastExecuteQueuedAt = true;

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " queued repeated Execute"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " plannedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';

    // std::cout << "MCM_DEBUG RV station " << mEgoContext.stationId
    //     << " queued repeated Execute for requestId "
    //     << static_cast<int>(command.requestId)
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

/*
 * Detects RV completion of the active negotiated trajectory and queues the
 * current Complete-as-Cancel workaround. The diagnostic sample is taken before
 * RV coordination state is reset by the sent-message path.
 */
void McApplication::evaluateRvExecutionProgress()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            !mHasRvNegotiatedTrajectory) {
        return;
    }

    if (!hasReachedActiveNegotiatedTrajectoryEnd()) {
        return;
    }

    PendingMcmCommand command = buildRvExecutionCompletionWorkaroundCommand();

    mPendingMcmCommand = command;
    EV_INFO << "[MCM-WIRE]"
        << " direction=queued"
        << " station=" << mEgoContext.stationId
        << " subtype=Cancel"
        << " kind=Negotiation"
        << " container=Negotiation"
        << " origin=execution-completion-workaround"
        << " role=RV"
        << " executionState=SendExecute"
        << " requestId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " hasTarget2=" << command.hasTargetVehicle2
        << " priority=" << priorityName(static_cast<long>(command.priority))
        << '\n';
    mMcmSubtype = mcmSubtype::Cancel;
    mCoordinationProgressRV = coordinationProgressRV::SendComplete;
    sampleMergingGapDiagnostics("rv-completion-queued");

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " queued completion workaround after passing negotiated trajectory end"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " currentY=" << mEgoContext.y
        << " finalY=" << mRvNegotiatedTrajectory.back().mY
        << '\n';

    // std::cout << "MCM_DEBUG RV station " << mEgoContext.stationId
    //     << " queued Complete workaround using Cancel for requestId "
    //     << static_cast<int>(command.requestId)
    //     << " currentY=" << mEgoContext.y
    //     << " finalY=" << mRvNegotiatedTrajectory.back().mY
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

/*
 * Detects CV completion of the selected negotiated trajectory and queues the
 * same Complete-as-Cancel workaround used by the RV side, preserving the CV
 * request and RV target identifiers.
 */
void McApplication::evaluateCvExecutionProgress()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressCV != coordinationProgressCV::SendExecuteCV ||
            !mHasCvNegotiatedTrajectory) {
        return;
    }

    if (!hasReachedActiveNegotiatedTrajectoryEnd()) {
        return;
    }

    PendingMcmCommand command = buildCvExecutionCompletionWorkaroundCommand();

    mPendingMcmCommand = command;
    EV_INFO << "[MCM-WIRE]"
        << " direction=queued"
        << " station=" << mEgoContext.stationId
        << " subtype=Cancel"
        << " kind=Negotiation"
        << " container=Negotiation"
        << " origin=execution-completion-workaround"
        << " role=CV"
        << " executionState=SendExecuteCV"
        << " requestId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " hasTarget2=" << command.hasTargetVehicle2
        << " priority=" << priorityName(static_cast<long>(command.priority))
        << '\n';
    mMcmSubtype = mcmSubtype::Cancel;
    mCoordinationProgressCV = coordinationProgressCV::SendCompleteCV;

    EV_INFO << "[MCM-CV-CONTROL]"
        << " simTime=" << mEgoContext.now
        << " event=execution-complete"
        << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " cvStation=" << mEgoContext.stationId
        << " rvStation=" << command.targetVehicle1
        << " requestId=" << static_cast<int>(command.requestId)
        << " priority=" << priorityName(static_cast<long>(mPriorityMcmCategory))
        << " selectedAction=" << controlManeuverName(mControlManeuver)
        << " currentX=" << mEgoContext.x
        << " currentY=" << mEgoContext.y
        << " finalX=" << mCvNegotiatedTrajectory.back().mX
        << " finalY=" << mCvNegotiatedTrajectory.back().mY
        << '\n';

    EV_INFO << "McApplication CV station " << mEgoContext.stationId
        << " queued completion workaround after passing negotiated trajectory end"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " rvStation=" << command.targetVehicle1
        << " currentY=" << mEgoContext.y
        << " finalY=" << mCvNegotiatedTrajectory.back().mY
        << '\n';

    // std::cout << "MCM_DEBUG CV station " << mEgoContext.stationId
    //     << " queued Complete workaround using Cancel for requestId "
    //     << static_cast<int>(command.requestId)
    //     << " currentY=" << mEgoContext.y
    //     << " finalY=" << mCvNegotiatedTrajectory.back().mY
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

/*
 * Checks whether the ego vehicle has passed the saved negotiated trajectory
 * endpoint. The comparison is route-aware and still uses global SUMO Cartesian
 * trajectory points, not an ego-relative coordinate transform.
 */
bool McApplication::hasReachedActiveNegotiatedTrajectoryEnd() const
{
    const TrajectoryPlanner::Trajectory* negotiatedTrajectory = nullptr;
    if (mCooperatingVehicleType == cooperatingVehicleType::RV && mHasRvNegotiatedTrajectory) {
        negotiatedTrajectory = &mRvNegotiatedTrajectory;
    } else if (mCooperatingVehicleType == cooperatingVehicleType::CV && mHasCvNegotiatedTrajectory) {
        negotiatedTrajectory = &mCvNegotiatedTrajectory;
    }

    if (!mHasEgoContext || !negotiatedTrajectory || negotiatedTrajectory->empty()) {
        return false;
    }

    const auto& firstPoint = negotiatedTrajectory->front();
    const auto& lastPoint = negotiatedTrajectory->back();

    if (mEgoContext.routeId == scMergingRouteId) {
        // The route_merging_1 RV behavior:
        // if RV passed the last negotiated trajectory point, then send Complete.
        // In this scenario, passing the point means current SUMO y is below the
        // final negotiated trajectory y.
        return mEgoContext.y <= lastPoint.mY;
    }

    // Generic trajectory-end check for non-merging-route vehicles, e.g. CVs on
    // the main lane. Use the dominant trajectory direction to decide whether the
    // vehicle has passed the final saved negotiated point.
    const double dx = lastPoint.mX - firstPoint.mX;
    const double dy = lastPoint.mY - firstPoint.mY;

    if (std::abs(dx) >= std::abs(dy)) {
        return dx >= 0.0 ? mEgoContext.x >= lastPoint.mX : mEgoContext.x <= lastPoint.mX;
    }

    return dy >= 0.0 ? mEgoContext.y >= lastPoint.mY : mEgoContext.y <= lastPoint.mY;
}

}  // namespace mcm
}  // namespace artery
