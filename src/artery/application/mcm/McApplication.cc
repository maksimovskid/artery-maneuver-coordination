#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/traci/VehicleController.h"

#include <algorithm>

namespace artery
{
namespace mcm
{

namespace
{
constexpr const char* scMergingRouteId = "route_merging_1";
constexpr double scMergeStartX = 216554.0;
constexpr double scMergeStartY = 452461.0;
constexpr double scHighwayLane0MinY = 452474.0;
constexpr double scHighwayLane0MaxY = 452574.0;
constexpr double scMergingTimeGap = 1.2;
constexpr int scRequestTrajectorySteps = 20;
constexpr double scRequestTrajectoryDt = 0.25;
constexpr std::size_t scMaxReceivedMcmCache = 256;
}

void McApplication::initialize(
    const traci::VehicleController* controller,
    const VehicleDataProvider* vehicleDataProvider)
{
    mVehicleController = controller;
    mVehicleDataProvider = vehicleDataProvider;
    mTrajectoryPlanner.initialize(controller, vehicleDataProvider, nullptr);
}

void McApplication::updateEgoContext(const McEgoContext& context)
{
    mEgoContext = context;
    mHasEgoContext = true;
}

void McApplication::tick(omnetpp::SimTime now)
{
    evaluateMergingRequestTrigger(now);
}

void McApplication::handleReceivedMcm(const ReceivedMcm& mcm)
{
    ++mReceivedMcmCount;
    mLastReceivedMcm = mcm;
    mHasLastReceivedMcm = true;
    mReceivedMcmCache.push_back(mcm);
    if (mReceivedMcmCache.size() > scMaxReceivedMcmCache) {
        mReceivedMcmCache.erase(mReceivedMcmCache.begin());
    }
}

void McApplication::handleSentMcm(const SentMcm& mcm)
{
    ++mSentMcmCount;
    mLastSentMcm = mcm;
    mHasLastSentMcm = true;

    if (mcm.data.hasNegotiationContainer &&
            mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Request) &&
            mCoordinationProgressRV == coordinationProgressRV::CoordinationRequired) {
        mCoordinationProgressRV = coordinationProgressRV::RequestSent;
        mMergingRequestQueuedOrSent = true;
    }
}

std::optional<PendingMcmCommand> McApplication::consumePendingCommand()
{
    auto command = mPendingMcmCommand;
    mPendingMcmCommand.reset();
    return command;
}

void McApplication::clearCommand()
{
    mPendingCommand = CommandKind::None;
    mExecutionState = ExecutionState::Idle;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mRestoreSpeed = 0.0;
    mRestoreMaxSpeed = 0.0;
    mPendingMcmCommand.reset();
}

bool McApplication::hasActiveExecution() const
{
    return mExecutionState == ExecutionState::Pending || mExecutionState == ExecutionState::Executing;
}

void McApplication::applyCommand()
{
    // TODO: later use VehicleController hooks for DecelerateTo and RestoreNormalSpeed commands.
}

void McApplication::evaluateMergingRequestTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || !mVehicleController ||
            mPendingMcmCommand || mMergingRequestQueuedOrSent) {
        return;
    }

    if (mEgoContext.routeId != scMergingRouteId) {
        EV_DETAIL << "McApplication merge trigger ignored for route "
            << mEgoContext.routeId << '\n';
        return;
    }

    const double distanceToStartingPoint =
        getDistance(mEgoContext.x, mEgoContext.y, scMergeStartX, scMergeStartY);
    const double desiredDistanceGap = mTrajectoryPlanner.getGap(0.5);

    EV_DETAIL << "McApplication route_merging_1 trigger check at x=" << mEgoContext.x
        << " y=" << mEgoContext.y << " distanceToStart=" << distanceToStartingPoint
        << " desiredGap=" << desiredDistanceGap << '\n';

    if (mCoordinationProgressRV == coordinationProgressRV::NoCoordination &&
            distanceToStartingPoint <= desiredDistanceGap) {
        mCoordinationProgressRV = coordinationProgressRV::CheckForCoordination;
        EV_INFO << "McApplication route_merging_1 old trigger condition reached for station "
            << mEgoContext.stationId << '\n';
    }

    if (mCoordinationProgressRV != coordinationProgressRV::CheckForCoordination) {
        return;
    }

    if (mEgoContext.plannedTrajectory.empty()) {
        EV_WARN << "McApplication route_merging_1 trigger has no ego plannedTrajectory; cannot request coordination\n";
        return;
    }

    unsigned considered = 0;
    uint32_t target1 = 0;
    uint32_t target2 = 0;
    unsigned conflicts = 0;

    for (const auto& received : mReceivedMcmCache) {
        const auto& snapshot = received.data;
        if (snapshot.stationId == mEgoContext.stationId || !snapshot.hasLaneId ||
                snapshot.laneId != 0 || snapshot.plannedTrajectory.empty()) {
            continue;
        }

        const auto& first = snapshot.plannedTrajectory.front();
        if (first.mX == 1.0 && first.mY == 1.0) {
            continue;
        }

        ++considered;

        if (first.mY <= scHighwayLane0MinY || first.mY >= scHighwayLane0MaxY) {
            continue;
        }

        const omnetpp::SimTime eteDelay = std::max(omnetpp::SimTime::ZERO, now - received.receivedAt);
        const bool conflict = mTrajectoryPlanner.check_traj_conflict_merging(
            mEgoContext.plannedTrajectory,
            snapshot.plannedTrajectory,
            scMergingTimeGap,
            eteDelay,
            true);

        if (!conflict) {
            continue;
        }

        EV_INFO << "McApplication route_merging_1 conflict found with station "
            << snapshot.stationId << '\n';

        if (target1 == 0 || target1 == snapshot.stationId) {
            target1 = snapshot.stationId;
            ++conflicts;
        } else if (target2 == 0 && target1 != snapshot.stationId) {
            target2 = snapshot.stationId;
            ++conflicts;
        }
    }

    EV_INFO << "McApplication route_merging_1 considered " << considered
        << " received planned trajectories; conflicts=" << conflicts << '\n';

    if (target1 == 0) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Request;
    command.priority = priorityMcmCategory::MediumPriority;
    command.cooperationType = 0;
    command.requestId = makeRequestId(now);
    command.numberOfVehicles = target2 == 0 ? 1 : 2;
    command.targetVehicle1 = target1;
    command.hasTargetVehicle2 = target2 != 0;
    command.targetVehicle2 = target2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mPendingMcmCommand = command;
    mMergingRequestQueuedOrSent = true;
    mCooperatingVehicleType = cooperatingVehicleType::RV;
    mMcmSubtype = mcmSubtype::Request;
    mPriorityMcmCategory = priorityMcmCategory::MediumPriority;
    mOperationMode = operationMode::ManeuverNegotiationMode;
    mCoordinationProgressRV = coordinationProgressRV::CoordinationRequired;

    EV_INFO << "McApplication queued route_merging_1 Request: requestId="
        << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size() << '\n';
}

uint8_t McApplication::makeRequestId(omnetpp::SimTime now) const
{
    const auto millis = static_cast<uint64_t>(now.inUnit(omnetpp::SIMTIME_MS));
    return static_cast<uint8_t>((mEgoContext.stationId + millis + mReceivedMcmCount) % 256);
}

}  // namespace mcm
}  // namespace artery
