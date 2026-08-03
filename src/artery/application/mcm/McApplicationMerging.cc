#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

/*
 * Implements merging-specific coordination for McApplication.
 *
 * This file classifies CV behavior for merging Requests, selects merge-gap
 * targets, and creates RV merging Requests from the pre-recorded global SUMO
 * reference paths. It preserves the current one-CV and two-CV coordination
 * paths and operates on the shared McApplication state.
 *
 * The source split is organizational only; it does not create a separate
 * merging coordinator object.
 */

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scHighwayLane0MaxY;
using scenario::scHighwayLane0MinY;
using scenario::scHighwayMergingRouteId;
using scenario::scMergeStartX;
using scenario::scMergeStartY;
using scenario::scMergeTargetMaxSnapshotAge;
using scenario::scMergingRouteId;
using scenario::scMergingTimeGap;
using scenario::scRequestTrajectoryDt;
using scenario::scRequestTrajectorySteps;
using scenario::scSafetyCriticalLaneChangeRouteId;
using scenario::scTargetLaneChangeRouteId;

bool isHighwayMergingCvRoute(const std::string& routeId)
{
    return routeId == scHighwayMergingRouteId ||
        routeId == scSafetyCriticalLaneChangeRouteId ||
        routeId == scTargetLaneChangeRouteId;
}

struct MergeTargetCandidate {
    uint32_t stationId = 0;
    double signedLongitudinalGap = 0.0;
    double absLongitudinalGap = 0.0;
    double euclideanDistance = 0.0;
    double rvY = 0.0;
    double cvY = 0.0;
    omnetpp::SimTime age = omnetpp::SimTime::ZERO;
    bool diagnosticPointConflict = false;
};

struct MergeTargetSelection {
    uint32_t target1 = 0;
    uint32_t target2 = 0;
};

// RV-side medium-priority merging helper. The RV selects the closest
// lead/follow pair around its predicted merge point from current idle MCM
// snapshots. This keeps the validation dynamic; the known vehicle pairs emerge
// from SUMO timing and trajectories, not from fixed station IDs.
MergeTargetSelection selectMergeGapTargets(std::vector<MergeTargetCandidate> candidates)
{
    auto closerToGap = [](const MergeTargetCandidate& lhs, const MergeTargetCandidate& rhs) {
        if (lhs.absLongitudinalGap != rhs.absLongitudinalGap) {
            return lhs.absLongitudinalGap < rhs.absLongitudinalGap;
        }
        return lhs.stationId < rhs.stationId;
    };

    std::vector<MergeTargetCandidate> behind;
    std::vector<MergeTargetCandidate> ahead;
    for (const auto& candidate : candidates) {
        if (candidate.signedLongitudinalGap < 0.0) {
            behind.push_back(candidate);
        } else {
            ahead.push_back(candidate);
        }
    }
    std::sort(behind.begin(), behind.end(), closerToGap);
    std::sort(ahead.begin(), ahead.end(), closerToGap);
    std::sort(candidates.begin(), candidates.end(), closerToGap);

    MergeTargetSelection selection;
    if (!behind.empty() && !ahead.empty()) {
        selection.target1 = behind.front().stationId;
        selection.target2 = ahead.front().stationId;
    } else if (!candidates.empty()) {
        selection.target1 = candidates.front().stationId;
        if (candidates.size() > 1) {
            selection.target2 = candidates[1].stationId;
        }
    }

    return selection;
}

}

/*
 * Chooses the CV-side control maneuver for a medium-priority merging Request.
 * The method keeps the current global-SUMO trajectory comparison and planner
 * cost evaluation, then stores the selected trajectory and control command for
 * later Offer/Accept construction and execution control.
 */
void McApplication::classifyCvMergingControlManeuver(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    mControlManeuver = controlManeuver::DoNothing;
    mCvSelectedTrajectory.clear();
    mHasCvSelectedTrajectory = false;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mCvDecelerationControlApplied = false;
    mCvDecelerationControlSkippedLogged = false;
    mCvAccelerationControlApplied = false;
    mCvLaneChangeControlLogged = false;

    if (!mHasEgoContext || !mVehicleDataProvider) {
        EV_WARN << "McApplication CV maneuver classification skipped: missing ego context or vehicle data\n";
        return;
    }

    if (!isHighwayMergingCvRoute(mEgoContext.routeId)) {
        EV_DETAIL << "McApplication CV maneuver classification skipped for route "
            << mEgoContext.routeId << " because this milestone only handles highway CVs for route_merging_1\n";
        return;
    }

    const auto& snapshot = received.data;
    if (snapshot.requestedTrajectory.empty()) {
        EV_WARN << "McApplication CV maneuver classification kept DoNothing: Request has no requestedTrajectory\n";
        return;
    }

    if (mEgoContext.routeReferenceX.empty() || mEgoContext.routeReferenceY.empty() ||
            mEgoContext.routeReferenceIndex < 0) {
        EV_WARN << "McApplication CV maneuver classification kept DoNothing: route reference coordinates/index are unavailable\n";
        return;
    }

    if (mEgoContext.plannedTrajectory.empty()) {
        EV_WARN << "McApplication CV maneuver classification kept DoNothing: ego plannedTrajectory is unavailable\n";
        return;
    }

    const omnetpp::SimTime eteDelay =
        std::max(omnetpp::SimTime::ZERO, mEgoContext.now - received.receivedAt);
    const bool conflict = mTrajectoryPlanner.check_traj_conflict_merging(
        mEgoContext.plannedTrajectory,
        snapshot.requestedTrajectory,
        scMergingTimeGap,
        eteDelay,
        false);

    if (!conflict) {
        EV_INFO << "McApplication CV station " << mEgoContext.stationId
            << " classified merging control maneuver: DoNothing"
            << " because requestedTrajectory has no conflict with current plannedTrajectory\n";
        return;
    }

    const auto& myLastPoint = mEgoContext.plannedTrajectory.back();
    const auto& requestedLastPoint = snapshot.requestedTrajectory.back();
    const bool decelerationRequired = myLastPoint.mY >= requestedLastPoint.mY;
    const bool accelerationRequired = !decelerationRequired;
    // The safety-critical lane-change CV branch keeps the CV lane-change
    // feasibility block disabled, so target-lane CVs cooperate by speed
    // adaptation or DoNothing rather than initiating another lane change.
    const bool laneChangePossible = false;
    const bool routeAffected = false;
    const int priority = snapshot.priorityManeuver >= 0 && snapshot.priorityManeuver <= 2 ?
        static_cast<int>(snapshot.priorityManeuver) : 1;

    auto result = mTrajectoryPlanner.findSuitableTrajectoryCV(
        snapshot.requestedTrajectory,
        priority,
        true,
        scRequestTrajectorySteps,
        scRequestTrajectoryDt,
        mEgoContext.routeReferenceX,
        mEgoContext.routeReferenceY,
        mEgoContext.routeReferenceIndex,
        mEgoContext.speed,
        eteDelay,
        decelerationRequired,
        accelerationRequired,
        laneChangePossible,
        routeAffected);

    const bool foundSuitableTrajectory = result.found;
    if (!foundSuitableTrajectory) {
        EV_INFO << "McApplication CV station " << mEgoContext.stationId
            << " classified merging control maneuver: DoNothing"
            << " because findSuitableTrajectoryCV found no safe trajectory"
            << " adaptationBranch=" << (decelerationRequired ? "decelerate-or-change-lane" : "accelerate-or-change-lane")
            << " requestId=" << snapshot.requestId
            << " priority=" << priority
            << '\n';
        return;
    }

    const auto& selectedTrajectory = result.trajectory;
    const PlannedTrajValues& plannedValues = result.plannedValues;
    const double trajectoryCost = result.cooperationCost;
    const int trajectoryType = result.trajectoryType;
    const int possiblePriorityLevel = result.possiblePriorityLevel;
    recordCvPlannerEvaluation(trajectoryCost, trajectoryType, possiblePriorityLevel);

    if (plannedValues.lane_change) {
        mControlManeuver = controlManeuver::ChangeLane;
    } else if (plannedValues.deceleration_change > 0.0) {
        mControlManeuver = controlManeuver::Decelerate;
        mTargetSpeed = mEgoContext.speed - plannedValues.speed_change * mEgoContext.speed;
        const double decelerationOffset = decelerationRequired ? 0.05 : 0.2;
        mCommandDuration = calculateDecelerationTime(
            mEgoContext.speed,
            mTargetSpeed,
            plannedValues.deceleration_change + decelerationOffset);
        if (mCommandDuration <= 0.0 || !std::isfinite(mCommandDuration)) {
            EV_WARN << "McApplication CV station " << mEgoContext.stationId
                << " classified Decelerate but cannot apply slowDown yet"
                << ": targetSpeed=" << mTargetSpeed
                << " currentSpeed=" << mEgoContext.speed
                << " deceleration=" << plannedValues.deceleration_change
                << " offset=" << decelerationOffset
                << " decelerationTime=" << mCommandDuration
                << '\n';
        }
    } else if (plannedValues.acc_change > 0.0) {
        mControlManeuver = controlManeuver::Accelerate;
        mTargetSpeed = mEgoContext.speed + plannedValues.speed_change * mEgoContext.speed;
    } else {
        mControlManeuver = controlManeuver::DoNothing;
    }

    mCvSelectedTrajectory = selectedTrajectory;
    mHasCvSelectedTrajectory = !mCvSelectedTrajectory.empty();

    EV_INFO << "McApplication CV station " << mEgoContext.stationId
        << " classified merging control maneuver: " << controlManeuverName(mControlManeuver)
        << " using findSuitableTrajectoryCV"
        << " adaptationBranch=" << (decelerationRequired ? "decelerate-or-change-lane" : "accelerate-or-change-lane")
        << " requestId=" << snapshot.requestId
        << " priority=" << priority
        << " trajectoryType=" << trajectoryType
        << " possiblePriorityLevel=" << possiblePriorityLevel
        << " cost=" << trajectoryCost
        << " speedChange=" << plannedValues.speed_change
        << " acceleration=" << plannedValues.acc_change
        << " deceleration=" << plannedValues.deceleration_change
        << " laneChange=" << plannedValues.lane_change
        << " targetSpeed=" << mTargetSpeed
        << " decelerationTime=" << mCommandDuration
        << " egoLastY=" << myLastPoint.mY
        << " requestedLastY=" << requestedLastPoint.mY
        << '\n';
}

/*
 * Starts RV-side merging coordination once the configured route and trigger
 * conditions are met. Target selection, request identity, active trajectory
 * state, and diagnostic initialization must remain ordered with Request
 * command creation.
 */
void McApplication::evaluateMergingRequestTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    // RV-side medium-priority merging validation.
    // route_merging_1 is a scenario route, not a protocol condition. Once the
    // RV reaches the configured trigger point, it chooses current idle highway CVs by
    // trajectory/gap feasibility around the predicted merge point. The
    // pointwise conflict check is retained as supporting diagnostics; it is not
    // a fixed vehicle-ID mapping.
    if (!mHasEgoContext || !mVehicleDataProvider || !mVehicleController ||
            mPendingMcmCommand || mMergingRequestQueuedOrSent ||
            hasRvCoordinationFailure()) {
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
        EV_INFO << "McApplication route_merging_1 trigger condition reached for station "
            << mEgoContext.stationId << '\n';
    }

    if (mCoordinationProgressRV != coordinationProgressRV::CheckForCoordination) {
        return;
    }

    if (mEgoContext.plannedTrajectory.empty()) {
        EV_WARN << "McApplication route_merging_1 trigger has no ego plannedTrajectory; cannot request coordination\n";
        return;
    }

    std::unordered_map<uint32_t, const ReceivedMcm*> latestByStation;
    for (const auto& received : mReceivedMcmCache) {
        const auto stationId = received.data.stationId;
        if (stationId == 0) {
            continue;
        }
        const auto previous = latestByStation.find(stationId);
        if (previous == latestByStation.end() ||
                previous->second->receivedAt < received.receivedAt) {
            latestByStation[stationId] = &received;
        }
    }

    unsigned considered = 0;
    unsigned skippedBusy = 0;
    unsigned skippedStale = 0;
    unsigned skippedLane = 0;
    unsigned skippedTrajectory = 0;
    unsigned diagnosticPointConflicts = 0;
    std::vector<MergeTargetCandidate> candidates;

    for (const auto& item : latestByStation) {
        const auto& received = *item.second;
        const auto& snapshot = received.data;
        if (snapshot.stationId == mEgoContext.stationId) {
            continue;
        }

        ++considered;

        const omnetpp::SimTime age = std::max(omnetpp::SimTime::ZERO, now - received.receivedAt);
        if (age.dbl() > scMergeTargetMaxSnapshotAge) {
            ++skippedStale;
            continue;
        }

        if (snapshot.operationMode != operationMode::IntentionSharingMode ||
                snapshot.hasNegotiationContainer || snapshot.hasExecutionContainer) {
            ++skippedBusy;
            EV_INFO << "[MCM-MERGE-TARGET]"
                << " t=" << now
                << " event=skip-busy-candidate"
                << " rvStation=" << mEgoContext.stationId
                << " candidateStation=" << snapshot.stationId
                << " operationMode=" << operationModeName(snapshot.operationMode)
                << " hasNegotiationContainer=" << snapshot.hasNegotiationContainer
                << " hasExecutionContainer=" << snapshot.hasExecutionContainer
                << " age=" << age
                << '\n';
            continue;
        }

        if (!snapshot.hasLaneId || snapshot.laneId != 0) {
            ++skippedLane;
            continue;
        }

        if (snapshot.plannedTrajectory.empty()) {
            ++skippedTrajectory;
            continue;
        }

        const auto& first = snapshot.plannedTrajectory.front();
        if (first.mX == 1.0 && first.mY == 1.0) {
            ++skippedTrajectory;
            continue;
        }

        if (first.mY <= scHighwayLane0MinY || first.mY >= scHighwayLane0MaxY) {
            ++skippedLane;
            continue;
        }

        const bool diagnosticPointConflict = mTrajectoryPlanner.check_traj_conflict_merging(
            mEgoContext.plannedTrajectory,
            snapshot.plannedTrajectory,
            scMergingTimeGap,
            age,
            true);

        const std::size_t rvIndex = mEgoContext.plannedTrajectory.size() - 1;
        const std::size_t cvIndex = std::min(rvIndex, snapshot.plannedTrajectory.size() - 1);
        const auto& rvPoint = mEgoContext.plannedTrajectory[rvIndex];
        const auto& cvPoint = snapshot.plannedTrajectory[cvIndex];
        const double signedLongitudinalGap = cvPoint.mY - rvPoint.mY;
        const double dx = cvPoint.mX - rvPoint.mX;
        const double dy = cvPoint.mY - rvPoint.mY;

        MergeTargetCandidate candidate;
        candidate.stationId = snapshot.stationId;
        candidate.signedLongitudinalGap = signedLongitudinalGap;
        candidate.absLongitudinalGap = std::abs(signedLongitudinalGap);
        candidate.euclideanDistance = std::sqrt(dx * dx + dy * dy);
        candidate.rvY = rvPoint.mY;
        candidate.cvY = cvPoint.mY;
        candidate.age = age;
        candidate.diagnosticPointConflict = diagnosticPointConflict;

        const double mergeGapWindow =
            std::max(desiredDistanceGap, scMergingTimeGap * std::max(mEgoContext.speed, 1.0) * 2.5);
        if (candidate.absLongitudinalGap > mergeGapWindow) {
            continue;
        }

        candidates.push_back(candidate);
        if (diagnosticPointConflict) {
            ++diagnosticPointConflicts;
        }

        EV_INFO << "[MCM-MERGE-TARGET]"
            << " t=" << now
            << " event=gap-candidate"
            << " rvStation=" << mEgoContext.stationId
            << " candidateStation=" << snapshot.stationId
            << " laneId=" << snapshot.laneId
            << " operationMode=" << operationModeName(snapshot.operationMode)
            << " age=" << age
            << " rvY=" << candidate.rvY
            << " cvY=" << candidate.cvY
            << " signedLongitudinalGap=" << candidate.signedLongitudinalGap
            << " absLongitudinalGap=" << candidate.absLongitudinalGap
            << " euclideanDistance=" << candidate.euclideanDistance
            << " mergeGapWindow=" << mergeGapWindow
            << " diagnosticPointConflict=" << candidate.diagnosticPointConflict
            << '\n';
    }

    const MergeTargetSelection selection = selectMergeGapTargets(candidates);
    const uint32_t target1 = selection.target1;
    const uint32_t target2 = selection.target2;

    EV_INFO << "[MCM-MERGE-TARGET]"
        << " t=" << now
        << " event=selection-summary"
        << " rvStation=" << mEgoContext.stationId
        << " latestStations=" << latestByStation.size()
        << " considered=" << considered
        << " skippedBusy=" << skippedBusy
        << " skippedStale=" << skippedStale
        << " skippedLane=" << skippedLane
        << " skippedTrajectory=" << skippedTrajectory
        << " gapCandidates=" << candidates.size()
        << " diagnosticPointConflicts=" << diagnosticPointConflicts
        << " target1=" << target1
        << " target2=" << target2
        << '\n';

    EV_INFO << "McApplication route_merging_1 considered " << considered
        << " latest planned trajectories; gapCandidates=" << candidates.size()
        << " diagnosticPointConflicts=" << diagnosticPointConflicts << '\n';

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
    mRvLastRequestQueuedAt = now;
    mHasRvLastRequestQueuedAt = true;
    mRvNegotiationStartedAt = now;
    mHasRvNegotiationStartedAt = true;

    // Store active RV-side negotiation state.
    // This is needed later to match incoming Offer/Accept/Reject messages
    // against the original Request and the selected CV targets.
    mRvRequestId = command.requestId;
    mRvNumberOfVehicles = command.numberOfVehicles;
    mRvTargetVehicle1 = command.targetVehicle1;
    mRvTargetVehicle2 = command.hasTargetVehicle2 ? command.targetVehicle2 : 0;

    mRvOfferReceived1 = false;
    mRvOfferReceived2 = false;
    mRvConfirmQueuedOrSent = false;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mRvNegotiationCompletionReported = false;
    mCompletedRvNegotiationRequestId.reset();
    mRvSecondRequestAttempted = false;
    mRvSecondRequestCompletedMeasured = false;
    mRvSecondRequestRejectedMeasured = false;
    mRvRequestedTrajectory = command.requestedTrajectory;
    mHasRvRequestedTrajectory = !mRvRequestedTrajectory.empty();
    mRvNegotiatedTrajectory.clear();
    mHasRvNegotiatedTrajectory = false;
    EV_INFO << "[MCM-TRAJECTORY]"
        << " event=rv-requested-trajectory-active"
        << " station=" << mEgoContext.stationId
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " trajectoryRole=requested"
        << " source=initial-request"
        << " trajectoryPoints=" << mRvRequestedTrajectory.size()
        << '\n';
    mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
    mHasLastExecuteQueuedAt = false;
    mSafetyCriticalLaneChangeExecutionActive = false;
    mLaneChangeMoveStepCounter = 0;
    mSafetyCriticalLaneChangeExecutionStartedAt = omnetpp::SimTime::ZERO;
    mLastSafetyCriticalLaneChangeMoveAt = omnetpp::SimTime::ZERO;
    resetMergingGapDiagnostics();

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

} // namespace mcm
} // namespace artery
