#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/traci/VehicleController.h"

#include <algorithm>
#include <iostream>
#include <tuple>

/*
 * Implements common negotiation support for McApplication.
 *
 * This file contains shared guards, command builders, completion predicates,
 * coordination-state resets, and second-request construction used by multiple
 * protocol handlers. The methods preserve the existing one-CV and two-CV
 * sequencing while operating on the single shared McApplication object.
 *
 * The source split is organizational only; it does not reduce state coupling or
 * create an independent negotiation component.
 */

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scRequestTrajectoryDt;
using scenario::scRequestTrajectorySteps;

} // namespace

// Shared message guard used by RV and CV handlers. It preserves retry
// behavior by accepting duplicate messages for the active request while
// filtering unrelated subtypes/request IDs before any state transition.
bool McApplication::isNegotiationMessageForActiveRequest(
    const McmSnapshot& snapshot,
    mcmSubtype subtype,
    uint8_t requestId) const
{
    return snapshot.hasNegotiationContainer &&
        snapshot.mcmCategory == static_cast<long>(subtype) &&
        snapshot.requestId >= 0 &&
        static_cast<uint8_t>(snapshot.requestId) == requestId;
}

bool McApplication::isExecuteEvidenceForActiveRvRequest(const McmSnapshot& snapshot) const
{
    if (snapshot.hasNegotiationContainer &&
            snapshot.mcmCategory == static_cast<long>(mcmSubtype::Execute) &&
            snapshot.requestId >= 0 &&
            static_cast<uint8_t>(snapshot.requestId) == mRvRequestId) {
        return true;
    }

    return snapshot.hasExecutionContainer &&
        snapshot.mcmCategory == static_cast<long>(mcmSubtype::Execute) &&
        snapshot.cooperationId >= 0 &&
        static_cast<uint8_t>(snapshot.cooperationId) == mRvRequestId;
}

// CV-side target guard for Confirm/Execute. Both one-CV and two-CV
// negotiations use the same ASN.1 target fields, so this helper only checks
// whether the local station appears in the message without changing flow type.
bool McApplication::isSnapshotTargetingEgo(const McmSnapshot& snapshot) const
{
    if (!mVehicleDataProvider) {
        return false;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    return snapshot.negotiationVehicleId1 == egoStationId ||
        (snapshot.hasNegotiationVehicleId2 && snapshot.negotiationVehicleId2 == egoStationId);
}

// RV-side duplicate-tolerant response marker. A response from each expected CV
// is counted once; later retransmissions are ignored by the callers exactly as
// before, so retry/timeout semantics remain unchanged.
bool McApplication::markRvResponseFromExpectedCv(
    uint32_t senderStationId,
    bool& fromTarget1,
    bool& fromTarget2) const
{
    fromTarget1 = senderStationId == mRvTargetVehicle1;
    fromTarget2 = mRvNumberOfVehicles > 1 && senderStationId == mRvTargetVehicle2;
    return fromTarget1 || fromTarget2;
}

// RV-side command builder for follow-up negotiation messages. Offer processing
// uses this for Confirm and Accept processing uses it for Execute, preserving
// the active target set and the one-CV/two-CV distinction.
PendingMcmCommand McApplication::makeRvFollowupCommand(
    mcmSubtype subtype,
    long cooperationType) const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = subtype;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = cooperationType;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;
    return command;
}

// CV-side Accept builder after Confirm. The selected CV trajectory is reused
// when available; this keeps cooperation-cost decisions intact and only
// centralizes the message construction.
PendingMcmCommand McApplication::makeCvAcceptCommand(const McmSnapshot& snapshot) const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Accept;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = snapshot.cooperationTypeMcm >= 0 ? snapshot.cooperationTypeMcm : 0;
    command.requestId = mCvRequestId;
    command.numberOfVehicles = snapshot.numberOfVehicles > 0 ?
        static_cast<uint8_t>(snapshot.numberOfVehicles) : 1;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;
    command.requestedTrajectory = mHasCvSelectedTrajectory ?
        mCvSelectedTrajectory :
        (mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {});
    return command;
}

// RV-side completion reset. This is reached after the Complete-as-Cancel
// workaround is sent, so retry timestamps, received-response flags, execution
// state, and scenario diagnostics are cleared together.
void McApplication::resetRvCoordinationStateAfterComplete()
{
    mCoordinationProgressRV = coordinationProgressRV::CompleteSent;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;

    mMergingRequestQueuedOrSent = false;
    mLaneChangeRequestQueuedOrSent = false;
    mLaneChangeThreeVehiclePath = false;
    mRvOfferReceived1 = false;
    mRvOfferReceived2 = false;
    mRvLastRequestQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastRequestQueuedAt = false;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasRvNegotiationStartedAt = false;
    mRvConfirmQueuedOrSent = false;
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mRvNegotiationCompletionReported = false;
    mCompletedRvNegotiationRequestId.reset();
    mRvSecondRequestAttempted = false;
    mRvSecondRequestCompletedMeasured = false;
    mRvSecondRequestRejectedMeasured = false;
    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;
    mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
    mHasLastExecuteQueuedAt = false;
    mRvMergingExecutionControlLogged = false;
    mSafetyCriticalLaneChangeExecutionActive = false;
    mLaneChangeMoveStepCounter = 0;
    mSafetyCriticalLaneChangeExecutionStartedAt = omnetpp::SimTime::ZERO;
    mLastSafetyCriticalLaneChangeMoveAt = omnetpp::SimTime::ZERO;
    mControlManeuver = controlManeuver::DoNothing;
    resetMergingGapDiagnostics();
}

// CV-side completion reset for the normal Complete-as-Cancel path. Early
// Cancel rollback reuses the same cleanup but restores the NoRequest
// progress state afterwards.
void McApplication::resetCvCoordinationStateAfterComplete()
{
    mCoordinationProgressCV = coordinationProgressCV::CompleteSentCV;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;

    mCvResponseQueuedOrSent = false;
    mCvRvStationId = 0;
    mCvRequestId = 0;
    mCvResponseNumberOfVehicles = 1;
    mCvHasRejectedRequest = false;
    mCvRejectedRvStationId = 0;
    mCvRejectedRequestId = 0;
    mCvLastOfferQueuedAt = omnetpp::SimTime::ZERO;
    mHasCvLastOfferQueuedAt = false;
    mCvLastAcceptQueuedAt = omnetpp::SimTime::ZERO;
    mHasCvLastAcceptQueuedAt = false;
    mCvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasCvNegotiationStartedAt = false;
    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;
    mControlManeuver = controlManeuver::DoNothing;
    mCvSelectedTrajectory.clear();
    mHasCvSelectedTrajectory = false;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mCvDecelerationControlApplied = false;
    mCvDecelerationControlSkippedLogged = false;
    mCvAccelerationControlApplied = false;
    mCvLaneChangeControlLogged = false;
    mCvTargetSpeedReachedLogged = false;
    mCvRestoreNormalSpeedSkippedLogged = false;
    mCvStoppedDecelerationForRvLogged = false;
}

bool McApplication::isHighPriorityLaneChangeRequestActive() const
{
    return mLaneChangeRequestQueuedOrSent &&
        mPriorityMcmCategory == priorityMcmCategory::HighPriority;
}

bool McApplication::isRvNegotiationRequestActive() const
{
    return mMergingRequestQueuedOrSent || isHighPriorityLaneChangeRequestActive();
}

bool McApplication::haveAllExpectedRvOffers() const
{
    return mRvNumberOfVehicles > 1 ?
        (mRvOfferReceived1 && mRvOfferReceived2) :
        mRvOfferReceived1;
}

bool McApplication::haveAllExpectedRvAccepts() const
{
    return mRvNumberOfVehicles > 1 ?
        (mRvAcceptReceived1 && mRvAcceptReceived2) :
        mRvAcceptReceived1;
}

/*
 * Builds the high-priority RV second Request after a Reject. The method keeps
 * the existing request-id update, target selection fallback, trajectory search,
 * and planner measurements together so the retry and Reject handlers can treat
 * the command as one atomic follow-up.
 */
std::optional<PendingMcmCommand> McApplication::makeRvSecondRequestCommand(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || mEgoContext.plannedTrajectory.empty() ||
            mEgoContext.routeReferenceX.empty() || mEgoContext.routeReferenceY.empty() ||
            mEgoContext.routeReferenceIndex < 0) {
        return std::nullopt;
    }

    const uint32_t rejectingCv = received.data.stationId;
    TrajectoryPlanner::Trajectory otherTrajectory = received.data.plannedTrajectory;
    for (auto it = mReceivedMcmCache.rbegin(); it != mReceivedMcmCache.rend(); ++it) {
        if (it->data.stationId == rejectingCv && !it->data.plannedTrajectory.empty()) {
            otherTrajectory = it->data.plannedTrajectory;
            break;
        }
    }

    if (otherTrajectory.empty()) {
        return std::nullopt;
    }

    const auto& rvPoint = mEgoContext.plannedTrajectory.front();
    const auto& cvPoint = otherTrajectory.front();
    bool decelerationRequired = rvPoint.mY >= cvPoint.mY;
    bool accelerationRequired = !decelerationRequired;

    TrajectoryPlanner::Trajectory estimatedOtherTrajectory = otherTrajectory;
    if (accelerationRequired) {
        const int requestPriority = priorityLevel(mPriorityMcmCategory);
        const double speedFactor = requestPriority <= 0 ? 0.2 : (requestPriority == 1 ? 0.4 : 0.5);
        const double deceleration = requestPriority <= 0 ? 2.0 : (requestPriority == 1 ? 4.0 : 5.0);
        estimatedOtherTrajectory = mTrajectoryPlanner.estimateOtherDecelerationTrajectory(
            scRequestTrajectorySteps,
            scRequestTrajectoryDt,
            mEgoContext.routeReferenceX,
            mEgoContext.routeReferenceY,
            mEgoContext.routeReferenceIndex,
            false,
            static_cast<float>(mEgoContext.speed - speedFactor * mEgoContext.speed),
            0.0F,
            static_cast<float>(deceleration),
            otherTrajectory);
        if (estimatedOtherTrajectory.empty()) {
            estimatedOtherTrajectory = otherTrajectory;
        }
    }

    const omnetpp::SimTime eteDelay =
        std::max(omnetpp::SimTime::ZERO, mEgoContext.now - received.receivedAt);
    auto result = mTrajectoryPlanner.findSecondRequestTrajRV(
        estimatedOtherTrajectory,
        priorityLevel(mPriorityMcmCategory),
        scRequestTrajectorySteps,
        scRequestTrajectoryDt,
        mEgoContext.routeReferenceX,
        mEgoContext.routeReferenceY,
        mEgoContext.routeReferenceIndex,
        mEgoContext.speed,
        eteDelay,
        decelerationRequired,
        accelerationRequired);

    const bool foundTrajectory = std::get<0>(result);
    const auto& secondRequestTrajectory = std::get<1>(result);
    if (secondRequestTrajectory.empty()) {
        return std::nullopt;
    }

    uint8_t newRequestId = makeRequestId(mEgoContext.now + mNegotiationRetryInterval);
    if (newRequestId == mRvRequestId) {
        newRequestId = static_cast<uint8_t>(newRequestId + 1);
    }
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Request;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = static_cast<long>(mCoordinationManeuver);
    command.requestId = newRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = secondRequestTrajectory;
    command.hasTrajectoryCostRv = true;
    command.trajectoryCostRv = std::get<3>(result);

    EV_INFO << "[MCM-LC-STATE]"
        << " simTime=" << mEgoContext.now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " event=queued-second-request"
        << " previousRequestId=" << static_cast<int>(mRvRequestId)
        << " requestId=" << static_cast<int>(newRequestId)
        << " rejectingCv=" << rejectingCv
        << " targetCv1=" << command.targetVehicle1
        << " targetCv2=" << command.targetVehicle2
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " trajectoryFound=" << foundTrajectory
        << " trajectoryCostRv=" << std::get<3>(result)
        << " trajectoryTypeRv=" << std::get<4>(result)
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';

    std::cout << "[MCM-LC-STATE]"
        << " t=" << mEgoContext.now
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << mEgoContext.stationId
        << " QUEUE second Request"
        << " previousRequestId=" << static_cast<int>(mRvRequestId)
        << " requestId=" << static_cast<int>(newRequestId)
        << " rejectingCv=" << rejectingCv
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " trajectoryFound=" << foundTrajectory
        << std::endl;

    return command;
}

}  // namespace mcm
}  // namespace artery
