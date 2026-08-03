#include "artery/application/mcm/McApplication.h"

#include "artery/traci/VehicleController.h"

#include <omnetpp.h>

#include <iostream>

/*
 * Implements retry and timeout handling for McApplication.
 *
 * This file evaluates RV and CV retry timers, rebuilds pending retry commands,
 * and resets negotiation state after configured timeout limits. The ordering of
 * these state changes is part of the protocol behavior and should be preserved.
 *
 * The source split is organizational only. All definitions are member functions
 * of the single McApplication class declared in McApplication.h.
 */

namespace artery
{
namespace mcm
{

omnetpp::SimTime McApplication::activeRvNegotiationLimit() const
{
    return isHighPriorityLaneChangeRequestActive() ?
        mNegotiationLimitLaneChange :
        mNegotiationLimitMerging;
}

bool McApplication::hasRvNegotiationTimedOut(
    omnetpp::SimTime now,
    omnetpp::SimTime limit) const
{
    return mHasRvNegotiationStartedAt &&
        now - mRvNegotiationStartedAt >= limit;
}

bool McApplication::hasCvNegotiationTimedOut(omnetpp::SimTime now) const
{
    return mHasCvNegotiationStartedAt &&
        now - mCvNegotiationStartedAt >= mNegotiationLimitMerging;
}

bool McApplication::shouldRetryAfter(
    omnetpp::SimTime now,
    omnetpp::SimTime lastQueuedAt,
    bool hasLastQueuedAt) const
{
    return !hasLastQueuedAt || now - lastQueuedAt >= mNegotiationRetryInterval;
}

// RV-side Request retransmission. The Request keeps the original target set
// and request ID; one-CV and two-CV behavior is decided by mRvNumberOfVehicles.
PendingMcmCommand McApplication::makeRvRequestRetryCommand() const
{
    EV_STATICCONTEXT;

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Request;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = static_cast<long>(mCoordinationManeuver);
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles >= 2 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mRvRequestedTrajectory;
    EV_INFO << "[MCM-TRAJECTORY]"
        << " event=rv-request-retry"
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " trajectoryRole=requested"
        << " trajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';
    return command;
}

// RV-side Confirm retransmission. Confirm retains the active Request proposal.
PendingMcmCommand McApplication::makeRvConfirmRetryCommand() const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Confirm;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = static_cast<long>(mCoordinationManeuver);
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles >= 2 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mRvRequestedTrajectory.empty()
        ? mEgoContext.plannedTrajectory
        : mRvRequestedTrajectory;
    return command;
}

// CV-side Offer retransmission while waiting for Confirm. The offer trajectory
// still comes from the selected CV maneuver if one was computed.
PendingMcmCommand McApplication::makeCvOfferRetryCommand() const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Offer;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = static_cast<long>(mCoordinationManeuver);
    command.requestId = mCvRequestId;
    command.numberOfVehicles = 1;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;
    command.requestedTrajectory = mHasEgoContext ?
        mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};
    command.offeredTrajectory = mHasCvSelectedTrajectory
        ? mCvSelectedTrajectory
        : (mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {});
    command.hasOfferedTrajectory = !command.offeredTrajectory.empty();
    return command;
}

// CV-side Accept retransmission while waiting for Execute. This intentionally
// preserves the response vehicle count from the original Request path.
PendingMcmCommand McApplication::makeCvAcceptRetryCommand() const
{
    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Accept;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = static_cast<long>(mCoordinationManeuver);
    command.requestId = mCvRequestId;
    command.numberOfVehicles = mCvResponseNumberOfVehicles;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;
    command.requestedTrajectory = mHasEgoContext ?
        mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};
    return command;
}

// RV timeout reset for unsuccessful medium-priority merging negotiations.
// High-priority lane-change timeouts use the emergency fallback brake instead,
// so this helper preserves the merging-only reset semantics.
void McApplication::resetRvNegotiationAfterTimeout()
{
    mMergingRequestQueuedOrSent = false;
    resetRvResponseTracking();
    mRvNegotiationCompletionReported = false;
    mCompletedRvNegotiationRequestId.reset();
    mRvSecondRequestAttempted = false;
    mRvSecondRequestCompletedMeasured = false;
    mRvSecondRequestRejectedMeasured = false;
    setRvCoordinationFailure(RvCoordinationFailureReason::Timeout, "negotiation-timeout");

    resetRvRetryTracking();
    clearRvTrajectoryState();

    mOperationMode = operationMode::IntentionSharingMode;
    mCoordinationProgressRV = coordinationProgressRV::NoCoordination;
}

// CV timeout reset for Offer/Accept retries. It clears only the negotiation
// bookkeeping used by timeout paths; execution/control flags are
// left untouched to avoid changing participant state outside negotiation.
void McApplication::resetCvNegotiationAfterTimeout()
{
    mPendingMcmCommand.reset();
    mCoordinationProgressCV = coordinationProgressCV::NoRequest;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;
    resetCvActiveNegotiationTracking();
    mCvHasRejectedRequest = false;
    mCvRejectedRvStationId = 0;
    mCvRejectedRequestId = 0;
    clearCvTrajectoryState();
}

void McApplication::evaluateRvRequestRetry(omnetpp::SimTime now)
{
    // RV-side Request retry while waiting for Offer messages. Two-CV
    // negotiations require both Offers; one-CV high-priority lane-change
    // requests may complete directly with Accept, so that direct path is kept.
    const bool laneChangeRequestActive = isHighPriorityLaneChangeRequestActive();
    if (!isRvNegotiationRequestActive()) {
        return;
    }

    if (mRvConfirmQueuedOrSent) {
        return;
    }

    const omnetpp::SimTime negotiationLimit = activeRvNegotiationLimit();

    if (mRvNumberOfVehicles < 2) {
        if (laneChangeRequestActive &&
                mCoordinationProgressRV == coordinationProgressRV::RequestSent &&
                !mRvAcceptReceived1 &&
                hasRvNegotiationTimedOut(now, negotiationLimit)) {
            EV_STATICCONTEXT;
            EV_INFO << "[MCM-NEGOTIATION-TIMEOUT]"
                << " t=" << now
                << " role=RV"
                << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
                << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
                << " event=timeout-waiting-for-accept"
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " expectedCv1=" << mRvTargetVehicle1
                << " elapsed=" << (now - mRvNegotiationStartedAt)
                << " limit=" << negotiationLimit
                << '\n';

            applyEmergencyFallbackBrake(
                "timeout-brake",
                "timeout-waiting-for-accept",
                mRvRequestId,
                RvCoordinationFailureReason::Timeout);
        }
        return;
    }

    if (haveAllExpectedRvOffers()) {
        return;
    }

    if (hasRvNegotiationTimedOut(now, negotiationLimit)) {
        EV_STATICCONTEXT;
        EV_INFO << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " role=RV"
            << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " event=timeout-waiting-for-offers"
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " expectedCv1=" << mRvTargetVehicle1
            << " expectedCv2=" << mRvTargetVehicle2
            << " offer1Received=" << mRvOfferReceived1
            << " offer2Received=" << mRvOfferReceived2
            << " elapsed=" << (now - mRvNegotiationStartedAt)
            << " limit=" << negotiationLimit
            << '\n';

        std::cout << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " TIMEOUT waiting-for-offers"
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " target1=" << mRvTargetVehicle1
            << " target2=" << mRvTargetVehicle2
            << " offer1Received=" << mRvOfferReceived1
            << " offer2Received=" << mRvOfferReceived2
            << " elapsed=" << (now - mRvNegotiationStartedAt)
            << " limit=" << negotiationLimit
            << std::endl;

        if (laneChangeRequestActive) {
            applyEmergencyFallbackBrake(
                "timeout-brake",
                "timeout-waiting-for-offers",
                mRvRequestId,
                RvCoordinationFailureReason::Timeout);
            return;
        }

        resetRvNegotiationAfterTimeout();
        return;
    }

    if (mRvSecondRequestAttempted) {
        return;
    }

    if (!shouldRetryAfter(now, mRvLastRequestQueuedAt, mHasRvLastRequestQueuedAt)) {
        return;
    }

    mPendingMcmCommand = makeRvRequestRetryCommand();
    mRvLastRequestQueuedAt = now;
    mHasRvLastRequestQueuedAt = true;

    EV_STATICCONTEXT;
    EV_INFO << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " role=RV"
        << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " event=resend-request"
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " expectedCv1=" << mRvTargetVehicle1
        << " expectedCv2=" << mRvTargetVehicle2
        << " offer1Received=" << mRvOfferReceived1
        << " offer2Received=" << mRvOfferReceived2
        << " retryInterval=" << mNegotiationRetryInterval
        << '\n';

    std::cout << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " RESEND Request"
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " target1=" << mRvTargetVehicle1
        << " target2=" << mRvTargetVehicle2
        << " offer1Received=" << mRvOfferReceived1
        << " offer2Received=" << mRvOfferReceived2
        << std::endl;
}

void McApplication::evaluateRvConfirmRetry(omnetpp::SimTime now)
{
    // RV-side Confirm retry while waiting for Accept messages. Two-CV flow
    // only resends Confirm for two-CV negotiation; one-CV Accept handling stays
    // on the direct Request->Accept path.
    const bool laneChangeRequestActive = isHighPriorityLaneChangeRequestActive();
    if (!isRvNegotiationRequestActive()) {
        return;
    }

    if (!mRvConfirmQueuedOrSent) {
        return;
    }

    if (mRvExecuteQueuedOrSent) {
        return;
    }

    if (mRvNumberOfVehicles < 2) {
        return;
    }

    if (haveAllExpectedRvAccepts()) {
        return;
    }

    const omnetpp::SimTime negotiationLimit = activeRvNegotiationLimit();

    if (hasRvNegotiationTimedOut(now, negotiationLimit)) {
        EV_STATICCONTEXT;
        EV_INFO << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " role=RV"
            << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " event=timeout-waiting-for-accepts"
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " expectedCv1=" << mRvTargetVehicle1
            << " expectedCv2=" << mRvTargetVehicle2
            << " accept1Received=" << mRvAcceptReceived1
            << " accept2Received=" << mRvAcceptReceived2
            << " elapsed=" << (now - mRvNegotiationStartedAt)
            << " limit=" << negotiationLimit
            << '\n';

        std::cout << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " TIMEOUT waiting-for-accepts"
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " target1=" << mRvTargetVehicle1
            << " target2=" << mRvTargetVehicle2
            << " accept1Received=" << mRvAcceptReceived1
            << " accept2Received=" << mRvAcceptReceived2
            << " elapsed=" << (now - mRvNegotiationStartedAt)
            << " limit=" << negotiationLimit
            << std::endl;

        if (laneChangeRequestActive) {
            applyEmergencyFallbackBrake(
                "timeout-brake",
                "timeout-waiting-for-accepts",
                mRvRequestId,
                RvCoordinationFailureReason::Timeout);
            return;
        }

        resetRvNegotiationAfterTimeout();
        return;
    }

    if (!shouldRetryAfter(now, mRvLastConfirmQueuedAt, mHasRvLastConfirmQueuedAt)) {
        return;
    }

    mPendingMcmCommand = makeRvConfirmRetryCommand();
    mRvLastConfirmQueuedAt = now;
    mHasRvLastConfirmQueuedAt = true;

    EV_STATICCONTEXT;
    EV_INFO << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " role=RV"
        << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " event=resend-confirm"
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " expectedCv1=" << mRvTargetVehicle1
        << " expectedCv2=" << mRvTargetVehicle2
        << " accept1Received=" << mRvAcceptReceived1
        << " accept2Received=" << mRvAcceptReceived2
        << " retryInterval=" << mNegotiationRetryInterval
        << '\n';

    std::cout << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " RESEND Confirm"
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " target1=" << mRvTargetVehicle1
        << " target2=" << mRvTargetVehicle2
        << " accept1Received=" << mRvAcceptReceived1
        << " accept2Received=" << mRvAcceptReceived2
        << std::endl;
}

void McApplication::evaluateCvOfferRetry(omnetpp::SimTime now)
{
    // CV-side Offer retry while waiting for Confirm. This uses the merging
    // negotiation limit currently shared by CV negotiation paths and keeps
    // resending the same Offer content until Confirm or timeout.
    if (mCooperatingVehicleType != cooperatingVehicleType::CV) {
        return;
    }

    if (!mCvResponseQueuedOrSent) {
        return;
    }

    if (mCoordinationProgressCV != coordinationProgressCV::SendOffer) {
        return;
    }

    if (!mHasCvLastOfferQueuedAt) {
        return;
    }

    if (hasCvNegotiationTimedOut(now)) {
        EV_STATICCONTEXT;
        EV_INFO << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " role=CV"
            << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " event=timeout-waiting-for-confirm"
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " rvStation=" << mCvRvStationId
            << " elapsed=" << (now - mCvNegotiationStartedAt)
            << " limit=" << mNegotiationLimitMerging
            << '\n';

        std::cout << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " TIMEOUT waiting-for-confirm"
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " rvStation=" << mCvRvStationId
            << " elapsed=" << (now - mCvNegotiationStartedAt)
            << " limit=" << mNegotiationLimitMerging
            << std::endl;

        resetCvNegotiationAfterTimeout();
        return;
    }

    if (!shouldRetryAfter(now, mCvLastOfferQueuedAt, mHasCvLastOfferQueuedAt)) {
        return;
    }

    mPendingMcmCommand = makeCvOfferRetryCommand();
    mCvLastOfferQueuedAt = now;
    mHasCvLastOfferQueuedAt = true;

    EV_STATICCONTEXT;
    EV_INFO << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " role=CV"
        << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " event=resend-offer"
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " rvStation=" << mCvRvStationId
        << " retryInterval=" << mNegotiationRetryInterval
        << '\n';

    std::cout << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " RESEND Offer"
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " rvStation=" << mCvRvStationId
        << std::endl;
}

void McApplication::evaluateCvAcceptRetry(omnetpp::SimTime now)
{
    // CV-side Accept retry while waiting for Execute. Accept retry starts only
    // after the sent-message handler marks AcceptSent, preserving duplicate
    // tolerance and the recent Accept timestamp bookkeeping fix.
    if (mCooperatingVehicleType != cooperatingVehicleType::CV) {
        return;
    }

    if (!mCvResponseQueuedOrSent) {
        return;
    }

    if (mCoordinationProgressCV != coordinationProgressCV::AcceptSent) {
        return;
    }

    if (!mHasCvLastAcceptQueuedAt) {
        return;
    }

    if (hasCvNegotiationTimedOut(now)) {
        EV_STATICCONTEXT;
        EV_INFO << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " role=CV"
            << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " event=timeout-waiting-for-execute"
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " rvStation=" << mCvRvStationId
            << " elapsed=" << (now - mCvNegotiationStartedAt)
            << " limit=" << mNegotiationLimitMerging
            << '\n';

        std::cout << "[MCM-NEGOTIATION-TIMEOUT]"
            << " t=" << now
            << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " TIMEOUT waiting-for-execute"
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " rvStation=" << mCvRvStationId
            << " elapsed=" << (now - mCvNegotiationStartedAt)
            << " limit=" << mNegotiationLimitMerging
            << std::endl;

        resetCvNegotiationAfterTimeout();
        return;
    }

    if (!shouldRetryAfter(now, mCvLastAcceptQueuedAt, mHasCvLastAcceptQueuedAt)) {
        return;
    }

    mPendingMcmCommand = makeCvAcceptRetryCommand();
    mCvLastAcceptQueuedAt = now;
    mHasCvLastAcceptQueuedAt = true;

    EV_STATICCONTEXT;
    EV_INFO << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " role=CV"
        << " vehicle=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " event=resend-accept"
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " rvStation=" << mCvRvStationId
        << " retryInterval=" << mNegotiationRetryInterval
        << '\n';

    std::cout << "[MCM-NEGOTIATION-RETRY]"
        << " t=" << now
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " RESEND Accept"
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " rvStation=" << mCvRvStationId
        << std::endl;
}

} // namespace mcm
} // namespace artery
