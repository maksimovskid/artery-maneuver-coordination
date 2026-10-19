#include "artery/application/mcm/McApplication.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include <omnetpp.h>
#include "artery/application/mcm/McmTrajectory.h"
#include "artery/application/mcm/McmTiming.h"
#include <algorithm>
#include <cmath>

namespace artery { namespace mcm {
bool McApplication::acceptRequestVersion(const ReceivedMcm& received)
{
    const auto& s = received.data;
    if (!s.hasNegotiationContainer || s.mcmCategory != static_cast<long>(mcmSubtype::Request) ||
            !isSnapshotTargetingEgo(s)) return true;
    const auto id = static_cast<uint8_t>(s.requestId);
    auto& retired = mRetiredRequestVersions[s.stationId];
    for (auto it = retired.begin(); it != retired.end();) {
        if (it->second < received.receivedAt - omnetpp::SimTime(32.768)) it = retired.erase(it);
        else ++it;
    }
    auto prior = mLatestRequestVersion.find(s.stationId);
    if (prior != mLatestRequestVersion.end()) {
        const auto incoming = std::make_pair(id, received.generatedAt);
        if (received.generatedAt < prior->second.second ||
                retired.count(incoming)) return false;
    }
    if (mCvActiveRequest && mCvRvStationId == s.stationId) {
        if (id == mCvRequestId) {
            if (received.generatedAt != mCvActiveRequest->generatedAt ||
                    !sameMcmProposal(s.requestedTrajectory, mCvActiveRequest->data.requestedTrajectory)) {
                EV_STATICCONTEXT;
                EV_WARN << "[MCM-PROPOSAL] drop changed Request under same ID sender=" << s.stationId
                    << " requestId=" << s.requestId << '\n';
                return false;
            }
        } else {
            // A newer Request invalidates both the old pending response and the
            // old Offer/Accept chain, but cannot replace an executing agreement.
            if (mOperationMode == operationMode::ManeuverExecutionMode) return false;
            resetCvNegotiationAfterTimeout();
        }
    }
    const auto incoming = std::make_pair(id, received.generatedAt);
    if (prior != mLatestRequestVersion.end() && incoming != prior->second) retired.insert(prior->second);
    mLatestRequestVersion[s.stationId] = incoming;
    return true;
}

void McApplication::refreshLiveNegotiationCandidates()
{
    if (!mHasEgoContext) return;
    if (isRvNegotiationRequestActive() && !mRvConfirmQueuedOrSent && !mRvExecuteQueuedOrSent) {
        auto candidate = mEgoContext.plannedTrajectory;
        if (mRvSecondRequestAttempted && mRvSecondRequestSource && mVehicleDataProvider) {
            auto second = makeRvSecondRequestCommand(*mRvSecondRequestSource, false);
            if (second) candidate = second->requestedTrajectory;
            else candidate.clear();
        } else if (isHighPriorityLaneChangeRequestActive()) {
            auto shifted = mEgoContext.routeReferenceX;
            for (auto& x : shifted) x += mSafetyCriticalLaneChangeConfig.plannedLateralShift;
            if (mVehicleController && !shifted.empty()) {
                candidate = mTrajectoryPlanner.calculateRefTrajectory(scenario::scRequestTrajectorySteps,
                    scenario::scRequestTrajectoryDt, shifted, mEgoContext.routeReferenceY,
                    mEgoContext.routeReferenceIndex, true, 0, 0, 0);
            } else {
                for (auto& point : candidate) point.mX += mSafetyCriticalLaneChangeConfig.plannedLateralShift;
            }
        }
        mRvLiveRequestedTrajectory = candidate;
        // DCC can defer the FIRST serialization by many ticks. Before issuance
        // no remote participant can refer to this proposal, so rebuilding is safe.
        // Afterwards retries deliberately retain its original content AND origin.
        if (!mRvProposalIssued && !candidate.empty()) {
            mRvRequestedTrajectory = candidate;
            mHasRvRequestedTrajectory = true;
            if (mRvRequestCommand) mRvRequestCommand->requestedTrajectory = candidate;
            if (mPendingMcmCommand && mPendingMcmCommand->subtype == mcmSubtype::Request)
                mPendingMcmCommand->requestedTrajectory = candidate;
        }
    }
    if (mCvActiveRequest && !mCvProposalIssued && mPendingMcmCommand &&
            (mPendingMcmCommand->subtype == mcmSubtype::Offer ||
             mPendingMcmCommand->subtype == mcmSubtype::Accept) &&
            mVehicleDataProvider) {
        const auto& trajectory = mPendingMcmCommand->hasOfferedTrajectory ?
            mPendingMcmCommand->offeredTrajectory : mPendingMcmCommand->requestedTrajectory;
        if (!trajectory.empty() && trajectory.front().mReferenceTime == generationReferenceTime(mEgoContext.now)) return;
        // Re-evaluate only an unissued response from CURRENT CV state. Preserve
        // the negotiation deadline; DCC waiting must not extend it indefinitely.
        const auto request = *mCvActiveRequest;
        const auto started = mCvNegotiationStartedAt;
        mPendingMcmCommand.reset();
        mCvResponseQueuedOrSent = false;
        mCoordinationProgressCV = coordinationProgressCV::NoRequest;
        evaluateCvRequestResponse(request);
        mCvNegotiationStartedAt = started;
    }
}

void McApplication::refreshExecutionPrediction()
{
    // Prediction is separate from immutable m*NegotiatedTrajectory. Re-use the
    // established planner with current state and the existing execution target.
    if (!mVehicleController || !mVehicleDataProvider ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mEgoContext.routeReferenceX.empty() || mEgoContext.routeReferenceY.empty()) return;
    const bool rv = mCooperatingVehicleType == cooperatingVehicleType::RV;
    const auto& agreed = rv ? mRvNegotiatedTrajectory : mCvNegotiatedTrajectory;
    if (agreed.empty()) return;
    auto cx = mEgoContext.routeReferenceX;
    if (rv && isHighPriorityLaneChangeRequestActive()) {
        // Existing lane-change model follows the shifted reference path. This
        // offset is the AGREED target lane, never a newly selected maneuver.
        for (auto& x : cx) x += mSafetyCriticalLaneChangeConfig.plannedLateralShift;
    }
    const bool adapting = !rv && (mControlManeuver == controlManeuver::Accelerate ||
        mControlManeuver == controlManeuver::Decelerate);
    const double target = rv && !isHighPriorityLaneChangeRequestActive() ?
        mMergingExecutionConfig.rvExecutionSpeed : mTargetSpeed;
    const double acceleration = mCvPlannedValues.acc_change;
    const double deceleration = mCvPlannedValues.deceleration_change;
    const auto steps = mEgoContext.plannedTrajectory.size();
    if (steps == 0) return;
    const double dt = (mEgoContext.plannedTrajectory.front().mTime -
        mEgoContext.plannedTrajectory.front().mReferenceTime).dbl();
    mEgoContext.plannedTrajectory = mTrajectoryPlanner.calculateExecuteTrajectory(
        static_cast<int>(steps), dt,
        cx, mEgoContext.routeReferenceY, mEgoContext.routeReferenceIndex,
        !adapting, target, acceleration, deceleration, agreed,
        target, acceleration, deceleration);
}
} }
