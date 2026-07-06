#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/traci/VehicleController.h"

#include <libsumo/TraCIConstants.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream> // temporary MCM_DEBUG prints
#include <limits>
#include <unordered_map>

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scEmergencyBroadcastDuration;
using scenario::scEmergencyBroadcastInterval;
using scenario::scEmergencyCoordinationTimeGap;
using scenario::scEmergencyMaxSpeed;
using scenario::scEmergencyStartTime;
using scenario::scEmergencyVehicleId;
using scenario::scHighwayLane0MaxY;
using scenario::scHighwayLane0MinY;
using scenario::scHighwayMergingRouteId;
using scenario::scInitialPaperTimeGap;
using scenario::scLaneChangeShiftX;
using scenario::scMaxReceivedMcmCache;
using scenario::scMergeStartX;
using scenario::scMergeStartY;
using scenario::scMergeTargetMaxSnapshotAge;
using scenario::scMergingRouteId;
using scenario::scMergingTimeGap;
using scenario::scNormalHighwaySpeed;
using scenario::scRequestTrajectoryDt;
using scenario::scRequestTrajectorySteps;
using scenario::scSafetyCriticalLaneChangeRouteId;
using scenario::scSafetyCriticalTimeGap;
using scenario::scTargetLaneChangeRouteId;

priorityMcmCategory priorityFromMcm(long priority)
{
    if (priority == static_cast<long>(priorityMcmCategory::LowPriority)) {
        return priorityMcmCategory::LowPriority;
    }
    if (priority == static_cast<long>(priorityMcmCategory::HighPriority)) {
        return priorityMcmCategory::HighPriority;
    }
    if (priority == static_cast<long>(priorityMcmCategory::EmergencyPriority)) {
        return priorityMcmCategory::EmergencyPriority;
    }
    return priorityMcmCategory::MediumPriority;
}

const char* subtypeName(mcmSubtype subtype)
{
    switch (subtype) {
        case mcmSubtype::Request: return "Request";
        case mcmSubtype::Accept: return "Accept";
        case mcmSubtype::Reject: return "Reject";
        case mcmSubtype::Offer: return "Offer";
        case mcmSubtype::Confirm: return "Confirm";
        case mcmSubtype::Execute: return "Execute";
        case mcmSubtype::Cancel: return "Cancel";
        default: return "Negotiation";
    }
}

bool isNegotiationTraceMessage(long subtype)
{
    return subtype == static_cast<long>(mcmSubtype::Request) ||
        subtype == static_cast<long>(mcmSubtype::Offer) ||
        subtype == static_cast<long>(mcmSubtype::Confirm) ||
        subtype == static_cast<long>(mcmSubtype::Accept) ||
        subtype == static_cast<long>(mcmSubtype::Reject);
}

const char* priorityName(long priority)
{
    if (priority == static_cast<long>(priorityMcmCategory::LowPriority)) {
        return "LowPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::MediumPriority)) {
        return "MediumPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::HighPriority)) {
        return "HighPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::EmergencyPriority)) {
        return "EmergencyPriority";
    }
    return "NoPriority";
}

const char* operationModeName(operationMode mode)
{
    switch (mode) {
        case operationMode::IntentionSharingMode: return "IntentionSharingMode";
        case operationMode::ManeuverNegotiationMode: return "ManeuverNegotiationMode";
        case operationMode::ManeuverExecutionMode: return "ManeuverExecutionMode";
        default: return "Unknown";
    }
}

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
    bool legacyPointConflict = false;
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

bool isSafetyCriticalLaneChangeScenarioVehicle(const std::string& vehicleId)
{
    return vehicleId == scEmergencyVehicleId ||
        vehicleId == "car_hl1_1" ||
        vehicleId == "car_hl1_2" ||
        vehicleId == "car_hl1_3" ||
        vehicleId == "car_hl2_1" ||
        vehicleId == "car_hl2_2" ||
        vehicleId == "car_hl2_3" ||
        vehicleId == "car_hl2_4" ||
        vehicleId == "car_hl2_5" ||
        vehicleId == "car_hl2_6";
}

const char* controlManeuverName(controlManeuver maneuver)
{
    switch (maneuver) {
        case controlManeuver::Decelerate: return "Decelerate";
        case controlManeuver::Accelerate: return "Accelerate";
        case controlManeuver::ChangeLane: return "ChangeLane";
        case controlManeuver::LaneChangeExecution: return "LaneChangeExecution";
        case controlManeuver::EmergencyDeceleration: return "EmergencyDeceleration";
        case controlManeuver::DoNothing:
        default:
            return "DoNothing";
    }
}

double costThresholdForPriority(priorityMcmCategory priority)
{
    switch (priority) {
        case priorityMcmCategory::LowPriority: return 0.20;
        case priorityMcmCategory::MediumPriority: return 0.40;
        case priorityMcmCategory::HighPriority: return 0.60;
        case priorityMcmCategory::EmergencyPriority: return 0.80;
        case priorityMcmCategory::NoPriority: return 0.0;
    }

    return 0.0;
}

int priorityLevel(priorityMcmCategory priority)
{
    switch (priority) {
        case priorityMcmCategory::LowPriority: return 0;
        case priorityMcmCategory::MediumPriority: return 1;
        case priorityMcmCategory::HighPriority: return 2;
        case priorityMcmCategory::EmergencyPriority: return 3;
        case priorityMcmCategory::NoPriority: return -1;
    }

    return -1;
}
}

void McApplication::initialize(
    traci::VehicleController* controller,
    const VehicleDataProvider* vehicleDataProvider,
    const LocalEnvironmentModel* localEnvironmentModel)
{
    mVehicleController = controller;
    mVehicleDataProvider = vehicleDataProvider;
    mLocalEnvironmentModel = localEnvironmentModel;
    mTrajectoryPlanner.initialize(controller, vehicleDataProvider, localEnvironmentModel);
}

void McApplication::setNegotiationRetryInterval(omnetpp::SimTime interval)
{
    mNegotiationRetryInterval = interval;
}

void McApplication::setNegotiationLimits(
    omnetpp::SimTime mergingLimit,
    omnetpp::SimTime laneChangeLimit)
{
    mNegotiationLimitMerging = mergingLimit;
    mNegotiationLimitLaneChange = laneChangeLimit;
}

void McApplication::updateEgoContext(const McEgoContext& context)
{
    mEgoContext = context;
    mHasEgoContext = true;
}

void McApplication::tick(omnetpp::SimTime now)
{
    logScenarioVehicleLifetime(now);
    evaluateEmergencyBrakingTrigger(now);
    evaluateMergingRequestTrigger(now);
    evaluateRvRequestRetry(now);
    evaluateRvConfirmRetry(now);
    evaluateCvOfferRetry(now);
    evaluateCvAcceptRetry(now);
    evaluateSafetyCriticalLaneChangeTrigger(now);
    applyRvExecutionControl();
    applySafetyCriticalLaneChangeExecutionControl();
    sampleMergingGapDiagnostics("tick");
    applyCvDecelerationControl();
    applyCvAccelerationControl();
    applyCvLaneChangeControl();
    monitorCvExecutionControl();
    evaluateRvExecutionProgress();
    evaluateCvExecutionProgress();
}

void McApplication::prepareMcmGeneration(omnetpp::SimTime now)
{
    if (mHasEgoContext) {
        mEgoContext.now = now;
    }

    if (mOperationMode == operationMode::ManeuverExecutionMode &&
            mCooperatingVehicleType == cooperatingVehicleType::RV &&
            mCoordinationProgressRV == coordinationProgressRV::SendExecute) {
        queueRepeatedExecute();
    }
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

    evaluateCvRequestResponse(mcm);
    handleReceivedCancelAsCv(mcm);
    handleReceivedOfferAsRv(mcm);
    handleReceivedConfirmAsCv(mcm);
    handleReceivedAcceptAsRv(mcm);
    handleReceivedExecuteEvidenceAsRv(mcm);
    handleReceivedRejectAsRv(mcm);
    handleReceivedExecuteAsCv(mcm);
    handleReceivedEmergencyAsFollower(mcm);
}

void McApplication::handleSentMcm(const SentMcm& mcm)
{
    EV_STATICCONTEXT;

    ++mSentMcmCount;
    mLastSentMcm = mcm;
    mHasLastSentMcm = true;

    logNegotiationTrace("SEND", mcm.data, mcm.sentAt);

    if (mcm.data.hasNegotiationContainer &&
            mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Request) &&
            mCoordinationProgressRV == coordinationProgressRV::CoordinationRequired) {
        mCoordinationProgressRV = coordinationProgressRV::RequestSent;
        if (mEgoContext.routeId == scMergingRouteId) {
            mMergingRequestQueuedOrSent = true;
        } else if (mEgoContext.routeId == scSafetyCriticalLaneChangeRouteId) {
            mLaneChangeRequestQueuedOrSent = true;
            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mcm.sentAt
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
                << " event=request-sent"
                << " requestId=" << mcm.data.requestId
                << " targetCv1=" << mcm.data.negotiationVehicleId1
                << " targetCv2=" << mcm.data.negotiationVehicleId2
                << " priority=HighPriority\n";
        }
    } else if (mcm.data.hasNegotiationContainer &&
            mCooperatingVehicleType == cooperatingVehicleType::RV &&
            mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Cancel) &&
            mCoordinationProgressRV == coordinationProgressRV::SendComplete &&
            mcm.data.requestId == mRvRequestId) {
        logMergingGapSummary(mcm.sentAt);

        if (mEgoContext.routeId == scMergingRouteId && mVehicleController) {
            const std::string& vehicleId = mVehicleController->getVehicleId();
            try {
                // route_merging_1 uses speedMode 0 only for the active
                // right-of-way workaround. After the Complete-as-Cancel has
                // been sent, hand control back to SUMO safety checks so the
                // merged RV does not keep ignoring downstream leaders.
                mVehicleController->setSpeedMode(vehicleId, 31);
                const bool restoreNormalSpeed = canRestoreNormalSpeedFromLeader(scNormalHighwaySpeed);
                if (restoreNormalSpeed) {
                    mVehicleController->setMaxSpeed(scNormalHighwaySpeed * boost::units::si::meter_per_second);
                }
                EV_INFO << "[MCM-MERGE-CONTROL]"
                    << " simTime=" << mcm.sentAt
                    << " event=rv-speedmode-restored-after-completion"
                    << " vehicleId=" << vehicleId
                    << " station=" << mEgoContext.stationId
                    << " requestId=" << static_cast<int>(mRvRequestId)
                    << " speedMode=31"
                    << " normalSpeedRestored=" << restoreNormalSpeed
                    << '\n';
            } catch (const std::exception& e) {
                EV_WARN << "[MCM-MERGE-CONTROL]"
                    << " simTime=" << mcm.sentAt
                    << " event=rv-speedmode-restore-failed"
                    << " vehicleId=" << vehicleId
                    << " station=" << mEgoContext.stationId
                    << " requestId=" << static_cast<int>(mRvRequestId)
                    << " reason=" << e.what()
                    << '\n';
            }
        }

        resetRvCoordinationStateAfterComplete();

        EV_INFO << "McApplication RV station completed coordination workaround"
            << ": requestId=" << static_cast<int>(mRvRequestId)
            << " returned to IntentionSharingMode\n";
    } else if (mcm.data.hasExecutionContainer &&
            mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Abort) &&
            mcm.data.priorityManeuver == static_cast<long>(priorityMcmCategory::EmergencyPriority)) {
        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << mcm.sentAt
            << " role=emergency-vehicle"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " route=" << mEgoContext.routeId
            << " laneIndex=" << mEgoContext.laneIndex
            << " event=sent-emergency-execution-mcm"
            << " subtype=Abort"
            << " priority=EmergencyPriority"
            << " executionContainer=1\n";
    } else if (mcm.data.hasNegotiationContainer &&
            mCooperatingVehicleType == cooperatingVehicleType::CV &&
            mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Cancel) &&
            mCoordinationProgressCV == coordinationProgressCV::SendCompleteCV &&
            mcm.data.requestId == mCvRequestId &&
            mcm.data.negotiationVehicleId1 == mCvRvStationId) {
        restoreCvSpeedControl();

        resetCvCoordinationStateAfterComplete();

        EV_INFO << "McApplication CV station completed coordination workaround"
            << " and returned to IntentionSharingMode\n";

        // std::cout << "MCM_DEBUG CV station " << mEgoContext.stationId
        //     << " returned to IntentionSharingMode after sending Complete workaround"
        //     << " at " << omnetpp::simTime() << " s" << std::endl;

    } else if (mcm.data.hasNegotiationContainer &&
            mCooperatingVehicleType == cooperatingVehicleType::CV &&
            mcm.data.requestId == mCvRequestId &&
            mcm.data.negotiationVehicleId1 == mCvRvStationId &&
            (mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Accept) ||
                mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Reject))) {
        if (mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Accept)) {
            mCoordinationProgressCV = coordinationProgressCV::AcceptSent;
        } else {
            mCoordinationProgressCV = coordinationProgressCV::NoRequest;
            mOperationMode = operationMode::IntentionSharingMode;
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
        mCvResponseQueuedOrSent = true;
    }
}

std::optional<PendingMcmCommand> McApplication::consumePendingCommand()
{
    auto command = mPendingMcmCommand;
    mPendingMcmCommand.reset();
    return command;
}

bool McApplication::hasPendingCoordinationCommand() const
{
    return mPendingMcmCommand.has_value();
}

std::optional<uint8_t> McApplication::consumeCompletedRvNegotiationRequestId()
{
    auto requestId = mCompletedRvNegotiationRequestId;
    mCompletedRvNegotiationRequestId.reset();
    return requestId;
}

void McApplication::logNegotiationTrace(
    const char* action,
    const McmSnapshot& snapshot,
    omnetpp::SimTime time) const
{
    if (!snapshot.hasNegotiationContainer ||
            !isNegotiationTraceMessage(snapshot.mcmCategory)) {
        return;
    }

    const char* role = "NCV";
    switch (mCooperatingVehicleType) {
        case cooperatingVehicleType::RV:
            role = "RV";
            break;
        case cooperatingVehicleType::CV:
            role = "CV";
            break;
        case cooperatingVehicleType::EmergencyV:
            role = "EmergencyV";
            break;
        case cooperatingVehicleType::NCV:
        default:
            role = "NCV";
            break;
    }

    const auto subtype = static_cast<mcmSubtype>(snapshot.mcmCategory);
    const std::string vehicleId = mVehicleController ? mVehicleController->getVehicleId() : "";

    EV_STATICCONTEXT;
    EV_INFO << "[MCM-NEGOTIATION]"
        << " t=" << time
        << " action=" << action
        << " msg=" << subtypeName(subtype)
        << " localVehicleId=" << vehicleId
        << " localStation=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " role=" << role
        << " route=" << (mHasEgoContext ? mEgoContext.routeId : "")
        << " senderStation=" << snapshot.stationId
        << " requestId=" << snapshot.requestId
        << " cooperationId=" << snapshot.cooperationId
        << " priority=" << priorityName(snapshot.priorityManeuver)
        << " numberOfVehicles=" << snapshot.numberOfVehicles
        << " target1=" << snapshot.negotiationVehicleId1
        << " hasTarget2=" << snapshot.hasNegotiationVehicleId2
        << " target2=" << snapshot.negotiationVehicleId2
        << " requestedTrajectoryPoints=" << snapshot.requestedTrajectoryPointCount
        << " offeredTrajectoryPoints=" << snapshot.offeredTrajectoryPointCount
        << '\n';

    std::cout << "[MCM-NEGOTIATION]"
        << " t=" << time
        << " " << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " " << action
        << " " << subtypeName(static_cast<mcmSubtype>(snapshot.mcmCategory))
        << " requestId=" << static_cast<int>(snapshot.requestId)
        << " priority=" << priorityName(snapshot.priorityManeuver)
        << " target1=" << snapshot.negotiationVehicleId1
        << " target2=" << (snapshot.hasNegotiationVehicleId2 ? snapshot.negotiationVehicleId2 : 0)
        << " fromStation=" << snapshot.stationId
        << std::endl;
}

// Shared message guard used by RV and CV handlers. It preserves the old retry
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
// when available; this keeps old cooperation-cost decisions intact and only
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

// RV-side completion reset. This is reached after the legacy Complete-as-Cancel
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
// Cancel rollback reuses the same cleanup but restores the legacy NoRequest
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
    command.requestedTrajectory = mActiveNegotiatedTrajectory;
    return command;
}

// RV-side Confirm retransmission. Confirm uses the fixed negotiated trajectory
// when present, otherwise the current ego trajectory, matching the previous
// retry behavior.
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
    command.requestedTrajectory = mActiveNegotiatedTrajectory.empty()
        ? mEgoContext.plannedTrajectory
        : mActiveNegotiatedTrajectory;
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
    command.requestedTrajectory = mHasActiveNegotiatedTrajectory
        ? mActiveNegotiatedTrajectory
        : (mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {});
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
    command.requestedTrajectory = mHasActiveNegotiatedTrajectory
        ? mActiveNegotiatedTrajectory
        : (mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {});
    return command;
}

// RV timeout reset for unsuccessful medium-priority merging negotiations.
// High-priority lane-change timeouts use the emergency fallback brake instead,
// so this helper preserves the old merging-only reset semantics.
void McApplication::resetRvNegotiationAfterTimeout()
{
    mMergingRequestQueuedOrSent = false;
    mRvOfferReceived1 = false;
    mRvOfferReceived2 = false;
    mRvConfirmQueuedOrSent = false;
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mRvNegotiationCompletionReported = false;
    mCompletedRvNegotiationRequestId.reset();
    mRvNegotiationTimedOut = true;

    mRvLastRequestQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastRequestQueuedAt = false;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasRvNegotiationStartedAt = false;

    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;

    mOperationMode = operationMode::IntentionSharingMode;
    mCoordinationProgressRV = coordinationProgressRV::NoCoordination;
}

// CV timeout reset for Offer/Accept retries. It clears only the negotiation
// bookkeeping that the old timeout paths cleared; execution/control flags are
// left untouched to avoid changing participant state outside negotiation.
void McApplication::resetCvNegotiationAfterTimeout()
{
    mPendingMcmCommand.reset();
    mCoordinationProgressCV = coordinationProgressCV::NoRequest;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;
    mCvResponseQueuedOrSent = false;
    mCvRvStationId = 0;
    mCvRequestId = 0;
    mCvResponseNumberOfVehicles = 1;
    mCvLastOfferQueuedAt = omnetpp::SimTime::ZERO;
    mHasCvLastOfferQueuedAt = false;
    mCvLastAcceptQueuedAt = omnetpp::SimTime::ZERO;
    mHasCvLastAcceptQueuedAt = false;
    mCvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasCvNegotiationStartedAt = false;
    mCvSelectedTrajectory.clear();
    mHasCvSelectedTrajectory = false;
}

void McApplication::clearCommand()
{
    mPendingCommand = CommandKind::None;
    mExecutionState = ExecutionState::Idle;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mRestoreSpeed = 0.0;
    mRestoreMaxSpeed = 0.0;
    mCvDecelerationControlApplied = false;
    mCvDecelerationControlSkippedLogged = false;
    mCvAccelerationControlApplied = false;
    mCvLaneChangeControlLogged = false;
    mCvTargetSpeedReachedLogged = false;
    mCvRestoreNormalSpeedSkippedLogged = false;
    mCvStoppedDecelerationForRvLogged = false;
    mSafetyCriticalLaneChangeExecutionActive = false;
    mLaneChangeMoveStepCounter = 0;
    mSafetyCriticalLaneChangeExecutionStartedAt = omnetpp::SimTime::ZERO;
    mLastSafetyCriticalLaneChangeMoveAt = omnetpp::SimTime::ZERO;
    mPendingMcmCommand.reset();
}

bool McApplication::hasActiveExecution() const
{
    return mExecutionState == ExecutionState::Pending || mExecutionState == ExecutionState::Executing;
}

operationMode McApplication::currentOperationMode() const
{
    return mOperationMode;
}

void McApplication::applyCommand()
{
    // TODO: later use VehicleController hooks for DecelerateTo and RestoreNormalSpeed commands.
}

void McApplication::applyRvExecutionControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            mEgoContext.routeId != scMergingRouteId) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();

    // Old route_merging_1 RV behavior uses speedMode 0 only for the merging RV.
    // This prevents SUMO right-of-way logic from stopping/decelerating the RV at
    // the merge. Later ordinary speed-control maneuvers, such as CV
    // acceleration/deceleration, should use speedMode 31 so SUMO safety checks
    // remain active and speed changes stay realistic.
    mVehicleController->setSpeedMode(vehicleId, 0);
    mVehicleController->setMaxSpeed(22.22 * boost::units::si::meter_per_second);
    mVehicleController->setSpeed(22.22 * boost::units::si::meter_per_second);

    if (!mRvMergingExecutionControlLogged) {
        EV_INFO << "McApplication applied route_merging_1 RV execution control"
            << ": station=" << mEgoContext.stationId
            << " vehicleId=" << vehicleId
            << " speedMode=0 targetSpeed=22.22 maxSpeed=22.22\n";
        mRvMergingExecutionControlLogged = true;
    }
}

bool McApplication::canRestoreNormalSpeedFromLeader(double desiredSpeed)
{
    if (!mHasEgoContext || !mVehicleController) {
        return false;
    }

    FrontVehicleInfo frontInfo;
    try {
        frontInfo = mTrajectoryPlanner.getFrontVehicleInfo();
    } catch (const std::exception&) {
        return false;
    }

    if (!frontInfo.sameEdgeLane) {
        return true;
    }

    const double timeGap = calculateTimeGap(frontInfo.distance, std::max(0.1, desiredSpeed));
    const double ttc = calculateTTC(desiredSpeed, frontInfo.speed, frontInfo.distance);

    // Speed hand-back is intentionally conservative: after RV/CV execution,
    // raising max speed is allowed only when the existing environment-model
    // leader is faster or far enough away. This avoids unsafe recovery surges.
    return frontInfo.speed >= desiredSpeed ||
        (frontInfo.distance > 10.0 && timeGap >= scSafetyCriticalTimeGap && ttc >= 2.0);
}

void McApplication::restoreNormalSpeedIfSafe(const char* role, uint8_t requestId)
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    FrontVehicleInfo frontInfo;
    try {
        frontInfo = mTrajectoryPlanner.getFrontVehicleInfo();
    } catch (const std::exception&) {
        frontInfo = {};
        frontInfo.distance = 999.0;
        frontInfo.speed = 99.0;
    }

    const double desiredSpeed = scNormalHighwaySpeed;
    const double timeGap = frontInfo.sameEdgeLane ?
        calculateTimeGap(frontInfo.distance, std::max(0.1, mEgoContext.speed)) :
        std::numeric_limits<double>::infinity();
    const double ttc = frontInfo.sameEdgeLane ?
        calculateTTC(mEgoContext.speed, frontInfo.speed, frontInfo.distance) :
        std::numeric_limits<double>::infinity();

    // Normal-speed restoration is shared by RV and CV execution completion.
    // It is deliberately gated by the leader check instead of blindly restoring
    // 27.77 m/s, because the emergency lane-change case can leave a close
    // front vehicle in the target lane.
    if (!canRestoreNormalSpeedFromLeader(desiredSpeed)) {
        EV_INFO << "[MCM-CV-CONTROL]"
            << " simTime=" << mEgoContext.now
            << " event=restore-normal-speed-skipped"
            << " role=" << role
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(requestId)
            << " currentSpeed=" << mEgoContext.speed
            << " targetSpeed=" << desiredSpeed
            << " currentLane=" << mEgoContext.laneIndex
            << " currentX=" << mEgoContext.x
            << " currentY=" << mEgoContext.y
            << " frontVehicleId=" << frontInfo.frontVehicleId
            << " frontDistance=" << frontInfo.distance
            << " frontSpeed=" << frontInfo.speed
            << " timeGap=" << timeGap
            << " ttc=" << ttc
            << " reason=leader-condition-not-safe\n";
        return;
    }

    try {
        mVehicleController->setSpeedMode(vehicleId, 31);
        mVehicleController->setMaxSpeed(desiredSpeed * boost::units::si::meter_per_second);
        if (mCvAccelerationControlApplied) {
            mVehicleController->setSpeed(desiredSpeed * boost::units::si::meter_per_second);
        }
        EV_INFO << "[MCM-CV-CONTROL]"
            << " simTime=" << mEgoContext.now
            << " event=restore-normal-speed"
            << " role=" << role
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(requestId)
            << " currentSpeed=" << mEgoContext.speed
            << " targetSpeed=" << desiredSpeed
            << " currentLane=" << mEgoContext.laneIndex
            << " currentX=" << mEgoContext.x
            << " currentY=" << mEgoContext.y
            << " frontVehicleId=" << frontInfo.frontVehicleId
            << " frontDistance=" << frontInfo.distance
            << " frontSpeed=" << frontInfo.speed
            << " timeGap=" << timeGap
            << " ttc=" << ttc
            << " reason=leader-condition-safe\n";
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-CV-CONTROL]"
            << " simTime=" << mEgoContext.now
            << " event=restore-normal-speed-skipped"
            << " role=" << role
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(requestId)
            << " reason=traci-exception"
            << " error=\"" << e.what() << "\"\n";
    }
}

void McApplication::applySafetyCriticalLaneChangeExecutionControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mPriorityMcmCategory != priorityMcmCategory::HighPriority ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            mEgoContext.routeId != scSafetyCriticalLaneChangeRouteId ||
            (mControlManeuver != controlManeuver::ChangeLane &&
                mControlManeuver != controlManeuver::LaneChangeExecution)) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    int laneIndex = mEgoContext.laneIndex;
    double currentX = mEgoContext.x;
    double currentY = mEgoContext.y;
    double currentSpeed = mEgoContext.speed;

    try {
        laneIndex = mVehicleController->getTraCI()->vehicle.getLaneIndex(vehicleId);
        const auto position = mVehicleController->getPositionSumo();
        currentX = position.x / boost::units::si::meter;
        currentY = position.y / boost::units::si::meter;
        currentSpeed = mVehicleController->getTraCI()->vehicle.getSpeed(vehicleId);
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-LC-EXEC]"
            << " simTime=" << mEgoContext.now
            << " event=moveToXY-failed"
            << " role=RV"
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " reason=traci-state-read-failed"
            << " error=\"" << e.what() << "\"\n";
        applyEmergencyFallbackBrake("moveToXY-failed-brake", "traci-state-read-failed", mRvRequestId);
        return;
    }

    if (!mSafetyCriticalLaneChangeExecutionActive) {
        mSafetyCriticalLaneChangeExecutionActive = true;
        mLaneChangeMoveStepCounter = 0;
        mSafetyCriticalLaneChangeExecutionStartedAt = mEgoContext.now;
        mLastSafetyCriticalLaneChangeMoveAt = omnetpp::SimTime::ZERO;
        mControlManeuver = controlManeuver::LaneChangeExecution;

        EV_INFO << "[MCM-LC-EXEC]"
            << " simTime=" << mEgoContext.now
            << " event=lane-change-execution-start"
            << " role=RV"
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " currentSpeed=" << currentSpeed
            << " currentLane=" << laneIndex
            << " currentX=" << currentX
            << " currentY=" << currentY
            << " stepCounter=" << mLaneChangeMoveStepCounter
            << '\n';
    }

    // The RV keeps monitoring the environment-model front vehicle while moving
    // laterally. If the target lane becomes unsafe, execution falls back to
    // braking instead of continuing the moveToXY sequence.
    FrontVehicleInfo frontInfo;
    try {
        frontInfo = mTrajectoryPlanner.getFrontVehicleInfo();
    } catch (const std::exception&) {
        frontInfo = {};
        frontInfo.distance = 999.0;
        frontInfo.speed = 99.0;
    }

    const double timeGap = frontInfo.sameEdgeLane ?
        calculateTimeGap(frontInfo.distance, std::max(0.1, currentSpeed)) :
        std::numeric_limits<double>::infinity();
    const double ttc = frontInfo.sameEdgeLane ?
        calculateTTC(currentSpeed, frontInfo.speed, frontInfo.distance) :
        std::numeric_limits<double>::infinity();

    EV_INFO << "[MCM-LC-EXEC]"
        << " simTime=" << mEgoContext.now
        << " event=execution-monitor"
        << " role=RV"
        << " vehicleId=" << vehicleId
        << " station=" << mEgoContext.stationId
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " currentSpeed=" << currentSpeed
        << " currentLane=" << laneIndex
        << " currentX=" << currentX
        << " currentY=" << currentY
        << " stepCounter=" << mLaneChangeMoveStepCounter
        << " frontVehicleId=" << frontInfo.frontVehicleId
        << " frontDistance=" << frontInfo.distance
        << " frontSpeed=" << frontInfo.speed
        << " timeGap=" << timeGap
        << " ttc=" << ttc
        << '\n';

    const bool unsafeFrontVehicle =
        frontInfo.sameEdgeLane &&
        (frontInfo.distance < 5.0 || timeGap < 0.3 || ttc < 1.0);
    if (unsafeFrontVehicle) {
        EV_WARN << "[MCM-LC-FAILSAFE]"
            << " simTime=" << mEgoContext.now
            << " event=unsafe-front-vehicle-brake"
            << " role=RV"
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " currentSpeed=" << currentSpeed
            << " currentLane=" << laneIndex
            << " currentX=" << currentX
            << " currentY=" << currentY
            << " frontVehicleId=" << frontInfo.frontVehicleId
            << " frontDistance=" << frontInfo.distance
            << " frontSpeed=" << frontInfo.speed
            << " timeGap=" << timeGap
            << " ttc=" << ttc
            << " reason=unsafe-front-vehicle\n";
        applyEmergencyFallbackBrake("unsafe-front-vehicle-brake", "unsafe-front-vehicle", mRvRequestId);
        return;
    }

    if (mLaneChangeMoveStepCounter >= 10) {
        if (mSafetyCriticalLaneChangeExecutionActive) {
            EV_INFO << "[MCM-LC-EXEC]"
                << " simTime=" << mEgoContext.now
                << " event=lane-change-execution-complete"
                << " role=RV"
                << " vehicleId=" << vehicleId
                << " station=" << mEgoContext.stationId
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " currentSpeed=" << currentSpeed
                << " currentLane=" << laneIndex
                << " currentX=" << currentX
                << " currentY=" << currentY
                << " stepCounter=" << mLaneChangeMoveStepCounter
                << '\n';
        }
        mSafetyCriticalLaneChangeExecutionActive = false;
        mControlManeuver = controlManeuver::DoNothing;
        restoreNormalSpeedIfSafe("RV", mRvRequestId);
        return;
    }

    // Old safety-critical lane-change execution is a scenario-specific
    // positive-X shift: 3.2 m lane width over 10 ticks, so 0.32 m each step.
    // The longitudinal step follows the old y -= speed / 10 pattern.
    const double targetX = currentX + 0.32;
    const double targetY = currentY - currentSpeed / 10.0;

    try {
        // Use SUMO/libsumo's invalid-angle sentinel. A NaN angle produced
        // invalid environment-model polygons during execution validation.
        mVehicleController->moveToXY(
            vehicleId,
            "-1",
            laneIndex,
            targetX,
            targetY,
            libsumo::INVALID_DOUBLE_VALUE,
            3);
        ++mLaneChangeMoveStepCounter;
        mLastSafetyCriticalLaneChangeMoveAt = mEgoContext.now;

        EV_INFO << "[MCM-LC-EXEC]"
            << " simTime=" << mEgoContext.now
            << " event=moveToXY-step"
            << " role=RV"
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " currentSpeed=" << currentSpeed
            << " currentLane=" << laneIndex
            << " currentX=" << currentX
            << " currentY=" << currentY
            << " targetX=" << targetX
            << " targetY=" << targetY
            << " stepCounter=" << mLaneChangeMoveStepCounter
            << " frontVehicleId=" << frontInfo.frontVehicleId
            << " frontDistance=" << frontInfo.distance
            << " frontSpeed=" << frontInfo.speed
            << " timeGap=" << timeGap
            << " ttc=" << ttc
            << '\n';
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-LC-EXEC]"
            << " simTime=" << mEgoContext.now
            << " event=moveToXY-failed"
            << " role=RV"
            << " vehicleId=" << vehicleId
            << " station=" << mEgoContext.stationId
            << " requestId=" << static_cast<int>(mRvRequestId)
            << " currentSpeed=" << currentSpeed
            << " currentLane=" << laneIndex
            << " currentX=" << currentX
            << " currentY=" << currentY
            << " targetX=" << targetX
            << " targetY=" << targetY
            << " stepCounter=" << mLaneChangeMoveStepCounter
            << " reason=moveToXY-exception"
            << " error=\"" << e.what() << "\"\n";
        applyEmergencyFallbackBrake("moveToXY-failed-brake", "moveToXY-exception", mRvRequestId);
    }
}

void McApplication::applyCvDecelerationControl()
{
    EV_STATICCONTEXT;

    const bool oldEquivalentAcceptPhase =
        mOperationMode == operationMode::ManeuverNegotiationMode &&
        mCoordinationProgressCV == coordinationProgressCV::SendAccept;
    const bool executionPhase =
        mOperationMode == operationMode::ManeuverExecutionMode &&
        mCoordinationProgressCV == coordinationProgressCV::SendExecuteCV;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            (!oldEquivalentAcceptPhase && !executionPhase) ||
            mControlManeuver != controlManeuver::Decelerate ||
            mCvDecelerationControlApplied) {
        return;
    }

    if (mTargetSpeed <= 0.0 || mCommandDuration <= 0.0 ||
            !std::isfinite(mTargetSpeed) || !std::isfinite(mCommandDuration)) {
        if (!mCvDecelerationControlSkippedLogged) {
            EV_WARN << "McApplication skipped CV deceleration control"
                << ": station=" << mEgoContext.stationId
                << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
                << " targetSpeed=" << mTargetSpeed
                << " decelerationTime=" << mCommandDuration
                << '\n';
            mCvDecelerationControlSkippedLogged = true;
        }
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();

    // The old CV deceleration path initially used speedMode 0. For this narrow
    // execution milestone we keep SUMO safety checks enabled with speedMode 31;
    // speedMode 0 remains reserved for the route_merging_1 RV right-of-way case.
    mVehicleController->setSpeedMode(vehicleId, 31);
    mVehicleController->slowDown(vehicleId, mTargetSpeed, mCommandDuration);
    mCvDecelerationControlApplied = true;
    mCvTargetSpeedReachedLogged = false;
    mCvRestoreNormalSpeedSkippedLogged = false;
    mCvStoppedDecelerationForRvLogged = false;

    EV_INFO << "McApplication applied highway-merging CV deceleration control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << " speedMode=31 targetSpeed=" << mTargetSpeed
        << " decelerationTime=" << mCommandDuration
        << '\n';
    EV_INFO << "[MCM-CV-CONTROL]"
        << " simTime=" << mEgoContext.now
        << " event=apply-deceleration"
        << " cvVehicleId=" << vehicleId
        << " cvStation=" << mEgoContext.stationId
        << " rvStation=" << mCvRvStationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " priority=" << priorityName(static_cast<long>(mPriorityMcmCategory))
        << " selectedAction=Decelerate"
        << " targetSpeed=" << mTargetSpeed
        << " decelerationTime=" << mCommandDuration
        << " speedMode=31"
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << '\n';
}

void McApplication::applyCvAccelerationControl()
{
    EV_STATICCONTEXT;

    const bool oldEquivalentAcceptPhase =
        mOperationMode == operationMode::ManeuverNegotiationMode &&
        mCoordinationProgressCV == coordinationProgressCV::SendAccept;
    const bool executionPhase =
        mOperationMode == operationMode::ManeuverExecutionMode &&
        mCoordinationProgressCV == coordinationProgressCV::SendExecuteCV;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            (!oldEquivalentAcceptPhase && !executionPhase) ||
            mControlManeuver != controlManeuver::Accelerate ||
            mCvAccelerationControlApplied) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    const bool highPriorityLaneChange =
        mPriorityMcmCategory == priorityMcmCategory::HighPriority &&
        mEgoContext.routeId == scTargetLaneChangeRouteId;
    const double targetSpeed = highPriorityLaneChange && mTargetSpeed > mEgoContext.speed ?
        mTargetSpeed : 33.33;

    // Old highway CV acceleration uses speedMode 31 so SUMO safety checks stay
    // active while raising the speed toward 120 km/h.
    mVehicleController->setSpeedMode(vehicleId, 31);
    mVehicleController->setSpeed(targetSpeed * boost::units::si::meter_per_second);
    mVehicleController->setMaxSpeed(targetSpeed * boost::units::si::meter_per_second);
    mCvAccelerationControlApplied = true;
    mCvRestoreNormalSpeedSkippedLogged = false;

    EV_INFO << "McApplication applied highway-merging CV acceleration control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << " speedMode=31 targetSpeed=" << targetSpeed
        << " maxSpeed=" << targetSpeed << '\n';
    EV_INFO << "[MCM-CV-CONTROL]"
        << " simTime=" << mEgoContext.now
        << " event=apply-acceleration"
        << " cvVehicleId=" << vehicleId
        << " cvStation=" << mEgoContext.stationId
        << " rvStation=" << mCvRvStationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " priority=" << priorityName(static_cast<long>(mPriorityMcmCategory))
        << " selectedAction=Accelerate"
        << " targetSpeed=" << targetSpeed
        << " maxSpeed=" << targetSpeed
        << " speedMode=31"
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << '\n';
}

void McApplication::applyCvLaneChangeControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressCV != coordinationProgressCV::SendExecuteCV ||
            mControlManeuver != controlManeuver::ChangeLane ||
            mCvLaneChangeControlLogged) {
        return;
    }

    // Legacy lateral control transitions ChangeLane to LaneChangeExecution and
    // uses a 10-step moveToXY loop based on live TraCI lane/position/speed.
    // Keep this milestone state-only until the lane target and step counter are
    // represented explicitly in McApplication.
    EV_INFO << "McApplication CV lane-change control not applied yet"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " laneIndex=" << mEgoContext.laneIndex
        << " selectedTrajectoryPoints="
        << (mHasCvSelectedTrajectory ? mCvSelectedTrajectory.size() : 0)
        << " missing=lateral-target-and-step-counter\n";
    EV_INFO << "[MCM-CV-CONTROL]"
        << " simTime=" << mEgoContext.now
        << " event=apply-lane-change"
        << " cvVehicleId=" << mVehicleController->getVehicleId()
        << " cvStation=" << mEgoContext.stationId
        << " rvStation=" << mCvRvStationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " priority=" << priorityName(static_cast<long>(mPriorityMcmCategory))
        << " selectedAction=ChangeLane"
        << " selectedTrajectoryPoints=" << (mHasCvSelectedTrajectory ? mCvSelectedTrajectory.size() : 0)
        << " applied=false"
        << " reason=lateral-target-and-step-counter-not-implemented"
        << '\n';
    mCvLaneChangeControlLogged = true;
}

void McApplication::monitorCvExecutionControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressCV != coordinationProgressCV::SendExecuteCV) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    int laneIndex = mEgoContext.laneIndex;
    double currentX = mEgoContext.x;
    double currentY = mEgoContext.y;
    double currentSpeed = mEgoContext.speed;

    try {
        laneIndex = mVehicleController->getTraCI()->vehicle.getLaneIndex(vehicleId);
        const auto position = mVehicleController->getPositionSumo();
        currentX = position.x / boost::units::si::meter;
        currentY = position.y / boost::units::si::meter;
        currentSpeed = mVehicleController->getTraCI()->vehicle.getSpeed(vehicleId);
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-CV-CONTROL]"
            << " simTime=" << mEgoContext.now
            << " event=execution-monitor"
            << " cvVehicleId=" << vehicleId
            << " cvStation=" << mEgoContext.stationId
            << " rvStation=" << mCvRvStationId
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " selectedAction=" << controlManeuverName(mControlManeuver)
            << " reason=traci-state-read-failed"
            << " error=\"" << e.what() << "\"\n";
        return;
    }

    FrontVehicleInfo frontInfo;
    try {
        frontInfo = mTrajectoryPlanner.getFrontVehicleInfo();
    } catch (const std::exception&) {
        frontInfo = {};
        frontInfo.distance = 999.0;
        frontInfo.speed = 99.0;
    }

    const double frontTimeGap = frontInfo.sameEdgeLane ?
        calculateTimeGap(frontInfo.distance, std::max(0.1, currentSpeed)) :
        std::numeric_limits<double>::infinity();
    const double frontTtc = frontInfo.sameEdgeLane ?
        calculateTTC(currentSpeed, frontInfo.speed, frontInfo.distance) :
        std::numeric_limits<double>::infinity();

    // During execution the CV monitors both its own environment-model leader
    // and the latest RV MCM. This keeps accepted acceleration/deceleration
    // bounded after the message handshake has completed.
    const McmSnapshot* rvSnapshot = nullptr;
    for (auto it = mReceivedMcmCache.rbegin(); it != mReceivedMcmCache.rend(); ++it) {
        if (it->data.stationId == mCvRvStationId && !it->data.plannedTrajectory.empty()) {
            rvSnapshot = &it->data;
            break;
        }
    }

    int rvLane = -1;
    double rvX = 0.0;
    double rvY = 0.0;
    double rvSpeed = -1.0;
    double rvDistance = -1.0;
    double rvTimeGap = -1.0;
    bool rvSameLane = false;
    bool rvAhead = false;
    bool laneWorkaroundApplied = false;

    if (rvSnapshot) {
        const auto& rvPoint = rvSnapshot->plannedTrajectory.front();
        rvX = rvPoint.mX;
        rvY = rvPoint.mY;
        rvLane = rvSnapshot->hasLaneId ? static_cast<int>(rvSnapshot->laneId) : -1;
        if (rvPoint.mY > 452365.0 && rvLane >= 0) {
            ++rvLane;
            laneWorkaroundApplied = true;
        }
        rvSpeed = rvSnapshot->speedValue >= 0 && rvSnapshot->speedValue < 16383 ?
            static_cast<double>(rvSnapshot->speedValue) / 100.0 : -1.0;
        rvDistance = getDistance(currentX, currentY, rvX, rvY);
        rvTimeGap = currentSpeed > 0.1 ? rvDistance / currentSpeed : -1.0;
        rvSameLane = rvLane == laneIndex;
        rvAhead = currentY >= rvY;
    }

    EV_INFO << "[MCM-CV-CONTROL]"
        << " simTime=" << mEgoContext.now
        << " event=execution-monitor"
        << " role=CV"
        << " cvVehicleId=" << vehicleId
        << " cvStation=" << mEgoContext.stationId
        << " rvStation=" << mCvRvStationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " priority=" << priorityName(static_cast<long>(mPriorityMcmCategory))
        << " selectedAction=" << controlManeuverName(mControlManeuver)
        << " currentSpeed=" << currentSpeed
        << " targetSpeed=" << mTargetSpeed
        << " currentLane=" << laneIndex
        << " currentX=" << currentX
        << " currentY=" << currentY
        << " frontVehicleId=" << frontInfo.frontVehicleId
        << " frontDistance=" << frontInfo.distance
        << " frontSpeed=" << frontInfo.speed
        << " timeGap=" << frontTimeGap
        << " ttc=" << frontTtc
        << " rvSeen=" << (rvSnapshot != nullptr)
        << " rvLane=" << rvLane
        << " rvLaneWorkaroundApplied=" << laneWorkaroundApplied
        << " rvX=" << rvX
        << " rvY=" << rvY
        << " rvSpeed=" << rvSpeed
        << " rvDistance=" << rvDistance
        << " rvTimeGap=" << rvTimeGap
        << " rvSameLane=" << rvSameLane
        << " rvAhead=" << rvAhead
        << '\n';

    if (mControlManeuver == controlManeuver::Decelerate && mTargetSpeed > 0.0) {
        if (currentSpeed <= mTargetSpeed + 1.0) {
            try {
                mVehicleController->setMaxSpeed(mTargetSpeed * boost::units::si::meter_per_second);
            } catch (const std::exception& e) {
                EV_WARN << "[MCM-CV-CONTROL]"
                    << " simTime=" << mEgoContext.now
                    << " event=target-speed-reached"
                    << " cvVehicleId=" << vehicleId
                    << " cvStation=" << mEgoContext.stationId
                    << " requestId=" << static_cast<int>(mCvRequestId)
                    << " targetSpeed=" << mTargetSpeed
                    << " applied=false"
                    << " reason=traci-exception"
                    << " error=\"" << e.what() << "\"\n";
            }

            if (!mCvTargetSpeedReachedLogged) {
                EV_INFO << "[MCM-CV-CONTROL]"
                    << " simTime=" << mEgoContext.now
                    << " event=target-speed-reached"
                    << " cvVehicleId=" << vehicleId
                    << " cvStation=" << mEgoContext.stationId
                    << " rvStation=" << mCvRvStationId
                    << " requestId=" << static_cast<int>(mCvRequestId)
                    << " currentSpeed=" << currentSpeed
                    << " targetSpeed=" << mTargetSpeed
                    << " currentLane=" << laneIndex
                    << " frontVehicleId=" << frontInfo.frontVehicleId
                    << " frontDistance=" << frontInfo.distance
                    << " frontSpeed=" << frontInfo.speed
                    << " timeGap=" << frontTimeGap
                    << " ttc=" << frontTtc
                    << '\n';
                mCvTargetSpeedReachedLogged = true;
            }

            if (frontInfo.sameEdgeLane && frontInfo.speed > currentSpeed &&
                    canRestoreNormalSpeedFromLeader(scNormalHighwaySpeed)) {
                restoreNormalSpeedIfSafe("CV", mCvRequestId);
            } else if (!mCvRestoreNormalSpeedSkippedLogged) {
                EV_INFO << "[MCM-CV-CONTROL]"
                    << " simTime=" << mEgoContext.now
                    << " event=restore-normal-speed-skipped"
                    << " role=CV"
                    << " cvVehicleId=" << vehicleId
                    << " cvStation=" << mEgoContext.stationId
                    << " rvStation=" << mCvRvStationId
                    << " requestId=" << static_cast<int>(mCvRequestId)
                    << " currentSpeed=" << currentSpeed
                    << " targetSpeed=" << scNormalHighwaySpeed
                    << " currentLane=" << laneIndex
                    << " frontVehicleId=" << frontInfo.frontVehicleId
                    << " frontDistance=" << frontInfo.distance
                    << " frontSpeed=" << frontInfo.speed
                    << " timeGap=" << frontTimeGap
                    << " ttc=" << frontTtc
                    << " reason=leader-condition-not-safe-or-not-faster\n";
                mCvRestoreNormalSpeedSkippedLogged = true;
            }
        }

        if (rvSnapshot && rvSameLane && rvAhead && rvSpeed > currentSpeed &&
                !mCvStoppedDecelerationForRvLogged) {
            // Once the RV is in front and faster, the old behavior stops
            // unnecessary CV deceleration, but only if the CV's own leader
            // constraints allow a safe return toward normal speed.
            if (canRestoreNormalSpeedFromLeader(scNormalHighwaySpeed)) {
                try {
                    mVehicleController->setSpeedMode(vehicleId, 31);
                    mVehicleController->slowDown(vehicleId, currentSpeed, 0.1);
                    mVehicleController->setSpeed(scNormalHighwaySpeed * boost::units::si::meter_per_second);
                    mVehicleController->setMaxSpeed(scNormalHighwaySpeed * boost::units::si::meter_per_second);
                    mControlManeuver = controlManeuver::DoNothing;
                    mCvStoppedDecelerationForRvLogged = true;

                    EV_INFO << "[MCM-CV-CONTROL]"
                        << " simTime=" << mEgoContext.now
                        << " event=stop-deceleration-rv-ahead-faster"
                        << " cvVehicleId=" << vehicleId
                        << " cvStation=" << mEgoContext.stationId
                        << " rvStation=" << mCvRvStationId
                        << " requestId=" << static_cast<int>(mCvRequestId)
                        << " currentSpeed=" << currentSpeed
                        << " targetSpeed=" << scNormalHighwaySpeed
                        << " currentLane=" << laneIndex
                        << " rvLane=" << rvLane
                        << " rvSpeed=" << rvSpeed
                        << " rvDistance=" << rvDistance
                        << " rvTimeGap=" << rvTimeGap
                        << " frontVehicleId=" << frontInfo.frontVehicleId
                        << " frontDistance=" << frontInfo.distance
                        << " frontSpeed=" << frontInfo.speed
                        << " reason=rv-ahead-faster-and-leader-safe\n";
                } catch (const std::exception& e) {
                    EV_WARN << "[MCM-CV-CONTROL]"
                        << " simTime=" << mEgoContext.now
                        << " event=restore-normal-speed-skipped"
                        << " role=CV"
                        << " cvVehicleId=" << vehicleId
                        << " cvStation=" << mEgoContext.stationId
                        << " requestId=" << static_cast<int>(mCvRequestId)
                        << " reason=traci-exception"
                        << " error=\"" << e.what() << "\"\n";
                }
            } else if (!mCvRestoreNormalSpeedSkippedLogged) {
                EV_INFO << "[MCM-CV-CONTROL]"
                    << " simTime=" << mEgoContext.now
                    << " event=restore-normal-speed-skipped"
                    << " role=CV"
                    << " cvVehicleId=" << vehicleId
                    << " cvStation=" << mEgoContext.stationId
                    << " rvStation=" << mCvRvStationId
                    << " requestId=" << static_cast<int>(mCvRequestId)
                    << " currentSpeed=" << currentSpeed
                    << " targetSpeed=" << scNormalHighwaySpeed
                    << " currentLane=" << laneIndex
                    << " rvLane=" << rvLane
                    << " rvSpeed=" << rvSpeed
                    << " rvDistance=" << rvDistance
                    << " frontVehicleId=" << frontInfo.frontVehicleId
                    << " frontDistance=" << frontInfo.distance
                    << " frontSpeed=" << frontInfo.speed
                    << " timeGap=" << frontTimeGap
                    << " ttc=" << frontTtc
                    << " reason=rv-ahead-faster-but-leader-not-safe\n";
                mCvRestoreNormalSpeedSkippedLogged = true;
            }
        }
    }
}

void McApplication::restoreCvSpeedControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            (!mCvDecelerationControlApplied && !mCvAccelerationControlApplied)) {
        return;
    }

    const bool restoredAfterAcceleration = mCvAccelerationControlApplied;
    const bool restoredAfterDeceleration = mCvDecelerationControlApplied;

    restoreNormalSpeedIfSafe("CV", mCvRequestId);

    EV_INFO << "McApplication restored highway-merging CV speed control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " speedMode=31 normalSpeed=27.77"
        << " restoredAfterDeceleration=" << restoredAfterDeceleration
        << " restoredAfterAcceleration=" << restoredAfterAcceleration
        << '\n';
}

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
    // Old safety-critical lane-change CV branch keeps the CV lane-change
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

    const bool foundSuitableTrajectory = std::get<0>(result);
    if (!foundSuitableTrajectory) {
        EV_INFO << "McApplication CV station " << mEgoContext.stationId
            << " classified merging control maneuver: DoNothing"
            << " because findSuitableTrajectoryCV found no safe trajectory"
            << " oldBranch=" << (decelerationRequired ? "decelerate-or-change-lane" : "accelerate-or-change-lane")
            << " requestId=" << snapshot.requestId
            << " priority=" << priority
            << '\n';
        return;
    }

    const auto& selectedTrajectory = std::get<1>(result);
    const PlannedTrajValues& plannedValues = std::get<2>(result);
    const double trajectoryCost = std::get<3>(result);
    const int trajectoryType = std::get<4>(result);
    const int possiblePriorityLevel = std::get<5>(result);

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
        << " oldBranch=" << (decelerationRequired ? "decelerate-or-change-lane" : "accelerate-or-change-lane")
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

void McApplication::evaluateMergingRequestTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    // RV-side medium-priority merging validation.
    // route_merging_1 is a scenario route, not a protocol condition. Once the
    // RV reaches the old trigger point, it chooses current idle highway CVs by
    // trajectory/gap feasibility around the predicted merge point. The old
    // pointwise conflict check is retained as supporting diagnostics; it is not
    // a fixed vehicle-ID mapping.
    if (!mHasEgoContext || !mVehicleDataProvider || !mVehicleController ||
            mPendingMcmCommand || mMergingRequestQueuedOrSent ||
            mRvNegotiationTimedOut) {
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
    unsigned legacyPointConflicts = 0;
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

        const bool legacyPointConflict = mTrajectoryPlanner.check_traj_conflict_merging(
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
        candidate.legacyPointConflict = legacyPointConflict;

        const double mergeGapWindow =
            std::max(desiredDistanceGap, scMergingTimeGap * std::max(mEgoContext.speed, 1.0) * 2.5);
        if (candidate.absLongitudinalGap > mergeGapWindow) {
            continue;
        }

        candidates.push_back(candidate);
        if (legacyPointConflict) {
            ++legacyPointConflicts;
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
            << " legacyPointConflict=" << candidate.legacyPointConflict
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
        << " legacyPointConflicts=" << legacyPointConflicts
        << " target1=" << target1
        << " target2=" << target2
        << '\n';

    EV_INFO << "McApplication route_merging_1 considered " << considered
        << " latest planned trajectories; gapCandidates=" << candidates.size()
        << " legacyPointConflicts=" << legacyPointConflicts << '\n';

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
    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;
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

void McApplication::evaluateRvRequestRetry(omnetpp::SimTime now)
{
    // RV-side Request retry while waiting for Offer messages. Two-CV
    // negotiations require both Offers; one-CV high-priority lane-change
    // requests may complete directly with Accept, so that legacy path is kept.
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
                mRvRequestId);
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
                mRvRequestId);
            return;
        }

        resetRvNegotiationAfterTimeout();
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
    // RV-side Confirm retry while waiting for Accept messages. The old flow
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
                mRvRequestId);
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

void McApplication::evaluateEmergencyBrakingTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    // Scenario-only emergency source for the high-priority lane-change
    // validation. car_hl0_Emergency reproduces the old paper scenario by
    // braking at a configured time and broadcasting EmergencyPriority execution
    // MCMs for 15 s at 10 Hz. This is not generic protocol behavior.
    if (!mVehicleController || !mHasEgoContext ||
            mVehicleController->getVehicleId() != scEmergencyVehicleId) {
        return;
    }

    if (mEgoContext.routeId == scSafetyCriticalLaneChangeRouteId &&
            mEgoContext.speed < scNormalHighwaySpeed - 0.5 &&
            now.dbl() < scEmergencyStartTime) {
        mVehicleController->setMaxSpeed(scNormalHighwaySpeed * boost::units::si::meter_per_second);
    }

    if (now.dbl() < scEmergencyStartTime) {
        if (!mEmergencyTriggerWaitingLogged && now.dbl() >= scEmergencyStartTime - 0.25) {
            EV_INFO << "[MCM-EMERGENCY]"
                << " simTime=" << now
                << " role=emergency-vehicle"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " laneIndex=" << mEgoContext.laneIndex
                << " event=trigger-skipped"
                << " reason=before-scheduled-start"
                << " scheduledStartTime=" << scEmergencyStartTime
                << " timeUntilStart=" << (scEmergencyStartTime - now.dbl())
                << '\n';
            mEmergencyTriggerWaitingLogged = true;
        }
        return;
    }

    const double emergencyElapsed = now.dbl() - scEmergencyStartTime;
    const bool withinBroadcastWindow =
        emergencyElapsed >= 0.0 && emergencyElapsed < scEmergencyBroadcastDuration;

    // Old emergency signaling is represented as an execution-container
    // Abort with EmergencyPriority. Sending every 0.1 s for 15 s gives the
    // expected 10 Hz broadcast window, approximately 150 emergency MCMs.
    if (!mEmergencyBroadcastStarted && withinBroadcastWindow) {
        mEmergencyBroadcastStarted = true;
        mEmergencyBroadcastStartedAt = now;
        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << now
            << " role=emergency-vehicle"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " laneIndex=" << mEgoContext.laneIndex
            << " event=emergency-broadcast-start"
            << " duration=" << scEmergencyBroadcastDuration
            << " sendInterval=" << scEmergencyBroadcastInterval
            << " speed=" << mEgoContext.speed
            << '\n';
    }

    if (!mEmergencyBroadcastFinished && emergencyElapsed >= scEmergencyBroadcastDuration) {
        mEmergencyBroadcastFinished = true;
        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << now
            << " role=emergency-vehicle"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " laneIndex=" << mEgoContext.laneIndex
            << " event=emergency-broadcast-finished"
            << " messageCount=" << mEmergencyMcmCount
            << " elapsed=" << emergencyElapsed
            << " duration=" << scEmergencyBroadcastDuration
            << " sendInterval=" << scEmergencyBroadcastInterval
            << " speed=" << mEgoContext.speed
            << '\n';
    }

    const bool emergencySendDue =
        withinBroadcastWindow &&
        !mPendingMcmCommand &&
        (!mHasLastEmergencyMcmQueuedAt ||
            now - mLastEmergencyMcmQueuedAt >= omnetpp::SimTime(scEmergencyBroadcastInterval));

    if (emergencySendDue) {
        PendingMcmCommand command;
        command.kind = PendingMcmCommand::Kind::Execution;
        // Complete emergency semantics are not available in the current MCM
        // model, so the old implementation's Abort + EmergencyPriority
        // execution-container workaround is preserved here.
        command.subtype = mcmSubtype::Abort;
        command.priority = priorityMcmCategory::EmergencyPriority;
        command.cooperationType = 0;
        command.requestId = 1;
        command.numberOfVehicles = 1;
        command.targetVehicle1 = 1;
        command.hasTargetVehicle2 = false;
        command.targetVehicle2 = 0;
        command.requestedTrajectory = mEgoContext.plannedTrajectory;

        mPendingMcmCommand = command;
        mEmergencyMcmQueued = true;
        mLastEmergencyMcmQueuedAt = now;
        mHasLastEmergencyMcmQueuedAt = true;
        ++mEmergencyMcmCount;
        mCooperatingVehicleType = cooperatingVehicleType::EmergencyV;
        mMcmSubtype = mcmSubtype::Abort;
        mPriorityMcmCategory = priorityMcmCategory::EmergencyPriority;
        mOperationMode = operationMode::ManeuverExecutionMode;
        mCoordinationProgressRV = coordinationProgressRV::SendExecute;

        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << now
            << " role=emergency-vehicle"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " laneIndex=" << mEgoContext.laneIndex
            << " event=queue-emergency-execution-mcm"
            << " messageIndex=" << mEmergencyMcmCount
            << " elapsed=" << emergencyElapsed
            << " duration=" << scEmergencyBroadcastDuration
            << " sendInterval=" << scEmergencyBroadcastInterval
            << " scheduledStartTime=" << scEmergencyStartTime
            << " subtype=Abort"
            << " priority=EmergencyPriority"
            << " executionContainer=1"
            << " plannedTrajectoryPoints=" << mEgoContext.plannedTrajectory.size()
            << " speed=" << mEgoContext.speed
            << '\n';
    }

    if (!mEmergencyBrakeApplied) {
        try {
            mVehicleController->setMaxSpeed(scEmergencyMaxSpeed * boost::units::si::meter_per_second);
            mEmergencyBrakeApplied = true;

            EV_INFO << "[MCM-EMERGENCY]"
                << " simTime=" << now
                << " role=emergency-vehicle"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " laneIndex=" << mEgoContext.laneIndex
                << " event=emergency-brake-trigger"
                << " scheduledStartTime=" << scEmergencyStartTime
                << " maxSpeed=" << scEmergencyMaxSpeed
                << '\n';
        } catch (const std::exception& e) {
            EV_WARN << "[MCM-EMERGENCY]"
                << " simTime=" << now
                << " role=emergency-vehicle"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " laneIndex=" << mEgoContext.laneIndex
                << " event=emergency-brake-trigger-failed"
                << " scheduledStartTime=" << scEmergencyStartTime
                << " maxSpeed=" << scEmergencyMaxSpeed
                << " reason=\"" << e.what() << "\"\n";
        }
    }
}

void McApplication::logScenarioVehicleLifetime(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    if (!isSafetyCriticalLaneChangeScenarioVehicle(vehicleId)) {
        return;
    }

    const bool isEmergencyVehicle = vehicleId == scEmergencyVehicleId;
    const bool isLane1Follower = vehicleId == "car_hl1_1" ||
        vehicleId == "car_hl1_2" ||
        vehicleId == "car_hl1_3";
    const bool isTargetLaneCv = !isEmergencyVehicle && !isLane1Follower;

    const bool shouldLogFirstSeen = !mScenarioVehicleFirstSeenLogged;
    const bool shouldLogNearEmergency =
        !mScenarioVehicleNearEmergencyLogged &&
        now.dbl() >= scEmergencyStartTime - 0.25 &&
        now.dbl() < scEmergencyStartTime;
    const bool shouldLogAfterEmergency =
        !mScenarioVehicleAfterEmergencyLogged &&
        now.dbl() >= scEmergencyStartTime;

    if (!shouldLogFirstSeen && !shouldLogNearEmergency && !shouldLogAfterEmergency) {
        return;
    }

    std::string laneId = "unavailable";
    int laneIndex = mEgoContext.laneIndex;
    double lanePosition = 0.0;
    bool hasLanePosition = false;

    try {
        auto traci = mVehicleController->getTraCI();
        if (traci) {
            laneId = traci->vehicle.getLaneID(vehicleId);
            laneIndex = traci->vehicle.getLaneIndex(vehicleId);
            lanePosition = traci->vehicle.getLanePosition(vehicleId);
            hasLanePosition = true;
        }
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-VEHICLE-LIFETIME]"
            << " simTime=" << now
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << vehicleId
            << " event=traci-query-failed"
            << " reason=\"" << e.what() << "\"\n";
    }

    const char* event = shouldLogFirstSeen ? "first-seen" :
        (shouldLogNearEmergency ? "near-emergency-start" : "alive-after-emergency-start");

    EV_INFO << "[MCM-SCENARIO-DIAG]"
        << " simTime=" << now
        << " event=active-highway-emergency-scenario-vehicle"
        << " vehicleId=" << vehicleId
        << " station=" << mEgoContext.stationId
        << " route=" << mEgoContext.routeId
        << " expectedConfig=envmod-19CAVs"
        << " expectedSumocfg=routes/test_19CAVs.sumocfg"
        << " scheduledEmergencyStart=" << scEmergencyStartTime
        << '\n';

    EV_INFO << "[MCM-VEHICLE-LIFETIME]"
        << " simTime=" << now
        << " event=" << event
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
        << " role=" << (isEmergencyVehicle ? "emergency-vehicle-v0" :
            (isLane1Follower ? "safety-critical-lane-change-candidate-rv" : "target-lane-candidate-cv"))
        << " route=" << mEgoContext.routeId
        << " laneId=" << laneId
        << " laneIndex=" << laneIndex
        << " lanePosition=";
    if (hasLanePosition) {
        EV_INFO << lanePosition;
    } else {
        EV_INFO << "unavailable";
    }
    EV_INFO << " x=" << mEgoContext.x
        << " y=" << mEgoContext.y
        << " speed=" << mEgoContext.speed
        << " isEmergencyVehicle=" << isEmergencyVehicle
        << " isLane1Follower=" << isLane1Follower
        << " isTargetLaneCv=" << isTargetLaneCv
        << '\n';

    if (shouldLogFirstSeen) {
        mScenarioVehicleFirstSeenLogged = true;
    }
    if (shouldLogNearEmergency) {
        mScenarioVehicleNearEmergencyLogged = true;
    }
    if (shouldLogAfterEmergency) {
        mScenarioVehicleAfterEmergencyLogged = true;
    }
}

void McApplication::evaluateSafetyCriticalLaneChangeTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleController || !mVehicleDataProvider ||
            mPendingMcmCommand || mLaneChangeRequestQueuedOrSent ||
            !mEmergencyReceived ||
            mEgoContext.routeId != scSafetyCriticalLaneChangeRouteId ||
            mCoordinationProgressRV != coordinationProgressRV::CheckForCoordination ||
            mControlManeuver != controlManeuver::ChangeLane) {
        return;
    }

    TrajectoryPlanner::Trajectory laneChangeTrajectory;
    if (!mEgoContext.routeReferenceX.empty() && !mEgoContext.routeReferenceY.empty() &&
            mEgoContext.routeReferenceIndex >= 0) {
        auto shiftedX = mEgoContext.routeReferenceX;
        std::transform(shiftedX.begin(), shiftedX.end(), shiftedX.begin(),
            [](float x) { return x + static_cast<float>(scLaneChangeShiftX); });
        laneChangeTrajectory = mTrajectoryPlanner.calculateRefTrajectory(
            scRequestTrajectorySteps,
            scRequestTrajectoryDt,
            shiftedX,
            mEgoContext.routeReferenceY,
            mEgoContext.routeReferenceIndex,
            true,
            0.0F,
            0.0F,
            0.0F);
    }

    if (laneChangeTrajectory.empty()) {
        laneChangeTrajectory = mEgoContext.plannedTrajectory;
        for (auto& point : laneChangeTrajectory) {
            point.mX += scLaneChangeShiftX;
        }
    }

    if (laneChangeTrajectory.empty()) {
        EV_WARN << "[MCM-LC-TRIGGER]"
            << " simTime=" << now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " result=no-request"
            << " reason=no-lane-change-trajectory\n";
        return;
    }

    unsigned considered = 0;
    unsigned conflicts = 0;
    uint32_t target1 = 0;
    uint32_t target2 = 0;

    // Arming the emergency response does not by itself send a Request. The RV
    // still scans target-lane MCM trajectories and selects only CVs whose
    // planned trajectories conflict with the intended lane-change trajectory.
    for (const auto& received : mReceivedMcmCache) {
        const auto& snapshot = received.data;
        if (snapshot.stationId == mEgoContext.stationId || snapshot.stationId == mEmergencyStationId ||
                !snapshot.hasLaneId || snapshot.plannedTrajectory.empty()) {
            continue;
        }

        const auto& first = snapshot.plannedTrajectory.front();
        const int rawLaneReceived = static_cast<int>(snapshot.laneId);
        int laneReceived = rawLaneReceived;
        const bool laneWorkaroundApplied = first.mY > 452365.0;
        if (laneWorkaroundApplied) {
            laneReceived += 1;
        }
        const bool targetLaneMatch = laneReceived == mEgoContext.laneIndex + 1;

        EV_INFO << "[MCM-LC-LANE-WORKAROUND]"
            << " simTime=" << now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " rvLaneIndex=" << mEgoContext.laneIndex
            << " targetLaneIndex=" << (mEgoContext.laneIndex + 1)
            << " candidateCvStation=" << snapshot.stationId
            << " rawLaneReceived=" << rawLaneReceived
            << " firstTrajectoryY=" << first.mY
            << " workaroundThresholdY=452365"
            << " workaroundApplied=" << laneWorkaroundApplied
            << " correctedLaneReceived=" << laneReceived
            << " targetLaneMatch=" << targetLaneMatch
            << " activeScenarioFlag=EmergencyReceived"
            << " emergencyReceived=" << mEmergencyReceived
            << " controlManeuver=" << controlManeuverName(mControlManeuver)
            << '\n';

        if (!targetLaneMatch) {
            continue;
        }

        ++considered;
        const omnetpp::SimTime eteDelay =
            std::max(omnetpp::SimTime::ZERO, now - received.receivedAt);
        const bool conflict = mTrajectoryPlanner.check_traj_conflict(
            laneChangeTrajectory,
            snapshot.plannedTrajectory,
            scMergingTimeGap,
            eteDelay);

        EV_INFO << "[MCM-LC-3VEH]"
            << " simTime=" << now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " candidateCvStation=" << snapshot.stationId
            << " candidateCvRawLane=" << rawLaneReceived
            << " candidateCvCorrectedLane=" << laneReceived
            << " targetLane=" << (mEgoContext.laneIndex + 1)
            << " eteDelay=" << eteDelay
            << " conflict=" << conflict
            << " selectedCv1=" << target1
            << " selectedCv2=" << target2
            << " currentConflicts=" << conflicts
            << " source=latest-mcm-plannedTrajectory\n";

        if (!conflict) {
            continue;
        }

        if (target1 == 0 || target1 == snapshot.stationId) {
            target1 = snapshot.stationId;
            ++conflicts;
        } else if (target2 == 0 && target1 != snapshot.stationId) {
            target2 = snapshot.stationId;
            ++conflicts;
        }

        if (target1 != 0 && target2 != 0) {
            break;
        }
    }

    EV_INFO << "[MCM-LC-TRIGGER]"
        << " simTime=" << now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " laneIndex=" << mEgoContext.laneIndex
        << " emergencyStation=" << mEmergencyStationId
        << " emergencyDistanceGap=" << mEmergencyDistanceGap
        << " emergencyTimeGap=" << mEmergencyTimeGap
        << " reactionWindow=" << mEmergencyReactionWindow
        << " consideredTargetLaneCv=" << considered
        << " conflicts=" << conflicts
        << " priority=HighPriority"
        << '\n';

    if (target1 == 0) {
        EV_WARN << "[MCM-LC-TRIGGER]"
            << " simTime=" << now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " result=no-request"
            << " reason=no-conflicting-target-lane-cv\n";
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Request;
    command.priority = priorityMcmCategory::HighPriority;
    command.cooperationType = 0;
    command.requestId = makeRequestId(now);
    command.numberOfVehicles = target2 == 0 ? 1 : 2;
    command.targetVehicle1 = target1;
    command.hasTargetVehicle2 = target2 != 0;
    command.targetVehicle2 = target2;
    command.requestedTrajectory = laneChangeTrajectory;

    mPendingMcmCommand = command;
    mLaneChangeRequestQueuedOrSent = true;
    mLaneChangeThreeVehiclePath = target2 != 0;
    mRvRequestId = command.requestId;
    mRvNumberOfVehicles = command.numberOfVehicles;
    mRvTargetVehicle1 = command.targetVehicle1;
    mRvTargetVehicle2 = command.hasTargetVehicle2 ? command.targetVehicle2 : 0;
    mRvOfferReceived1 = false;
    mRvOfferReceived2 = false;
    mRvConfirmQueuedOrSent = false;
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mRvNegotiationCompletionReported = false;
    mCompletedRvNegotiationRequestId.reset();
    mRvLastRequestQueuedAt = now;
    mHasRvLastRequestQueuedAt = true;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvNegotiationStartedAt = now;
    mHasRvNegotiationStartedAt = true;
    mRvNegotiationTimedOut = false;
    mActiveNegotiatedTrajectory = laneChangeTrajectory;
    mHasActiveNegotiatedTrajectory = !mActiveNegotiatedTrajectory.empty();
    mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
    mHasLastExecuteQueuedAt = false;

    mCooperatingVehicleType = cooperatingVehicleType::RV;
    mMcmSubtype = mcmSubtype::Request;
    mPriorityMcmCategory = priorityMcmCategory::HighPriority;
    mOperationMode = operationMode::ManeuverNegotiationMode;
    mCoordinationProgressRV = coordinationProgressRV::CoordinationRequired;

    EV_INFO << "[MCM-LC-3VEH]"
        << " simTime=" << now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " event=queued-request"
        << " requestId=" << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " targetCv1=" << command.targetVehicle1
        << " targetCv2=" << command.targetVehicle2
        << " path=" << (mLaneChangeThreeVehiclePath ? "three-vehicle-two-cv" : "two-vehicle-one-cv")
        << " priority=HighPriority"
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';
}

McApplication::CvCooperationDecision McApplication::evaluateCvCooperationDecision(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    // This wrapper only gathers request context, leader information, and log
    // fields. Feasibility, candidate trajectory generation, and cooperation
    // cost stay in TrajectoryPlanner::findSuitableTrajectoryCV().
    CvCooperationDecision decision;
    const auto& snapshot = received.data;
    const uint32_t egoStationId = mVehicleDataProvider ? mVehicleDataProvider->station_id() : 0;
    const uint8_t requestId = snapshot.requestId >= 0 ? static_cast<uint8_t>(snapshot.requestId) : 0;
    const int requestPriority = priorityLevel(mPriorityMcmCategory);
    decision.threshold = costThresholdForPriority(mPriorityMcmCategory);

    EV_INFO << "[MCM-CV-DECISION]"
        << " simTime=" << (mHasEgoContext ? mEgoContext.now : received.receivedAt)
        << " event=received-request"
        << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " cvStation=" << egoStationId
        << " rvStation=" << snapshot.stationId
        << " requestId=" << static_cast<int>(requestId)
        << " priority=" << priorityName(snapshot.priorityManeuver)
        << " numberOfVehicles=" << snapshot.numberOfVehicles
        << " requestedTrajectoryPoints=" << snapshot.requestedTrajectory.size()
        << '\n';

    if (!mHasEgoContext || !mVehicleDataProvider || !mVehicleController) {
        decision.reason = "missing-ego-context-or-controller";
        return decision;
    }

    if (snapshot.requestedTrajectory.empty()) {
        decision.reason = "missing-requested-trajectory";
        return decision;
    }

    if (mEgoContext.routeReferenceX.empty() || mEgoContext.routeReferenceY.empty() ||
            mEgoContext.routeReferenceIndex < 0) {
        decision.reason = "missing-route-reference";
        return decision;
    }

    if (mEgoContext.plannedTrajectory.empty()) {
        decision.reason = "missing-ego-planned-trajectory";
        return decision;
    }

    FrontVehicleInfo leaderInfo;
    try {
        // Leader/front-vehicle constraints come from the existing local
        // environment model through TrajectoryPlanner; do not create a parallel
        // perception path for CV decisions.
        leaderInfo = mTrajectoryPlanner.getFrontVehicleInfo();
    } catch (const std::exception& e) {
        EV_WARN << "[MCM-CV-DECISION]"
            << " simTime=" << mEgoContext.now
            << " event=leader-monitor"
            << " cvVehicleId=" << mVehicleController->getVehicleId()
            << " cvStation=" << egoStationId
            << " rvStation=" << snapshot.stationId
            << " requestId=" << static_cast<int>(requestId)
            << " leaderLookupFailed=true"
            << " reason=" << e.what()
            << '\n';
    }

    decision.leaderSeen = leaderInfo.sameEdgeLane;
    decision.leaderVehicleId = leaderInfo.frontVehicleId;
    decision.leaderDistance = leaderInfo.distance;
    decision.leaderSpeed = leaderInfo.speed;

    EV_INFO << "[MCM-CV-DECISION]"
        << " simTime=" << mEgoContext.now
        << " event=leader-monitor"
        << " cvVehicleId=" << mVehicleController->getVehicleId()
        << " cvStation=" << egoStationId
        << " rvStation=" << snapshot.stationId
        << " requestId=" << static_cast<int>(requestId)
        << " leaderSeen=" << decision.leaderSeen
        << " leaderVehicleId=" << decision.leaderVehicleId
        << " leaderDistance=" << decision.leaderDistance
        << " leaderSpeed=" << decision.leaderSpeed
        << " cvRoute=" << mEgoContext.routeId
        << " cvLaneIndex=" << mEgoContext.laneIndex
        << '\n';

    const omnetpp::SimTime eteDelay =
        std::max(omnetpp::SimTime::ZERO, mEgoContext.now - received.receivedAt);
    decision.conflictWithRequestedTrajectory = mTrajectoryPlanner.check_traj_conflict_lane_change(
        mEgoContext.plannedTrajectory,
        snapshot.requestedTrajectory,
        scMergingTimeGap,
        eteDelay);

    const auto& myFirstPoint = mEgoContext.plannedTrajectory.front();
    const auto& requestedFirstPoint = snapshot.requestedTrajectory.front();
    const bool decelerationRequired = myFirstPoint.mY >= requestedFirstPoint.mY;
    const bool accelerationRequired = !decelerationRequired;
    const bool laneChangePossible = false;
    const bool routeAffected = false;

    EV_INFO << "[MCM-CV-DECISION]"
        << " simTime=" << mEgoContext.now
        << " event=trajectory-candidate"
        << " cvVehicleId=" << mVehicleController->getVehicleId()
        << " cvStation=" << egoStationId
        << " rvStation=" << snapshot.stationId
        << " requestId=" << static_cast<int>(requestId)
        << " priority=" << priorityName(snapshot.priorityManeuver)
        << " conflictWithRequestedTrajectory=" << decision.conflictWithRequestedTrajectory
        << " decelerationRequired=" << decelerationRequired
        << " accelerationRequired=" << accelerationRequired
        << " laneChangePossible=" << laneChangePossible
        << " myFirstY=" << myFirstPoint.mY
        << " requestedFirstY=" << requestedFirstPoint.mY
        << '\n';

    if (!decision.conflictWithRequestedTrajectory) {
        decision.feasible = true;
        decision.responseSubtype = snapshot.numberOfVehicles > 1 ? mcmSubtype::Offer : mcmSubtype::Accept;
        decision.selectedManeuver = controlManeuver::DoNothing;
        decision.selectedTrajectory = mEgoContext.plannedTrajectory;
        decision.cooperationCost = 0.0;
        decision.threshold = costThresholdForPriority(mPriorityMcmCategory);
        decision.trajectoryType = 0;
        decision.possiblePriorityLevel = 0;
        decision.reason = "no-conflict-with-requested-trajectory";
    } else {
        auto result = mTrajectoryPlanner.findSuitableTrajectoryCV(
            snapshot.requestedTrajectory,
            requestPriority,
            false,
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

        decision.feasible = std::get<0>(result);
        decision.selectedTrajectory = std::get<1>(result);
        decision.plannedValues = std::get<2>(result);
        decision.cooperationCost = std::get<3>(result);
        decision.trajectoryType = std::get<4>(result);
        decision.possiblePriorityLevel = std::get<5>(result);
        decision.responseSubtype = snapshot.numberOfVehicles > 1 ? mcmSubtype::Offer : mcmSubtype::Accept;

        if (decision.plannedValues.lane_change) {
            decision.selectedManeuver = controlManeuver::ChangeLane;
        } else if (decision.plannedValues.deceleration_change > 0.0) {
            decision.selectedManeuver = controlManeuver::Decelerate;
        } else if (decision.plannedValues.acc_change > 0.0) {
            decision.selectedManeuver = controlManeuver::Accelerate;
        } else {
            decision.selectedManeuver = controlManeuver::DoNothing;
        }

        const bool priorityAllowsCost =
            decision.possiblePriorityLevel >= 0 &&
            decision.possiblePriorityLevel <= requestPriority;
        // Old priority-level ordering is numeric severity:
        // 0 = Low, 1 = Medium, 2 = High. A request may accept a cooperation
        // trajectory whose required priority is less than or equal to its own.
        const bool leaderBlocksAcceleration =
            decision.selectedManeuver == controlManeuver::Accelerate &&
            decision.leaderSeen &&
            decision.leaderDistance < std::max(3.0, 0.5 * std::max(0.0, mEgoContext.speed)) &&
            decision.leaderSpeed < mEgoContext.speed;

        if (!decision.feasible) {
            decision.reason = "planner-found-no-feasible-trajectory";
        } else if (!priorityAllowsCost) {
            decision.feasible = false;
            decision.reason = "possible-priority-level-exceeds-request-priority";
        } else if (leaderBlocksAcceleration) {
            decision.feasible = false;
            decision.reason = "leader-blocks-acceleration";
        } else {
            decision.reason = "planner-found-feasible-trajectory";
        }
    }

    EV_INFO << "[MCM-CV-DECISION]"
        << " simTime=" << mEgoContext.now
        << " event=cooperation-cost"
        << " cvVehicleId=" << mVehicleController->getVehicleId()
        << " cvStation=" << egoStationId
        << " rvStation=" << snapshot.stationId
        << " requestId=" << static_cast<int>(requestId)
        << " selectedAction=" << controlManeuverName(decision.selectedManeuver)
        << " cooperationCost=" << decision.cooperationCost
        << " threshold=" << decision.threshold
        << " possiblePriorityLevel=" << decision.possiblePriorityLevel
        << " requestPriorityLevel=" << requestPriority
        << " trajectoryType=" << decision.trajectoryType
        << " feasible=" << decision.feasible
        << " speedChange=" << decision.plannedValues.speed_change
        << " acceleration=" << decision.plannedValues.acc_change
        << " deceleration=" << decision.plannedValues.deceleration_change
        << " laneChange=" << decision.plannedValues.lane_change
        << " timeGap=" << decision.plannedValues.time_gap_change
        << " ttc=" << decision.plannedValues.TTC_change
        << " leaderSeen=" << decision.leaderSeen
        << " leaderVehicleId=" << decision.leaderVehicleId
        << " leaderDistance=" << decision.leaderDistance
        << " leaderSpeed=" << decision.leaderSpeed
        << " reason=" << decision.reason
        << '\n';

    return decision;
}

void McApplication::evaluateCvRequestResponse(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mVehicleDataProvider || mPendingMcmCommand || mCvResponseQueuedOrSent ||
            mCoordinationProgressCV != coordinationProgressCV::NoRequest) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Request)) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    const bool targetsEgo = snapshot.negotiationVehicleId1 == egoStationId ||
        (snapshot.hasNegotiationVehicleId2 && snapshot.negotiationVehicleId2 == egoStationId);
    if (!targetsEgo) {
        return;
    }

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    mCvRvStationId = snapshot.stationId;
    mCvRequestId = snapshot.requestId >= 0 ? static_cast<uint8_t>(snapshot.requestId) : 0;
    mCooperatingVehicleType = cooperatingVehicleType::CV;
    mOperationMode = operationMode::ManeuverNegotiationMode;
    mCoordinationProgressCV = coordinationProgressCV::ReceivedRequest;
    mPriorityMcmCategory = priorityFromMcm(snapshot.priorityManeuver);
    mControlManeuver = controlManeuver::DoNothing;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mCvDecelerationControlApplied = false;
    mCvDecelerationControlSkippedLogged = false;
    mCvAccelerationControlApplied = false;
    mCvLaneChangeControlLogged = false;

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = snapshot.cooperationTypeMcm >= 0 ? snapshot.cooperationTypeMcm : 0;
    command.requestId = mCvRequestId;
    command.numberOfVehicles = 1;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;

    if (snapshot.requestedTrajectory.empty()) {
        command.subtype = mcmSubtype::Reject;
        command.requestedTrajectory = mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};
        mCoordinationProgressCV = coordinationProgressCV::SendReject;

        EV_WARN << "McApplication CV station " << egoStationId
            << " rejecting Request " << static_cast<int>(command.requestId)
            << " from RV " << mCvRvStationId
            << " because requestedTrajectory is missing\n";

        // Temporary manual debug helper. Uncomment if EV_INFO is suppressed in Cmdenv.
        // std::cout << "MCM_DEBUG CV station " << egoStationId
        //     << " queued Reject for requestId " << static_cast<int>(command.requestId)
        //     << " to RV " << mCvRvStationId
        //     << " at " << omnetpp::simTime() << " s" << std::endl;
    } else if (mPriorityMcmCategory == priorityMcmCategory::HighPriority &&
            mHasEgoContext && mEgoContext.routeId == scTargetLaneChangeRouteId) {
        CvCooperationDecision decision = evaluateCvCooperationDecision(received);
        if (decision.feasible) {
            command.subtype = decision.responseSubtype;
            command.requestedTrajectory = snapshot.requestedTrajectory;
            command.offeredTrajectory = !decision.selectedTrajectory.empty() ?
                decision.selectedTrajectory : mEgoContext.plannedTrajectory;
            command.hasOfferedTrajectory = !command.offeredTrajectory.empty();
            mCoordinationProgressCV = command.subtype == mcmSubtype::Offer ?
                coordinationProgressCV::SendOffer : coordinationProgressCV::SendAccept;

            mControlManeuver = decision.selectedManeuver;
            mCvSelectedTrajectory = decision.selectedTrajectory;
            mHasCvSelectedTrajectory = !mCvSelectedTrajectory.empty();

            if (decision.selectedManeuver == controlManeuver::Decelerate) {
                mTargetSpeed = mEgoContext.speed - decision.plannedValues.speed_change * mEgoContext.speed;
                mCommandDuration = calculateDecelerationTime(
                    mEgoContext.speed,
                    mTargetSpeed,
                    decision.plannedValues.deceleration_change + 0.05);
            } else if (decision.selectedManeuver == controlManeuver::Accelerate) {
                mTargetSpeed = mEgoContext.speed + decision.plannedValues.speed_change * mEgoContext.speed;
            }

            EV_INFO << "[MCM-CV-DECISION]"
                << " simTime=" << mEgoContext.now
                << " event=" << (command.subtype == mcmSubtype::Offer ? "feasible-offer" : "feasible-accept")
                << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
                << " cvStation=" << egoStationId
                << " rvStation=" << command.targetVehicle1
                << " requestId=" << static_cast<int>(command.requestId)
                << " priority=HighPriority"
                << " numberOfVehicles=" << snapshot.numberOfVehicles
                << " selectedAction=" << controlManeuverName(mControlManeuver)
                << " cooperationCost=" << decision.cooperationCost
                << " threshold=" << decision.threshold
                << " feasible=true"
                << " targetSpeed=" << mTargetSpeed
                << " decelerationTime=" << mCommandDuration
                << " offeredTrajectoryPoints=" << command.offeredTrajectory.size()
                << " reason=" << decision.reason
                << '\n';
        } else {
            command.subtype = mcmSubtype::Reject;
            command.requestedTrajectory = mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};
            command.offeredTrajectory.clear();
            command.hasOfferedTrajectory = false;
            mCoordinationProgressCV = coordinationProgressCV::SendReject;

            EV_WARN << "[MCM-CV-DECISION]"
                << " simTime=" << mEgoContext.now
                << " event=reject"
                << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
                << " cvStation=" << egoStationId
                << " rvStation=" << command.targetVehicle1
                << " requestId=" << static_cast<int>(command.requestId)
                << " priority=HighPriority"
                << " numberOfVehicles=" << snapshot.numberOfVehicles
                << " selectedAction=" << controlManeuverName(decision.selectedManeuver)
                << " cooperationCost=" << decision.cooperationCost
                << " threshold=" << decision.threshold
                << " feasible=false"
                << " leaderSeen=" << decision.leaderSeen
                << " leaderVehicleId=" << decision.leaderVehicleId
                << " leaderDistance=" << decision.leaderDistance
                << " leaderSpeed=" << decision.leaderSpeed
                << " reason=" << decision.reason
                << '\n';
        }

        EV_INFO << "[MCM-LC-3VEH]"
            << " simTime=" << mEgoContext.now
            << " role=target-cv"
            << " station=" << egoStationId
            << " route=" << mEgoContext.routeId
            << " laneIndex=" << mEgoContext.laneIndex
            << " event=received-high-priority-lane-change-request"
            << " response=" << subtypeName(command.subtype)
            << " requestId=" << static_cast<int>(command.requestId)
            << " rvStation=" << command.targetVehicle1
            << " numberOfVehicles=" << snapshot.numberOfVehicles
            << " path=" << (snapshot.numberOfVehicles > 1 ? "three-vehicle-two-cv" : "two-vehicle-one-cv")
            << " priority=HighPriority"
            << " selectedAction=" << controlManeuverName(mControlManeuver)
            << " feasible=" << decision.feasible
            << " requestedTrajectoryPoints=" << snapshot.requestedTrajectory.size()
            << " offeredTrajectoryPoints=" << (command.hasOfferedTrajectory ? command.offeredTrajectory.size() : 0)
            << '\n';
    } else if (snapshot.numberOfVehicles > 1) {
        classifyCvMergingControlManeuver(received);

        command.subtype = mcmSubtype::Offer;
        command.requestedTrajectory = snapshot.requestedTrajectory;
        command.offeredTrajectory = mHasEgoContext && !mEgoContext.plannedTrajectory.empty() ?
            mEgoContext.plannedTrajectory : snapshot.requestedTrajectory;
        command.hasOfferedTrajectory = !command.offeredTrajectory.empty();
        mCoordinationProgressCV = coordinationProgressCV::SendOffer;

        EV_INFO << "McApplication CV station " << egoStationId
            << " offering for targeted route_merging_1 Request "
            << static_cast<int>(command.requestId)
            << " from RV " << mCvRvStationId
            << " because numberOfVehicles=" << snapshot.numberOfVehicles
            << " matches old two-CV Offer path"
            << " requestedTrajectoryPoints=" << snapshot.requestedTrajectory.size()
            << " offeredTrajectoryPoints=" << command.offeredTrajectory.size() << '\n';

        // Temporary manual debug helper. Uncomment if EV_INFO is suppressed in Cmdenv.
        // std::cout << "MCM_DEBUG CV station " << egoStationId
        //     << " queued Offer for requestId " << static_cast<int>(command.requestId)
        //     << " to RV " << mCvRvStationId
        //     << " offeredTrajectoryPoints=" << command.offeredTrajectory.size()
        //     << " at " << omnetpp::simTime() << " s" << std::endl;
    } else {
        classifyCvMergingControlManeuver(received);

        command.subtype = mcmSubtype::Accept;
        command.requestedTrajectory = mHasEgoContext && !mEgoContext.plannedTrajectory.empty() ?
            mEgoContext.plannedTrajectory : snapshot.requestedTrajectory;
        mCoordinationProgressCV = coordinationProgressCV::SendAccept;

        EV_INFO << "McApplication CV station " << egoStationId
            << " accepted targeted route_merging_1 Request "
            << static_cast<int>(command.requestId)
            << " from RV " << mCvRvStationId
            << " because numberOfVehicles=" << snapshot.numberOfVehicles
            << " matches old one-CV Accept path"
            << " requestedTrajectoryPoints=" << snapshot.requestedTrajectory.size() << '\n';

        // Temporary manual debug helper. Uncomment if EV_INFO is suppressed in Cmdenv.
        // std::cout << "MCM_DEBUG CV station " << egoStationId
        //     << " queued Accept for requestId " << static_cast<int>(command.requestId)
        //     << " to RV " << mCvRvStationId
        //     << " at " << omnetpp::simTime() << " s" << std::endl;
    }

    mMcmSubtype = command.subtype;
    mPendingMcmCommand = command;
    mCvResponseQueuedOrSent = true;
    mCvNegotiationStartedAt = received.receivedAt;
    mHasCvNegotiationStartedAt = true;
    mCvResponseNumberOfVehicles = command.numberOfVehicles;

    if (command.subtype == mcmSubtype::Offer) {
        mCvLastOfferQueuedAt = received.receivedAt;
        mHasCvLastOfferQueuedAt = true;
        mCvLastAcceptQueuedAt = omnetpp::SimTime::ZERO;
        mHasCvLastAcceptQueuedAt = false;
    } else if (command.subtype == mcmSubtype::Accept) {
        mCvLastAcceptQueuedAt = received.receivedAt;
        mHasCvLastAcceptQueuedAt = true;
        mCvLastOfferQueuedAt = omnetpp::SimTime::ZERO;
        mHasCvLastOfferQueuedAt = false;
    }

    EV_INFO << "McApplication queued CV " << subtypeName(command.subtype)
        << ": requestId=" << static_cast<int>(command.requestId)
        << " rvStation=" << command.targetVehicle1
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << " offeredTrajectoryPoints="
        << (command.hasOfferedTrajectory ? command.offeredTrajectory.size() : 0)
        << '\n';
}

void McApplication::handleReceivedCancelAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV == coordinationProgressCV::NoRequest ||
            mCoordinationProgressCV == coordinationProgressCV::CompleteSentCV) {
        return;
    }

    // This rollback path is only for early CV speed control that was applied
    // before Execute. During execution the legacy ASN.1 uses Cancel as the
    // normal Complete workaround too, so execution-phase Cancel needs a later
    // explicit abort-vs-complete distinction before it can be consumed here.
    if (mOperationMode != operationMode::ManeuverNegotiationMode) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Cancel)) {
        return;
    }

    if (snapshot.stationId != mCvRvStationId) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mCvRequestId) {
        return;
    }

    const auto previousProgress = mCoordinationProgressCV;
    const bool hadDecelerationControl = mCvDecelerationControlApplied;
    const bool hadAccelerationControl = mCvAccelerationControlApplied;

    restoreCvSpeedControl();

    mPendingMcmCommand.reset();
    resetCvCoordinationStateAfterComplete();
    mCoordinationProgressCV = coordinationProgressCV::NoRequest;

    EV_INFO << "McApplication CV station " << mEgoContext.stationId
        << " rolled back coordination after receiving Cancel from RV"
        << ": requestId=" << snapshot.requestId
        << " rvStation=" << snapshot.stationId
        << " previousProgress=" << static_cast<int>(previousProgress)
        << " restoredAfterDeceleration=" << hadDecelerationControl
        << " restoredAfterAcceleration=" << hadAccelerationControl
        << '\n';
}

void McApplication::handleReceivedOfferAsRv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mRvConfirmQueuedOrSent ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            (mCoordinationProgressRV != coordinationProgressRV::CoordinationRequired &&
                mCoordinationProgressRV != coordinationProgressRV::RequestSent)) {
        return;
    }

    const auto& snapshot = received.data;
    if (!isNegotiationMessageForActiveRequest(snapshot, mcmSubtype::Offer, mRvRequestId)) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;
    bool fromTarget1 = false;
    bool fromTarget2 = false;
    if (!markRvResponseFromExpectedCv(senderStationId, fromTarget1, fromTarget2)) {
        return;
    }

    if (fromTarget1 && !mRvOfferReceived1) {
        mRvOfferReceived1 = true;
        if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mEgoContext.now
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " event=received-offer-1"
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " cvStation=" << senderStationId
                << " path=three-vehicle-two-cv\n";
        }
        EV_INFO << "McApplication RV station " << mEgoContext.stationId
            << " received Offer 1 from CV " << senderStationId
            << " for requestId=" << static_cast<int>(mRvRequestId)
            << " offeredTrajectoryPoints=" << snapshot.offeredTrajectoryPointCount << '\n';
    } else if (fromTarget2 && !mRvOfferReceived2) {
        mRvOfferReceived2 = true;
        if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mEgoContext.now
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " event=received-offer-2"
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " cvStation=" << senderStationId
                << " path=three-vehicle-two-cv\n";
        }
        EV_INFO << "McApplication RV station " << mEgoContext.stationId
            << " received Offer 2 from CV " << senderStationId
            << " for requestId=" << static_cast<int>(mRvRequestId)
            << " offeredTrajectoryPoints=" << snapshot.offeredTrajectoryPointCount << '\n';
    } else {
        return;
    }

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    const bool allExpectedOffersReceived =
        mRvNumberOfVehicles > 1 ?
        (mRvOfferReceived1 && mRvOfferReceived2) :
        mRvOfferReceived1;

    if (!allExpectedOffersReceived) {
        return;
    }

    PendingMcmCommand command = makeRvFollowupCommand(mcmSubtype::Confirm);

    mPendingMcmCommand = command;
    mRvConfirmQueuedOrSent = true;
    mRvLastConfirmQueuedAt = received.receivedAt;
    mHasRvLastConfirmQueuedAt = true;
    mMcmSubtype = mcmSubtype::Confirm;
    mCoordinationProgressRV = coordinationProgressRV::SendConfirm;

    if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
        EV_INFO << "[MCM-LC-3VEH]"
            << " simTime=" << mEgoContext.now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " event=queued-confirm-after-all-offers"
            << " requestId=" << static_cast<int>(command.requestId)
            << " targetCv1=" << command.targetVehicle1
            << " targetCv2=" << command.targetVehicle2
            << " priority=HighPriority\n";
    }

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " queued Confirm after receiving all Offers"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';

    // std::cout << "MCM_DEBUG RV station " << mEgoContext.stationId
    //     << " queued Confirm for requestId " << static_cast<int>(command.requestId)
    //     << " after receiving Offers from " << mRvTargetVehicle1
    //     << " and " << mRvTargetVehicle2
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

void McApplication::handleReceivedConfirmAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV != coordinationProgressCV::SendOffer) {
        return;
    }

    const auto& snapshot = received.data;
    if (!isNegotiationMessageForActiveRequest(snapshot, mcmSubtype::Confirm, mCvRequestId) ||
            snapshot.stationId != mCvRvStationId ||
            !isSnapshotTargetingEgo(snapshot)) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    PendingMcmCommand command = makeCvAcceptCommand(snapshot);

    mPendingMcmCommand = command;
    mMcmSubtype = mcmSubtype::Accept;
    mCoordinationProgressCV = coordinationProgressCV::SendAccept;
    mCvLastAcceptQueuedAt = received.receivedAt;
    mHasCvLastAcceptQueuedAt = true;

    if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
        EV_INFO << "[MCM-LC-STATE]"
            << " simTime=" << mEgoContext.now
            << " role=target-cv"
            << " station=" << egoStationId
            << " event=queued-accept-after-confirm"
            << " requestId=" << static_cast<int>(command.requestId)
            << " rvStation=" << command.targetVehicle1
            << " priority=HighPriority"
            << " selectedAction=" << controlManeuverName(mControlManeuver)
            << " selectedTrajectoryPoints=" << (mHasCvSelectedTrajectory ? mCvSelectedTrajectory.size() : 0)
            << '\n';
    }

    EV_INFO << "McApplication CV station " << egoStationId
        << " queued Accept after receiving Confirm"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " rvStation=" << command.targetVehicle1
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';

    // std::cout << "MCM_DEBUG CV station " << egoStationId
    //     << " queued Accept for requestId " << static_cast<int>(command.requestId)
    //     << " after receiving Confirm from RV " << mCvRvStationId
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

void McApplication::handleReceivedAcceptAsRv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mRvExecuteQueuedOrSent ||
            mCooperatingVehicleType != cooperatingVehicleType::RV) {
        return;
    }

    const auto& snapshot = received.data;

    if (!isNegotiationMessageForActiveRequest(snapshot, mcmSubtype::Accept, mRvRequestId)) {
        return;
    }

    const bool waitingForAcceptAsRv =
        mCoordinationProgressRV == coordinationProgressRV::SendConfirm ||
        (mRvNumberOfVehicles <= 1 &&
                mCoordinationProgressRV == coordinationProgressRV::RequestSent);

    if (!waitingForAcceptAsRv) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;
    bool fromTarget1 = false;
    bool fromTarget2 = false;
    if (!markRvResponseFromExpectedCv(senderStationId, fromTarget1, fromTarget2)) {
        return;
    }

    if (fromTarget1 && !mRvAcceptReceived1) {
        mRvAcceptReceived1 = true;
        if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mEgoContext.now
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " event=received-accept-1"
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " cvStation=" << senderStationId
                << '\n';
        }
        EV_INFO << "McApplication RV station " << mEgoContext.stationId
            << " received Accept 1 from CV " << senderStationId
            << " for requestId=" << static_cast<int>(mRvRequestId) << '\n';
    } else if (fromTarget2 && !mRvAcceptReceived2) {
        mRvAcceptReceived2 = true;
        if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mEgoContext.now
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " event=received-accept-2"
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " cvStation=" << senderStationId
                << '\n';
        }
        EV_INFO << "McApplication RV station " << mEgoContext.stationId
            << " received Accept 2 from CV " << senderStationId
            << " for requestId=" << static_cast<int>(mRvRequestId) << '\n';
    } else {
        return;
    }

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    const bool allExpectedAcceptsReceived =
        mRvNumberOfVehicles > 1 ?
        (mRvAcceptReceived1 && mRvAcceptReceived2) :
        mRvAcceptReceived1;

    if (!allExpectedAcceptsReceived) {
        return;
    }

    PendingMcmCommand command = makeRvFollowupCommand(
        mcmSubtype::Execute,
        snapshot.cooperationTypeMcm >= 0 ? snapshot.cooperationTypeMcm : 0);

    mActiveNegotiatedTrajectory = command.requestedTrajectory;
    mHasActiveNegotiatedTrajectory = !mActiveNegotiatedTrajectory.empty();

    mPendingMcmCommand = command;
    if (!mRvNegotiationCompletionReported) {
        mCompletedRvNegotiationRequestId = command.requestId;
        mRvNegotiationCompletionReported = true;
    }
    mRvExecuteQueuedOrSent = true;
    mLastExecuteQueuedAt = mEgoContext.now;
    mHasLastExecuteQueuedAt = true;
    mMcmSubtype = mcmSubtype::Execute;
    mOperationMode = operationMode::ManeuverExecutionMode;
    mCoordinationProgressRV = coordinationProgressRV::SendExecute;
    mMergingGapDiagActive = mEgoContext.routeId == scMergingRouteId;
    mMergingGapDiagExecutionStart = mEgoContext.now;
    mMergingGapDiagTargetCvStationId = mRvTargetVehicle1;
    sampleMergingGapDiagnostics("execution-start");

    if (mPriorityMcmCategory == priorityMcmCategory::HighPriority &&
            mEgoContext.routeId == scSafetyCriticalLaneChangeRouteId) {
        EV_INFO << "[MCM-LC-3VEH]"
            << " simTime=" << mEgoContext.now
            << " role=safety-critical-lane-change-rv"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " event=queued-execute-after-all-accepts"
            << " requestId=" << static_cast<int>(command.requestId)
            << " targetCv1=" << command.targetVehicle1
            << " targetCv2=" << command.targetVehicle2
            << " priority=HighPriority"
            << " execution=moveToXY-10-step-monitoring\n";
    }

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " queued Execute after receiving all Accepts"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << '\n';

    // std::cout << "MCM_DEBUG RV station " << mEgoContext.stationId
    //     << " queued Execute for requestId " << static_cast<int>(command.requestId)
    //     << " after receiving Accepts from " << mRvTargetVehicle1
    //     << " and " << mRvTargetVehicle2
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

void McApplication::handleReceivedExecuteEvidenceAsRv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mRvNegotiationCompletionReported ||
            mRvExecuteQueuedOrSent ||
            mCooperatingVehicleType != cooperatingVehicleType::RV) {
        return;
    }

    const auto& snapshot = received.data;
    if (!isExecuteEvidenceForActiveRvRequest(snapshot)) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;
    bool fromMissingTarget1 = false;
    bool fromMissingTarget2 = false;
    if (senderStationId == mRvTargetVehicle1 && !mRvAcceptReceived1) {
        fromMissingTarget1 = true;
    } else if (mRvNumberOfVehicles > 1 &&
            senderStationId == mRvTargetVehicle2 &&
            !mRvAcceptReceived2) {
        fromMissingTarget2 = true;
    } else {
        return;
    }

    mCompletedRvNegotiationRequestId = mRvRequestId;
    mRvNegotiationCompletionReported = true;

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " treated Execute from missing CV as negotiation completion evidence"
        << ": requestId=" << static_cast<int>(mRvRequestId)
        << " cvStation=" << senderStationId
        << " missingAccept1=" << fromMissingTarget1
        << " missingAccept2=" << fromMissingTarget2
        << '\n';
}

void McApplication::handleReceivedRejectAsRv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mPriorityMcmCategory != priorityMcmCategory::HighPriority ||
            !mLaneChangeRequestQueuedOrSent) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Reject)) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mRvRequestId) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;
    const bool fromExpectedCv =
        senderStationId == mRvTargetVehicle1 ||
        (mRvNumberOfVehicles > 1 && senderStationId == mRvTargetVehicle2);
    if (!fromExpectedCv) {
        return;
    }

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    EV_WARN << "[MCM-LC-STATE]"
        << " simTime=" << mEgoContext.now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " event=received-reject"
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " rejectingCv=" << senderStationId
        << " targetCv1=" << mRvTargetVehicle1
        << " targetCv2=" << mRvTargetVehicle2
        << " priority=HighPriority\n";

    applyEmergencyFallbackBrake("rejected-brake", "received-reject", mRvRequestId);
}

void McApplication::handleReceivedExecuteAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV != coordinationProgressCV::AcceptSent) {
        return;
    }

    const auto& snapshot = received.data;
    if (!isNegotiationMessageForActiveRequest(snapshot, mcmSubtype::Execute, mCvRequestId) ||
            snapshot.stationId != mCvRvStationId ||
            !isSnapshotTargetingEgo(snapshot)) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();

    mMcmSubtype = mcmSubtype::Execute;
    mOperationMode = operationMode::ManeuverExecutionMode;
    mCoordinationProgressCV = coordinationProgressCV::SendExecuteCV;

    if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
        EV_INFO << "[MCM-LC-STATE]"
            << " simTime=" << mEgoContext.now
            << " role=target-cv"
            << " station=" << egoStationId
            << " event=received-execute"
            << " requestId=" << static_cast<int>(mCvRequestId)
            << " rvStation=" << mCvRvStationId
            << " priority=HighPriority\n";
    }

    // Store the CV-side negotiated trajectory reference.
    // During execution, the live plannedTrajectory/intent may continue updating;
    // this saved trajectory is used only to detect maneuver completion.
    mActiveNegotiatedTrajectory = mHasCvSelectedTrajectory ?
        mCvSelectedTrajectory : mEgoContext.plannedTrajectory;
    mHasActiveNegotiatedTrajectory = !mActiveNegotiatedTrajectory.empty();

    EV_INFO << "McApplication CV station " << egoStationId
        << " received Execute from RV " << mCvRvStationId
        << ": requestId=" << static_cast<int>(mCvRequestId)
        << " numberOfVehicles=" << snapshot.numberOfVehicles
        << " requestedTrajectoryPoints=" << snapshot.requestedTrajectoryPointCount
        << '\n';
    
    // std::cout << "MCM_DEBUG CV station " << egoStationId
    //     << " received Execute for requestId " << static_cast<int>(mCvRequestId)
    //     << " from RV " << mCvRvStationId
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

void McApplication::handleReceivedEmergencyAsFollower(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleController || !mVehicleDataProvider ||
            mEgoContext.routeId != scSafetyCriticalLaneChangeRouteId ||
            mVehicleController->getVehicleId() == scEmergencyVehicleId) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasExecutionContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Abort) ||
            snapshot.priorityManeuver != static_cast<long>(priorityMcmCategory::EmergencyPriority) ||
            snapshot.plannedTrajectory.empty() || mEgoContext.plannedTrajectory.empty()) {
        return;
    }

    if (mEmergencyReceived) {
        if (!mEmergencyAlreadyArmedLogged) {
            EV_INFO << "[MCM-EMERGENCY]"
                << " simTime=" << mEgoContext.now
                << " role=lane-1-follower"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " event=emergency-mcm-ignored"
                << " emergencyStation=" << snapshot.stationId
                << " reason=already-armed\n";
            mEmergencyAlreadyArmedLogged = true;
        }
        return;
    }

    const auto& emergencyPoint = snapshot.plannedTrajectory.front();
    const auto& egoPoint = mEgoContext.plannedTrajectory.front();
    const int emergencyLane = snapshot.hasLaneId ? static_cast<int>(snapshot.laneId) : -1;
    const int receiverLane = mEgoContext.laneIndex;
    const bool sameLane = snapshot.hasLaneId && emergencyLane == receiverLane;
    const bool emergencyAhead = egoPoint.mY >= emergencyPoint.mY;
    const double dx = egoPoint.mX - emergencyPoint.mX;
    const double dy = egoPoint.mY - emergencyPoint.mY;
    const double distanceGap = std::sqrt(dx * dx + dy * dy);
    const double emergencySpeed =
        snapshot.speedValue >= 0 && snapshot.speedValue < 16382 ?
        static_cast<double>(snapshot.speedValue) / 100.0 : -1.0;
    const double relativeSpeed = emergencySpeed >= 0.0 ? mEgoContext.speed - emergencySpeed : -1.0;
    const double timeGap = mEgoContext.speed > 0.1 ? distanceGap / mEgoContext.speed : -1.0;
    const double ttc = relativeSpeed > 0.1 ? distanceGap / relativeSpeed : -1.0;
    const double reactionWindow = timeGap >= 0.0 ? timeGap - scSafetyCriticalTimeGap : -1.0;

    // Same lane and ahead are the trigger gates for arming the emergency
    // lane-change search. Gap, TTC, and reaction window stay diagnostic here;
    // target-lane trajectory conflict detection decides whether a Request is sent.
    EV_INFO << "[MCM-EMERGENCY]"
        << " simTime=" << mEgoContext.now
        << " role=lane-1-follower"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " event=same-lane-ahead-check"
        << " emergencyStation=" << snapshot.stationId
        << " emergencyLane=" << emergencyLane
        << " receiverLane=" << receiverLane
        << " emergencyX=" << emergencyPoint.mX
        << " emergencyY=" << emergencyPoint.mY
        << " receiverX=" << egoPoint.mX
        << " receiverY=" << egoPoint.mY
        << " emergencySpeed=" << emergencySpeed
        << " receiverSpeed=" << mEgoContext.speed
        << " distanceGap=" << distanceGap
        << " timeGap=" << timeGap
        << " ttc=" << ttc
        << " reactionWindow=" << reactionWindow
        << " sameLane=" << sameLane
        << " emergencyAhead=" << emergencyAhead
        << " reason="
        << (!sameLane ? "not-same-lane" : (!emergencyAhead ? "emergency-not-ahead" : "armed-same-lane-emergency-ahead"))
        << '\n';

    if (!sameLane || !emergencyAhead) {
        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << mEgoContext.now
            << " role=lane-1-follower"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " event=emergency-mcm-ignored"
            << " reason=" << (!sameLane ? "not-same-lane" : "emergency-not-ahead")
            << " emergencyLane=" << emergencyLane
            << " receiverLane=" << receiverLane
            << " egoY=" << egoPoint.mY
            << " emergencyY=" << emergencyPoint.mY
            << '\n';
        return;
    }

    // These old conflict checks are logged for comparison with the thesis
    // behavior, but they are not a hard gate for arming the safety-critical RV.
    const omnetpp::SimTime eteDelay =
        std::max(omnetpp::SimTime::ZERO, mEgoContext.now - received.receivedAt);
    const bool conflictAtMinimumGap = mTrajectoryPlanner.check_traj_conflict(
        mEgoContext.plannedTrajectory,
        snapshot.plannedTrajectory,
        scSafetyCriticalTimeGap,
        eteDelay);
    const bool conflictAtCoordinationGap = mTrajectoryPlanner.check_traj_conflict(
        mEgoContext.plannedTrajectory,
        snapshot.plannedTrajectory,
        scEmergencyCoordinationTimeGap,
        eteDelay);
    EV_INFO << "[MCM-EMERGENCY]"
        << " simTime=" << mEgoContext.now
        << " role=lane-1-follower"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " laneIndex=" << mEgoContext.laneIndex
        << " event=received-emergency-execution-mcm"
        << " emergencyStation=" << snapshot.stationId
        << " subtype=Abort"
        << " priority=EmergencyPriority"
        << " emergencyLane=" << emergencyLane
        << " receiverLane=" << receiverLane
        << " emergencyX=" << emergencyPoint.mX
        << " emergencyY=" << emergencyPoint.mY
        << " receiverX=" << egoPoint.mX
        << " receiverY=" << egoPoint.mY
        << " distanceGap=" << distanceGap
        << " timeGap=" << timeGap
        << " ttc=" << ttc
        << " desiredMinimumTimeGap=" << scSafetyCriticalTimeGap
        << " reactionWindow=" << reactionWindow
        << " paperInitialTimeGap=" << scInitialPaperTimeGap
        << " oldMinimumGapConflict=" << conflictAtMinimumGap
        << " oldCoordinationGap=" << scEmergencyCoordinationTimeGap
        << " oldCoordinationConflict=" << conflictAtCoordinationGap
        << " safetyCritical=1"
        << " reason=armed-same-lane-emergency-ahead"
        << '\n';

    mEmergencyReceived = true;
    mEmergencyStationId = snapshot.stationId;
    mEmergencyDistanceGap = distanceGap;
    mEmergencyTimeGap = timeGap;
    mEmergencyReactionWindow = reactionWindow;
    mCooperatingVehicleType = cooperatingVehicleType::RV;
    mCoordinationProgressRV = coordinationProgressRV::CheckForCoordination;
    mControlManeuver = controlManeuver::ChangeLane;
    mPriorityMcmCategory = priorityMcmCategory::HighPriority;
    mOperationMode = operationMode::ManeuverNegotiationMode;

    EV_INFO << "[MCM-LC-TRIGGER]"
        << " simTime=" << mEgoContext.now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " laneIndex=" << mEgoContext.laneIndex
        << " event=safety-critical-trigger-armed"
        << " emergencyStation=" << mEmergencyStationId
        << " emergencyLane=" << emergencyLane
        << " receiverLane=" << receiverLane
        << " distanceGap=" << mEmergencyDistanceGap
        << " timeGap=" << mEmergencyTimeGap
        << " ttc=" << ttc
        << " reactionWindow=" << mEmergencyReactionWindow
        << " reason=armed-same-lane-emergency-ahead"
        << " priority=HighPriority"
        << " controlManeuver=ChangeLane\n";
}

void McApplication::applyEmergencyFallbackBrake(
    const char* event,
    const char* reason,
    uint8_t requestId)
{
    EV_STATICCONTEXT;

    const double currentSpeed = mHasEgoContext ? mEgoContext.speed : 0.0;
    constexpr double targetSpeed = 0.1;
    constexpr double decelerationTime = 1.0;

    // Reject, negotiation timeout, unsafe front vehicle, and moveToXY failure
    // all use the same RV safety fallback: brake and clear active coordination.
    if (mVehicleController) {
        const std::string& vehicleId = mVehicleController->getVehicleId();
        try {
            mVehicleController->setSpeedMode(vehicleId, 31);
            mVehicleController->slowDown(vehicleId, targetSpeed, decelerationTime);
            mVehicleController->setMaxSpeed(targetSpeed * boost::units::si::meter_per_second);
        } catch (const std::exception& e) {
            EV_WARN << "[MCM-LC-FAILSAFE]"
                << " simTime=" << (mHasEgoContext ? mEgoContext.now : omnetpp::simTime())
                << " event=" << event
                << " requestId=" << static_cast<int>(requestId)
                << " reason=" << reason
                << " brakeApplied=0"
                << " error=\"" << e.what() << "\"\n";
        }
    }

    EV_WARN << "[MCM-LC-FAILSAFE]"
        << " simTime=" << (mHasEgoContext ? mEgoContext.now : omnetpp::simTime())
        << " event=" << event
        << " requestId=" << static_cast<int>(requestId)
        << " reason=" << reason
        << " station=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " currentSpeed=" << currentSpeed
        << " targetSpeed=" << targetSpeed
        << " decelerationTime=" << decelerationTime
        << " speedMode=31\n";

    mPendingMcmCommand.reset();
    mLaneChangeRequestQueuedOrSent = false;
    mLaneChangeThreeVehiclePath = false;
    mRvOfferReceived1 = false;
    mRvOfferReceived2 = false;
    mRvConfirmQueuedOrSent = false;
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mRvNegotiationTimedOut = true;
    mRvLastRequestQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastRequestQueuedAt = false;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasRvNegotiationStartedAt = false;
    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;
    mSafetyCriticalLaneChangeExecutionActive = false;
    mLaneChangeMoveStepCounter = 0;
    mSafetyCriticalLaneChangeExecutionStartedAt = omnetpp::SimTime::ZERO;
    mLastSafetyCriticalLaneChangeMoveAt = omnetpp::SimTime::ZERO;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;
    mCoordinationProgressRV = coordinationProgressRV::NoCoordination;
    mControlManeuver = controlManeuver::EmergencyDeceleration;
}

void McApplication::queueRepeatedExecute()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            !mHasActiveNegotiatedTrajectory) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
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

void McApplication::evaluateRvExecutionProgress()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            !mHasActiveNegotiatedTrajectory) {
        return;
    }

    if (!hasReachedActiveNegotiatedTrajectoryEnd()) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Cancel;
    // TODO: temporary Complete workaround.
    // The legacy ASN.1/model used Cancel here because Complete was not available.
    // Semantically this means: maneuver execution completed after reaching/passing
    // the final point of the saved negotiated trajectory.
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mPendingMcmCommand = command;
    mMcmSubtype = mcmSubtype::Cancel;
    mCoordinationProgressRV = coordinationProgressRV::SendComplete;
    sampleMergingGapDiagnostics("rv-completion-queued");

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " queued completion workaround after passing negotiated trajectory end"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " currentY=" << mEgoContext.y
        << " finalY=" << mActiveNegotiatedTrajectory.back().mY
        << '\n';

    // std::cout << "MCM_DEBUG RV station " << mEgoContext.stationId
    //     << " queued Complete workaround using Cancel for requestId "
    //     << static_cast<int>(command.requestId)
    //     << " currentY=" << mEgoContext.y
    //     << " finalY=" << mActiveNegotiatedTrajectory.back().mY
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

void McApplication::evaluateCvExecutionProgress()
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider || mPendingMcmCommand ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV != coordinationProgressCV::SendExecuteCV ||
            !mHasActiveNegotiatedTrajectory) {
        return;
    }

    if (!hasReachedActiveNegotiatedTrajectoryEnd()) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Cancel;
    // TODO: temporary Complete workaround.
    // The legacy ASN.1/model used Cancel here because Complete was not available.
    // Semantically this means: CV maneuver execution completed after reaching/passing
    // the final point of the saved negotiated trajectory.
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mCvRequestId;
    command.numberOfVehicles = 1;
    command.targetVehicle1 = mCvRvStationId;
    command.hasTargetVehicle2 = false;
    command.targetVehicle2 = 0;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mPendingMcmCommand = command;
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
        << " finalX=" << mActiveNegotiatedTrajectory.back().mX
        << " finalY=" << mActiveNegotiatedTrajectory.back().mY
        << '\n';

    EV_INFO << "McApplication CV station " << mEgoContext.stationId
        << " queued completion workaround after passing negotiated trajectory end"
        << ": requestId=" << static_cast<int>(command.requestId)
        << " rvStation=" << command.targetVehicle1
        << " currentY=" << mEgoContext.y
        << " finalY=" << mActiveNegotiatedTrajectory.back().mY
        << '\n';

    // std::cout << "MCM_DEBUG CV station " << mEgoContext.stationId
    //     << " queued Complete workaround using Cancel for requestId "
    //     << static_cast<int>(command.requestId)
    //     << " currentY=" << mEgoContext.y
    //     << " finalY=" << mActiveNegotiatedTrajectory.back().mY
    //     << " at " << omnetpp::simTime() << " s" << std::endl;
}

bool McApplication::hasReachedActiveNegotiatedTrajectoryEnd() const
{
    if (!mHasEgoContext || !mHasActiveNegotiatedTrajectory ||
            mActiveNegotiatedTrajectory.empty()) {
        return false;
    }

    const auto& firstPoint = mActiveNegotiatedTrajectory.front();
    const auto& lastPoint = mActiveNegotiatedTrajectory.back();

    if (mEgoContext.routeId == scMergingRouteId) {
        // Old route_merging_1 RV behavior:
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

void McApplication::sampleMergingGapDiagnostics(const char* phase)
{
    EV_STATICCONTEXT;

    if (!mMergingGapDiagActive || !mHasEgoContext || !mVehicleController ||
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mOperationMode != operationMode::ManeuverExecutionMode ||
            mCoordinationProgressRV != coordinationProgressRV::SendExecute ||
            mEgoContext.routeId != scMergingRouteId) {
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

void McApplication::logMergingGapSummary(omnetpp::SimTime completionTime) const
{
    EV_STATICCONTEXT;

    if (!mMergingGapDiagHasMinDistance) {
        EV_INFO << "[MCM-GAP-DIAG]"
            << " summary=rv-completion"
            << " rvStation=" << (mHasEgoContext ? mEgoContext.stationId : 0)
            << " route=" << scMergingRouteId
            << " executionStart=" << mMergingGapDiagExecutionStart
            << " completionTime=" << completionTime
            << " targetCvStation=" << mMergingGapDiagTargetCvStationId
            << " result=no-gap-samples\n";
        return;
    }

    EV_INFO << "[MCM-GAP-DIAG]"
        << " summary=rv-completion"
        << " rvStation=" << (mHasEgoContext ? mEgoContext.stationId : 0)
        << " route=" << scMergingRouteId
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

uint8_t McApplication::makeRequestId(omnetpp::SimTime now) const
{
    const auto millis = static_cast<uint64_t>(now.inUnit(omnetpp::SIMTIME_MS));
    return static_cast<uint8_t>((mEgoContext.stationId + millis + mReceivedMcmCount) % 256);
}

}  // namespace mcm
}  // namespace artery
