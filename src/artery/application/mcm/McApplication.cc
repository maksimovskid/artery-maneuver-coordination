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
#include <utility>

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

void McApplication::setSecondRequestSmokeReject(bool enabled, uint32_t stationId)
{
    mSecondRequestSmokeRejectEnabled = enabled;
    mSecondRequestSmokeRejectStationId = stationId;
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
    handleReceivedRejectAsCv(mcm);
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
            (mCoordinationProgressRV == coordinationProgressRV::CoordinationRequired ||
                mCoordinationProgressRV == coordinationProgressRV::SecondRequest)) {
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
                    << " event=rv-speedmode-enabled-after-completion"
                    << " vehicleId=" << vehicleId
                    << " station=" << mEgoContext.stationId
                    << " requestId=" << static_cast<int>(mRvRequestId)
                    << " speedMode=31"
                    << " normalSpeedEnabled=" << restoreNormalSpeed
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
            mCvResponseQueuedOrSent = true;
        } else {
            mCvHasRejectedRequest = true;
            mCvRejectedRvStationId = mCvRvStationId;
            mCvRejectedRequestId = mCvRequestId;
            mCoordinationProgressCV = coordinationProgressCV::NoRequest;
            mOperationMode = operationMode::IntentionSharingMode;
            mMcmSubtype = mcmSubtype::Regular;
            mCooperatingVehicleType = cooperatingVehicleType::NCV;
            mControlManeuver = controlManeuver::DoNothing;
            mCvSelectedTrajectory.clear();
            mHasCvSelectedTrajectory = false;
            mCvRvStationId = 0;
            mCvRequestId = 0;
            mCvResponseNumberOfVehicles = 1;
            mCvLastOfferQueuedAt = omnetpp::SimTime::ZERO;
            mHasCvLastOfferQueuedAt = false;
            mCvLastAcceptQueuedAt = omnetpp::SimTime::ZERO;
            mHasCvLastAcceptQueuedAt = false;
            mCvNegotiationStartedAt = omnetpp::SimTime::ZERO;
            mHasCvNegotiationStartedAt = false;
            mCvResponseQueuedOrSent = false;
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

std::vector<PlannerMeasurement> McApplication::consumePlannerMeasurements()
{
    auto measurements = std::move(mPendingPlannerMeasurements);
    mPendingPlannerMeasurements.clear();
    return measurements;
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

void McApplication::evaluateEmergencyBrakingTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    // Scenario-only emergency source for the high-priority lane-change
    // validation. car_hl0_Emergency brakes at a configured time and
    // broadcasts EmergencyPriority execution
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

    // Emergency signaling is represented as an execution-container
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
        // model, so the Abort + EmergencyPriority
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
    mRvSecondRequestAttempted = false;
    mRvSecondRequestCompletedMeasured = false;
    mRvSecondRequestRejectedMeasured = false;
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

    const uint8_t incomingRequestId = snapshot.requestId >= 0 ? static_cast<uint8_t>(snapshot.requestId) : 0;
    if (mCvHasRejectedRequest &&
            snapshot.stationId == mCvRejectedRvStationId &&
            incomingRequestId == mCvRejectedRequestId) {
        EV_INFO << "[MCM-CV-DECISION]"
            << " simTime=" << (mHasEgoContext ? mEgoContext.now : omnetpp::simTime())
            << " event=ignore-rejected-request-retry"
            << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " cvStation=" << egoStationId
            << " rvStation=" << snapshot.stationId
            << " requestId=" << static_cast<int>(incomingRequestId)
            << '\n';
        return;
    }
    if (mCvHasRejectedRequest && snapshot.stationId == mCvRejectedRvStationId) {
        mCvHasRejectedRequest = false;
        mCvRejectedRvStationId = 0;
        mCvRejectedRequestId = 0;
    }

    logNegotiationTrace("RECV", snapshot, received.receivedAt);

    mCvRvStationId = snapshot.stationId;
    mCvRequestId = incomingRequestId;
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

    const bool forceSmokeReject =
        mSecondRequestSmokeRejectEnabled &&
        !mSecondRequestSmokeRejectConsumed &&
        mPriorityMcmCategory == priorityMcmCategory::HighPriority &&
        mHasEgoContext &&
        mEgoContext.routeId == scTargetLaneChangeRouteId &&
        egoStationId == mSecondRequestSmokeRejectStationId;

    if (forceSmokeReject) {
        mSecondRequestSmokeRejectConsumed = true;
        command.subtype = mcmSubtype::Reject;
        command.requestedTrajectory = mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};
        command.offeredTrajectory.clear();
        command.hasOfferedTrajectory = false;
        mCoordinationProgressCV = coordinationProgressCV::SendReject;

        EV_WARN << "[MCM-SECOND-REQUEST-SMOKE]"
            << " simTime=" << mEgoContext.now
            << " event=force-first-cv-reject"
            << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
            << " cvStation=" << egoStationId
            << " rvStation=" << command.targetVehicle1
            << " requestId=" << static_cast<int>(command.requestId)
            << " priority=HighPriority"
            << " numberOfVehicles=" << snapshot.numberOfVehicles
            << '\n';
    } else if (snapshot.requestedTrajectory.empty()) {
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
            << " matches two-CV Offer path"
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
            << " matches one-CV Accept path"
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
    // before Execute. During execution the current ASN.1 workaround uses Cancel as the
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
        << " hadDecelerationControl=" << hadDecelerationControl
        << " hadAccelerationControl=" << hadAccelerationControl
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
    if (mRvSecondRequestAttempted && !mRvSecondRequestCompletedMeasured) {
        enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestCompletedCounter);
        mRvSecondRequestCompletedMeasured = true;
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
    if (mRvSecondRequestAttempted && !mRvSecondRequestCompletedMeasured) {
        enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestCompletedCounter);
        mRvSecondRequestCompletedMeasured = true;
    }

    EV_INFO << "McApplication RV station " << mEgoContext.stationId
        << " treated Execute from missing CV as negotiation completion evidence"
        << ": requestId=" << static_cast<int>(mRvRequestId)
        << " cvStation=" << senderStationId
        << " missingAccept1=" << fromMissingTarget1
        << " missingAccept2=" << fromMissingTarget2
        << '\n';
}

void McApplication::handleReceivedRejectAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV != coordinationProgressCV::SendOffer ||
            !mCvResponseQueuedOrSent) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Reject) ||
            snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mCvRequestId) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    if (snapshot.stationId == egoStationId ||
            snapshot.negotiationVehicleId1 != mCvRvStationId) {
        return;
    }

    EV_INFO << "[MCM-CV-DECISION]"
        << " simTime=" << (mHasEgoContext ? mEgoContext.now : omnetpp::simTime())
        << " event=peer-reject-clears-pending-offer"
        << " cvVehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
        << " cvStation=" << egoStationId
        << " rvStation=" << mCvRvStationId
        << " rejectingCv=" << snapshot.stationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << '\n';

    resetCvNegotiationAfterTimeout();
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

    const bool waitingForInitialRequestResponse =
        mCoordinationProgressRV == coordinationProgressRV::CoordinationRequired ||
        mCoordinationProgressRV == coordinationProgressRV::RequestSent;
    if (!mPendingMcmCommand && !mRvSecondRequestAttempted && waitingForInitialRequestResponse) {
        auto secondRequest = makeRvSecondRequestCommand(received);
        if (secondRequest) {
            const uint8_t previousRequestId = mRvRequestId;

            mPendingMcmCommand = *secondRequest;
            mRvSecondRequestAttempted = true;
            mRvSecondRequestCompletedMeasured = false;
            mRvSecondRequestRejectedMeasured = false;
            enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestStartedCounter);
            if (secondRequest->hasTrajectoryCostRv) {
                enqueuePlannerMeasurement(
                    PlannerMeasurementMetric::TrajectoryCostRV,
                    secondRequest->trajectoryCostRv);
            }
            mRvRequestId = secondRequest->requestId;
            mRvNumberOfVehicles = secondRequest->numberOfVehicles;
            mRvTargetVehicle1 = secondRequest->targetVehicle1;
            mRvTargetVehicle2 = secondRequest->hasTargetVehicle2 ? secondRequest->targetVehicle2 : 0;
            mRvOfferReceived1 = false;
            mRvOfferReceived2 = false;
            mRvConfirmQueuedOrSent = false;
            mRvAcceptReceived1 = false;
            mRvAcceptReceived2 = false;
            mRvExecuteQueuedOrSent = false;
            mRvNegotiationCompletionReported = false;
            mCompletedRvNegotiationRequestId.reset();
            mRvLastRequestQueuedAt = received.receivedAt;
            mHasRvLastRequestQueuedAt = true;
            mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
            mHasRvLastConfirmQueuedAt = false;
            mRvNegotiationStartedAt = received.receivedAt;
            mHasRvNegotiationStartedAt = true;
            mActiveNegotiatedTrajectory = secondRequest->requestedTrajectory;
            mHasActiveNegotiatedTrajectory = !mActiveNegotiatedTrajectory.empty();
            mMcmSubtype = mcmSubtype::Request;
            mOperationMode = operationMode::ManeuverNegotiationMode;
            mCoordinationProgressRV = coordinationProgressRV::SecondRequest;

            EV_INFO << "[MCM-LC-STATE]"
                << " simTime=" << mEgoContext.now
                << " role=safety-critical-lane-change-rv"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << (mVehicleController ? mVehicleController->getVehicleId() : "")
                << " event=second-request-ready"
                << " previousRequestId=" << static_cast<int>(previousRequestId)
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " rejectingCv=" << senderStationId
                << " targetCv1=" << mRvTargetVehicle1
                << " targetCv2=" << mRvTargetVehicle2
                << '\n';
            return;
        }
        if (!mRvSecondRequestRejectedMeasured) {
            enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestRejectedCounter);
            mRvSecondRequestRejectedMeasured = true;
        }
    } else if (mRvSecondRequestAttempted && !mRvSecondRequestRejectedMeasured) {
        enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestRejectedCounter);
        mRvSecondRequestRejectedMeasured = true;
    }

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

    // These conflict checks are logged as diagnostics, but they are not a hard
    // gate for arming the safety-critical RV.
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
        << " minimumGapConflict=" << conflictAtMinimumGap
        << " coordinationGap=" << scEmergencyCoordinationTimeGap
        << " coordinationConflict=" << conflictAtCoordinationGap
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
    if (mRvSecondRequestAttempted &&
            !mRvSecondRequestCompletedMeasured &&
            !mRvSecondRequestRejectedMeasured) {
        enqueuePlannerMeasurement(PlannerMeasurementMetric::SecondRequestRejectedCounter);
        mRvSecondRequestRejectedMeasured = true;
    }
    mRvSecondRequestAttempted = false;
    mRvSecondRequestCompletedMeasured = false;
    mRvSecondRequestRejectedMeasured = false;
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

uint8_t McApplication::makeRequestId(omnetpp::SimTime now) const
{
    const auto millis = static_cast<uint64_t>(now.inUnit(omnetpp::SIMTIME_MS));
    return static_cast<uint8_t>((mEgoContext.stationId + millis + mReceivedMcmCount) % 256);
}

}  // namespace mcm
}  // namespace artery
