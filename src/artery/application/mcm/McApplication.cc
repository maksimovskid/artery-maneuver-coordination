#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
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

/*
 * Implements the service-facing core of McApplication.
 *
 * This translation unit contains lifecycle setup, command and message dispatch,
 * sent-message tracking, negotiation trace logging, and the remaining protocol
 * handlers that have not been moved to more focused files. All definitions are
 * member functions of the single McApplication class declared in
 * McApplication.h and operate on the same shared application state.
 *
 * The source split is organizational only; it does not create a separate
 * runtime component or independent protocol state owner.
 */

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

void McApplication::setEmergencyBrakingOnlyBaseline(bool enabled)
{
    mEmergencyBrakingOnlyBaseline = enabled;
}

void McApplication::updateEgoContext(const McEgoContext& context)
{
    mEgoContext = context;
    mHasEgoContext = true;
}

/*
 * Runs one application update in the fixed order expected by the current
 * protocol implementation. Trigger evaluation, retry handling, execution
 * control, diagnostics, and completion checks all share McApplication state, so
 * reordering this sequence can change observable message timing.
 */
void McApplication::tick(omnetpp::SimTime now)
{
    logScenarioVehicleLifetime(now);
    evaluateEmergencyBrakingTrigger(now);
    if (mEmergencyBrakingOnlyBaseline) {
        return;
    }
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

/*
 * Prepares generation-time state before McService serializes the next MCM. RVs
 * in execution mode may enqueue a repeated Execute here so the generated
 * message reflects the current live planned trajectory.
 */
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

/*
 * Records the received snapshot and dispatches it to all protocol handlers.
 * The handler order is part of the current behavior because each handler may
 * inspect or update shared negotiation and pending-command state.
 */
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

bool McApplication::isRvExecutionCompletionWorkaround(const SentMcm& mcm) const
{
    return mcm.data.hasNegotiationContainer &&
        mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Cancel) &&
        mCooperatingVehicleType == cooperatingVehicleType::RV &&
        mOperationMode == operationMode::ManeuverExecutionMode &&
        mCoordinationProgressRV == coordinationProgressRV::SendComplete &&
        mcm.data.requestId == mRvRequestId;
}

bool McApplication::isCvExecutionCompletionWorkaround(const SentMcm& mcm) const
{
    return mcm.data.hasNegotiationContainer &&
        mcm.data.mcmCategory == static_cast<long>(mcmSubtype::Cancel) &&
        mCooperatingVehicleType == cooperatingVehicleType::CV &&
        mOperationMode == operationMode::ManeuverExecutionMode &&
        mCoordinationProgressCV == coordinationProgressCV::SendCompleteCV &&
        mcm.data.requestId == mCvRequestId &&
        mcm.data.negotiationVehicleId1 == mCvRvStationId;
}

/*
 * Tracks messages produced by this application and advances local negotiation
 * progress after McService has actually sent them. This keeps queued-command
 * creation separate from sent-message evidence.
 */
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
    } else if (isRvExecutionCompletionWorkaround(mcm)) {
        EV_INFO << "[MCM-WIRE]"
            << " direction=sent"
            << " event=execution-completion-workaround-confirmed"
            << " role=RV"
            << " station=" << mEgoContext.stationId
            << " requestId=" << mcm.data.requestId
            << '\n';
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
    } else if (isCvExecutionCompletionWorkaround(mcm)) {
        EV_INFO << "[MCM-WIRE]"
            << " direction=sent"
            << " event=execution-completion-workaround-confirmed"
            << " role=CV"
            << " station=" << mEgoContext.stationId
            << " requestId=" << mcm.data.requestId
            << '\n';
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

/*
 * Emits the compact protocol trace used to compare Request, Offer, Confirm,
 * Accept, Reject, Cancel, Execute, and emergency message sequencing across RV
 * and CV roles.
 */
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

/*
 * Handles an incoming Request from the CV side. The method evaluates the
 * requested trajectory, chooses the cooperative maneuver, records planner
 * measurements, and queues either Offer, Accept, or Reject while preserving the
 * one-CV and two-CV response paths.
 */
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

/*
 * Rolls back CV-side negotiation state when a pre-execution Cancel arrives
 * from the active RV. Execution-phase Cancel is reserved for the current
 * Complete workaround and is intentionally ignored here.
 */
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

/*
 * RV-side Offer handling for one-CV and two-CV coordination. It records which
 * expected CVs have responded and queues Confirm only after the required Offer
 * set has arrived.
 */
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
    if (!classifyExpectedRvResponseSender(senderStationId, fromTarget1, fromTarget2)) {
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

/*
 * CV-side Confirm handling. Once the active RV confirms the offered trajectory,
 * this builds and queues the Accept that transitions the CV toward execution.
 */
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

/*
 * RV-side Accept handling. The method supports both the direct one-CV
 * Request-to-Accept path and the two-CV Confirm-to-Accept path, then queues the
 * Execute command only after all expected Accepts have been observed.
 */
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
    if (!classifyExpectedRvResponseSender(senderStationId, fromTarget1, fromTarget2)) {
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
    command.kind = PendingMcmCommand::Kind::Execution;

    mRvNegotiatedTrajectory = mRvRequestedTrajectory;
    mHasRvNegotiatedTrajectory = !mRvNegotiatedTrajectory.empty();
    EV_INFO << "[MCM-TRAJECTORY]"
        << " event=rv-negotiated-trajectory-established"
        << " station=" << mEgoContext.stationId
        << " requestId=" << static_cast<int>(mRvRequestId)
        << " trajectoryRole=negotiated"
        << " trajectoryPoints=" << mRvNegotiatedTrajectory.size()
        << '\n';

    mPendingMcmCommand = command;
    EV_INFO << "[MCM-WIRE]"
        << " direction=queued"
        << " station=" << mEgoContext.stationId
        << " subtype=Execute"
        << " kind=Execution"
        << " container=Execution"
        << " origin=initial-execute"
        << " requestId=-1"
        << " cooperationId=" << static_cast<int>(command.requestId)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " hasTarget2=" << command.hasTargetVehicle2
        << " priority=" << priorityName(static_cast<long>(command.priority))
        << '\n';
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

/*
 * Treats an Execute from a missing CV as completion evidence for the RV when a
 * duplicate or reordered message sequence prevented the matching Accept from
 * being recorded.
 */
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

/*
 * Clears a CV's pending Offer when another target CV rejects the same active
 * Request. This keeps the CV-side state from retrying an Offer after the RV has
 * already lost the multi-vehicle negotiation.
 */
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

/*
 * Handles a high-priority RV-side Reject. The first eligible Reject can build a
 * second Request with a different target set; later or unrecoverable Rejects
 * fall back to emergency braking.
 */
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
            mRvRequestedTrajectory = secondRequest->requestedTrajectory;
            mHasRvRequestedTrajectory = !mRvRequestedTrajectory.empty();
            mRvNegotiatedTrajectory.clear();
            mHasRvNegotiatedTrajectory = false;
            EV_INFO << "[MCM-TRAJECTORY]"
                << " event=rv-requested-trajectory-active"
                << " station=" << mEgoContext.stationId
                << " requestId=" << static_cast<int>(mRvRequestId)
                << " trajectoryRole=requested"
                << " source=second-request"
                << " trajectoryPoints=" << mRvRequestedTrajectory.size()
                << '\n';
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

/*
 * CV-side Execute handling. It arms execution after the active RV sends Execute
 * for the accepted Request, preserving selected trajectory and control
 * maneuver state chosen during the CV decision phase.
 */
void McApplication::handleReceivedExecuteAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::CV) {
        return;
    }

    const auto& snapshot = received.data;
    if (snapshot.mcmCategory != static_cast<long>(mcmSubtype::Execute) ||
            (!snapshot.hasNegotiationContainer && !snapshot.hasExecutionContainer)) {
        return;
    }

    const bool usesExecutionContainer = snapshot.hasExecutionContainer;
    const char* containerName = usesExecutionContainer ? "Execution" : "Negotiation";
    const long identity = usesExecutionContainer ? snapshot.cooperationId : snapshot.requestId;
    const uint32_t target1 = usesExecutionContainer ?
        snapshot.cooperationVehicleId1 : snapshot.negotiationVehicleId1;
    const bool hasTarget2 = usesExecutionContainer ?
        snapshot.hasCooperationVehicleId2 : snapshot.hasNegotiationVehicleId2;
    const uint32_t target2 = usesExecutionContainer ?
        snapshot.cooperationVehicleId2 : snapshot.negotiationVehicleId2;
    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    const bool eligibleState = mCoordinationProgressCV == coordinationProgressCV::AcceptSent;
    const bool identityMatches = identity >= 0 && static_cast<uint8_t>(identity) == mCvRequestId;
    const bool senderMatches = snapshot.stationId == mCvRvStationId;
    const bool participantMatches = target1 == egoStationId ||
        (hasTarget2 && target2 == egoStationId);
    const bool guardsPassed = eligibleState && identityMatches && senderMatches && participantMatches;
    const char* rejectionReason = "none";
    if (!eligibleState) {
        rejectionReason = "invalid-cv-state";
    } else if (!identityMatches) {
        rejectionReason = "identity-mismatch";
    } else if (!senderMatches) {
        rejectionReason = "sender-mismatch";
    } else if (!participantMatches) {
        rejectionReason = "missing-ego-participant";
    }

    EV_INFO << "[MCM-WIRE]"
        << " direction=received"
        << " event=execute-guard"
        << " station=" << mEgoContext.stationId
        << " sender=" << snapshot.stationId
        << " subtype=Execute"
        << " container=" << containerName
        << " requestId=" << snapshot.requestId
        << " cooperationId=" << snapshot.cooperationId
        << " target1=" << target1
        << " target2=" << target2
        << " hasTarget2=" << hasTarget2
        << " eligibleState=" << eligibleState
        << " activeRequestMatch=" << identityMatches
        << " senderMatch=" << senderMatches
        << " participantMatch=" << participantMatches
        << " result=" << (guardsPassed ? "pass" : "reject")
        << " reason=" << rejectionReason
        << '\n';
    if (!guardsPassed) {
        return;
    }

    mMcmSubtype = mcmSubtype::Execute;
    mOperationMode = operationMode::ManeuverExecutionMode;
    mCoordinationProgressCV = coordinationProgressCV::SendExecuteCV;

    EV_INFO << "[MCM-WIRE]"
        << " direction=received"
        << " event=cv-execution-armed"
        << " station=" << egoStationId
        << " sender=" << snapshot.stationId
        << " subtype=Execute"
        << " container=" << containerName
        << " requestId=" << snapshot.requestId
        << " cooperationId=" << snapshot.cooperationId
        << " target1=" << target1
        << " target2=" << target2
        << " hasTarget2=" << hasTarget2
        << " mode=ManeuverExecutionMode"
        << '\n';

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
    mCvNegotiatedTrajectory = mHasCvSelectedTrajectory ?
        mCvSelectedTrajectory : mEgoContext.plannedTrajectory;
    mHasCvNegotiatedTrajectory = !mCvNegotiatedTrajectory.empty();
    EV_INFO << "[MCM-TRAJECTORY]"
        << " event=cv-negotiated-trajectory-established"
        << " station=" << mEgoContext.stationId
        << " requestId=" << static_cast<int>(mCvRequestId)
        << " trajectoryRole=negotiated"
        << " trajectoryPoints=" << mCvNegotiatedTrajectory.size()
        << '\n';

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

uint8_t McApplication::makeRequestId(omnetpp::SimTime now) const
{
    const auto millis = static_cast<uint64_t>(now.inUnit(omnetpp::SIMTIME_MS));
    return static_cast<uint8_t>((mEgoContext.stationId + millis + mReceivedMcmCount) % 256);
}

}  // namespace mcm
}  // namespace artery
