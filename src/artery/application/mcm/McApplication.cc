#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/traci/VehicleController.h"

#include <algorithm>
#include <cmath>
// #include <iostream> // temporary MCM_DEBUG prints

namespace artery
{
namespace mcm
{

namespace
{
constexpr const char* scMergingRouteId = "route_merging_1";
constexpr const char* scEmergencyVehicleId = "car_hl0_Emergency";
constexpr const char* scSafetyCriticalLaneChangeRouteId = "route_highway_1";
constexpr const char* scTargetLaneChangeRouteId = "route_highway_2";
constexpr double scEmergencyStartTime = 12.0;
constexpr double scEmergencyMaxSpeed = 0.1;
constexpr double scNormalHighwaySpeed = 27.77;
constexpr double scLaneChangeShiftX = 3.0;
constexpr double scSafetyCriticalTimeGap = 1.0;
constexpr double scEmergencyCoordinationTimeGap = 1.5;
constexpr double scInitialPaperTimeGap = 1.22;
constexpr double scMergeStartX = 216554.0;
constexpr double scMergeStartY = 452461.0;
constexpr double scHighwayLane0MinY = 452474.0;
constexpr double scHighwayLane0MaxY = 452574.0;
constexpr double scMergingTimeGap = 1.2;
constexpr int scRequestTrajectorySteps = 20;
constexpr double scRequestTrajectoryDt = 0.25;
constexpr std::size_t scMaxReceivedMcmCache = 256;

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

bool isHighwayMergingCvRoute(const std::string& routeId)
{
    return routeId == "route_highway_0" ||
        routeId == "route_highway_1" ||
        routeId == "route_highway_2";
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
}

void McApplication::initialize(
    traci::VehicleController* controller,
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
    logScenarioVehicleLifetime(now);
    evaluateEmergencyBrakingTrigger(now);
    evaluateMergingRequestTrigger(now);
    evaluateSafetyCriticalLaneChangeTrigger(now);
    applyRvExecutionControl();
    sampleMergingGapDiagnostics("tick");
    applyCvDecelerationControl();
    applyCvAccelerationControl();
    applyCvLaneChangeControl();
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
    handleReceivedExecuteAsCv(mcm);
    handleReceivedEmergencyAsFollower(mcm);
}

void McApplication::handleSentMcm(const SentMcm& mcm)
{
    EV_STATICCONTEXT;

    ++mSentMcmCount;
    mLastSentMcm = mcm;
    mHasLastSentMcm = true;

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

        mCoordinationProgressRV = coordinationProgressRV::CompleteSent;
        mOperationMode = operationMode::IntentionSharingMode;
        mMcmSubtype = mcmSubtype::Regular;
        mCooperatingVehicleType = cooperatingVehicleType::NCV;

        mMergingRequestQueuedOrSent = false;
        mRvOfferReceived1 = false;
        mRvOfferReceived2 = false;
        mRvConfirmQueuedOrSent = false;
        mRvAcceptReceived1 = false;
        mRvAcceptReceived2 = false;
        mRvExecuteQueuedOrSent = false;
        mActiveNegotiatedTrajectory.clear();
        mHasActiveNegotiatedTrajectory = false;
        mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
        mHasLastExecuteQueuedAt = false;
        mRvMergingExecutionControlLogged = false;
        resetMergingGapDiagnostics();

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

        mCoordinationProgressCV = coordinationProgressCV::CompleteSentCV;
        mOperationMode = operationMode::IntentionSharingMode;
        mMcmSubtype = mcmSubtype::Regular;
        mCooperatingVehicleType = cooperatingVehicleType::NCV;

        mCvResponseQueuedOrSent = false;
        mCvRvStationId = 0;
        mCvRequestId = 0;
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

    EV_INFO << "McApplication applied highway-merging CV deceleration control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << " speedMode=31 targetSpeed=" << mTargetSpeed
        << " decelerationTime=" << mCommandDuration
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

    // Old highway CV acceleration uses speedMode 31 so SUMO safety checks stay
    // active while raising the speed toward 120 km/h.
    mVehicleController->setSpeedMode(vehicleId, 31);
    mVehicleController->setSpeed(33.33 * boost::units::si::meter_per_second);
    mVehicleController->setMaxSpeed(33.33 * boost::units::si::meter_per_second);
    mCvAccelerationControlApplied = true;

    EV_INFO << "McApplication applied highway-merging CV acceleration control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
        << " timing=" << (oldEquivalentAcceptPhase ? "SendAccept" : "SendExecuteCV")
        << " speedMode=31 targetSpeed=33.33 maxSpeed=33.33\n";
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
    mCvLaneChangeControlLogged = true;
}

void McApplication::restoreCvSpeedControl()
{
    EV_STATICCONTEXT;

    if (!mVehicleController || !mHasEgoContext ||
            (!mCvDecelerationControlApplied && !mCvAccelerationControlApplied)) {
        return;
    }

    const std::string& vehicleId = mVehicleController->getVehicleId();
    const bool restoredAfterAcceleration = mCvAccelerationControlApplied;
    const bool restoredAfterDeceleration = mCvDecelerationControlApplied;

    // Legacy highway restore uses 27.77 m/s as the normal mainline speed.
    // Deceleration completion only restored maxSpeed; acceleration changed both
    // speed and maxSpeed, so restore both in that case.
    mVehicleController->setSpeedMode(vehicleId, 31);
    if (restoredAfterAcceleration) {
        mVehicleController->setSpeed(27.77 * boost::units::si::meter_per_second);
    }
    mVehicleController->setMaxSpeed(27.77 * boost::units::si::meter_per_second);

    EV_INFO << "McApplication restored highway-merging CV speed control"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << vehicleId
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
    mRvAcceptReceived1 = false;
    mRvAcceptReceived2 = false;
    mRvExecuteQueuedOrSent = false;
    mActiveNegotiatedTrajectory.clear();
    mHasActiveNegotiatedTrajectory = false;
    mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
    mHasLastExecuteQueuedAt = false;
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

void McApplication::evaluateEmergencyBrakingTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

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

    if (!mPendingMcmCommand && !mEmergencyMcmQueued) {
        PendingMcmCommand command;
        command.kind = PendingMcmCommand::Kind::Execution;
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
            << " scheduledStartTime=" << scEmergencyStartTime
            << " subtype=Abort"
            << " priority=EmergencyPriority"
            << " executionContainer=1"
            << " plannedTrajectoryPoints=" << mEgoContext.plannedTrajectory.size()
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
        command.subtype = snapshot.numberOfVehicles > 1 ? mcmSubtype::Offer : mcmSubtype::Accept;
        command.requestedTrajectory = snapshot.requestedTrajectory;
        command.offeredTrajectory = !mEgoContext.plannedTrajectory.empty() ?
            mEgoContext.plannedTrajectory : snapshot.requestedTrajectory;
        command.hasOfferedTrajectory = !command.offeredTrajectory.empty();
        mCoordinationProgressCV = snapshot.numberOfVehicles > 1 ?
            coordinationProgressCV::SendOffer : coordinationProgressCV::SendAccept;

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
            << " requestedTrajectoryPoints=" << snapshot.requestedTrajectory.size()
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
    mCoordinationProgressCV = coordinationProgressCV::NoRequest;
    mOperationMode = operationMode::IntentionSharingMode;
    mMcmSubtype = mcmSubtype::Regular;
    mCooperatingVehicleType = cooperatingVehicleType::NCV;
    mCvResponseQueuedOrSent = false;
    mCvRvStationId = 0;
    mCvRequestId = 0;
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
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Offer)) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mRvRequestId) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;

    if (senderStationId == mRvTargetVehicle1 && !mRvOfferReceived1) {
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
    } else if (mRvNumberOfVehicles > 1 &&
            senderStationId == mRvTargetVehicle2 && !mRvOfferReceived2) {
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

    const bool allExpectedOffersReceived =
        mRvNumberOfVehicles > 1 ?
        (mRvOfferReceived1 && mRvOfferReceived2) :
        mRvOfferReceived1;

    if (!allExpectedOffersReceived) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Confirm;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = 0;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mPendingMcmCommand = command;
    mRvConfirmQueuedOrSent = true;
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
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Confirm)) {
        return;
    }

    if (snapshot.stationId != mCvRvStationId) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mCvRequestId) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    const bool targetsEgo = snapshot.negotiationVehicleId1 == egoStationId ||
        (snapshot.hasNegotiationVehicleId2 && snapshot.negotiationVehicleId2 == egoStationId);
    if (!targetsEgo) {
        return;
    }

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
    command.requestedTrajectory = mHasEgoContext ? mEgoContext.plannedTrajectory : TrajectoryPlanner::Trajectory {};

    mPendingMcmCommand = command;
    mMcmSubtype = mcmSubtype::Accept;
    mCoordinationProgressCV = coordinationProgressCV::SendAccept;

    if (mPriorityMcmCategory == priorityMcmCategory::HighPriority) {
        EV_INFO << "[MCM-LC-STATE]"
            << " simTime=" << mEgoContext.now
            << " role=target-cv"
            << " station=" << egoStationId
            << " event=queued-accept-after-confirm"
            << " requestId=" << static_cast<int>(command.requestId)
            << " rvStation=" << command.targetVehicle1
            << " priority=HighPriority\n";
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
            mCooperatingVehicleType != cooperatingVehicleType::RV ||
            mCoordinationProgressRV != coordinationProgressRV::SendConfirm) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Accept)) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mRvRequestId) {
        return;
    }

    const uint32_t senderStationId = snapshot.stationId;

    if (senderStationId == mRvTargetVehicle1 && !mRvAcceptReceived1) {
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
    } else if (mRvNumberOfVehicles > 1 &&
            senderStationId == mRvTargetVehicle2 && !mRvAcceptReceived2) {
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

    const bool allExpectedAcceptsReceived =
        mRvNumberOfVehicles > 1 ?
        (mRvAcceptReceived1 && mRvAcceptReceived2) :
        mRvAcceptReceived1;

    if (!allExpectedAcceptsReceived) {
        return;
    }

    PendingMcmCommand command;
    command.kind = PendingMcmCommand::Kind::Negotiation;
    command.subtype = mcmSubtype::Execute;
    command.priority = mPriorityMcmCategory;
    command.cooperationType = snapshot.cooperationTypeMcm >= 0 ? snapshot.cooperationTypeMcm : 0;
    command.requestId = mRvRequestId;
    command.numberOfVehicles = mRvNumberOfVehicles;
    command.targetVehicle1 = mRvTargetVehicle1;
    command.hasTargetVehicle2 = mRvNumberOfVehicles > 1 && mRvTargetVehicle2 != 0;
    command.targetVehicle2 = mRvTargetVehicle2;
    command.requestedTrajectory = mEgoContext.plannedTrajectory;

    mActiveNegotiatedTrajectory = command.requestedTrajectory;
    mHasActiveNegotiatedTrajectory = !mActiveNegotiatedTrajectory.empty();

    mPendingMcmCommand = command;
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
            << " execution=lateral-control-not-implemented\n";
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

void McApplication::handleReceivedExecuteAsCv(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (!mHasEgoContext || !mVehicleDataProvider ||
            mCooperatingVehicleType != cooperatingVehicleType::CV ||
            mCoordinationProgressCV != coordinationProgressCV::AcceptSent) {
        return;
    }

    const auto& snapshot = received.data;
    if (!snapshot.hasNegotiationContainer ||
            snapshot.mcmCategory != static_cast<long>(mcmSubtype::Execute)) {
        return;
    }

    if (snapshot.stationId != mCvRvStationId) {
        return;
    }

    if (snapshot.requestId < 0 ||
            static_cast<uint8_t>(snapshot.requestId) != mCvRequestId) {
        return;
    }

    const uint32_t egoStationId = mVehicleDataProvider->station_id();
    const bool targetsEgo = snapshot.negotiationVehicleId1 == egoStationId ||
        (snapshot.hasNegotiationVehicleId2 && snapshot.negotiationVehicleId2 == egoStationId);
    if (!targetsEgo) {
        return;
    }

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
    mActiveNegotiatedTrajectory = mEgoContext.plannedTrajectory;
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
            mEmergencyReceived ||
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

    const auto& emergencyPoint = snapshot.plannedTrajectory.front();
    const auto& egoPoint = mEgoContext.plannedTrajectory.front();
    if (mEgoContext.y >= 452368.0 || egoPoint.mY < emergencyPoint.mY) {
        EV_INFO << "[MCM-EMERGENCY]"
            << " simTime=" << mEgoContext.now
            << " role=lane-1-follower"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " route=" << mEgoContext.routeId
            << " event=emergency-mcm-ignored"
            << " reason=not-behind-relevant-emergency-window"
            << " egoY=" << egoPoint.mY
            << " emergencyY=" << emergencyPoint.mY
            << '\n';
        return;
    }

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
    const double dx = egoPoint.mX - emergencyPoint.mX;
    const double dy = egoPoint.mY - emergencyPoint.mY;
    const double distanceGap = std::sqrt(dx * dx + dy * dy);
    const double timeGap = mEgoContext.speed > 0.1 ? distanceGap / mEgoContext.speed : -1.0;
    const double reactionWindow = timeGap >= 0.0 ? timeGap - scSafetyCriticalTimeGap : -1.0;
    const bool safetyCritical = conflictAtCoordinationGap;

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
        << " distanceGap=" << distanceGap
        << " timeGap=" << timeGap
        << " desiredMinimumTimeGap=" << scSafetyCriticalTimeGap
        << " reactionWindow=" << reactionWindow
        << " paperInitialTimeGap=" << scInitialPaperTimeGap
        << " oldMinimumGapConflict=" << conflictAtMinimumGap
        << " oldCoordinationGap=" << scEmergencyCoordinationTimeGap
        << " oldCoordinationConflict=" << conflictAtCoordinationGap
        << " safetyCritical=" << safetyCritical
        << '\n';

    if (!safetyCritical) {
        EV_INFO << "[MCM-LC-TRIGGER]"
            << " simTime=" << mEgoContext.now
            << " role=lane-1-follower"
            << " station=" << mEgoContext.stationId
            << " vehicleId=" << mVehicleController->getVehicleId()
            << " result=no-request"
            << " reason=emergency-gap-not-safety-critical"
            << " timeGap=" << timeGap
            << " reactionWindow=" << reactionWindow
            << '\n';
        return;
    }

    mEmergencyReceived = true;
    mEmergencyStationId = snapshot.stationId;
    mEmergencyDistanceGap = distanceGap;
    mEmergencyTimeGap = timeGap;
    mEmergencyReactionWindow = reactionWindow;
    mCooperatingVehicleType = cooperatingVehicleType::RV;
    mCoordinationProgressRV = coordinationProgressRV::CheckForCoordination;
    mControlManeuver = controlManeuver::ChangeLane;
    mPriorityMcmCategory = priorityMcmCategory::HighPriority;

    EV_INFO << "[MCM-LC-TRIGGER]"
        << " simTime=" << mEgoContext.now
        << " role=safety-critical-lane-change-rv"
        << " station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " route=" << mEgoContext.routeId
        << " laneIndex=" << mEgoContext.laneIndex
        << " event=safety-critical-trigger-armed"
        << " emergencyStation=" << mEmergencyStationId
        << " distanceGap=" << mEmergencyDistanceGap
        << " timeGap=" << mEmergencyTimeGap
        << " reactionWindow=" << mEmergencyReactionWindow
        << " priority=HighPriority"
        << " controlManeuver=ChangeLane\n";
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
