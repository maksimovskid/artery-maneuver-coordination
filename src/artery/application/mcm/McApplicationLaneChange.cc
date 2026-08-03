#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/traci/VehicleController.h"

#include <boost/units/systems/si/velocity.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

/*
 * Implements emergency and safety-critical lane-change behavior for
 * McApplication.
 *
 * This file contains the emergency trigger, scenario vehicle lifetime logging,
 * safety-critical lane-change Request generation, emergency-follower handling,
 * and fallback-braking logic. The lane-change trajectory construction preserves
 * the current simulation-specific global-coordinate and fixed-width shift
 * assumptions.
 *
 * The source split is organizational only. All definitions are member functions
 * of the single McApplication class declared in McApplication.h and operate on
 * the same shared application state.
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
using scenario::scInitialPaperTimeGap;
using scenario::scLaneChangeShiftX;
using scenario::scLaneChangeEmergencyFallbackDecelerationTime;
using scenario::scLaneChangeEmergencyFallbackSpeed;
using scenario::scMergingTimeGap;
using scenario::scNormalHighwaySpeed;
using scenario::scRequestTrajectoryDt;
using scenario::scRequestTrajectorySteps;
using scenario::scSafetyCriticalLaneChangeRouteId;
using scenario::scSafetyCriticalTimeGap;
using scenario::scValidationMapLaneIndexCorrectionThresholdY;

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

} // namespace

/*
 * Drives the scenario emergency source vehicle. Once the configured start time
 * is reached, the vehicle brakes and repeatedly broadcasts EmergencyPriority
 * execution evidence for followers to evaluate.
 */
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

    if (mEmergencyBrakingOnlyBaseline) {
        if (!mEmergencyBroadcastStarted && withinBroadcastWindow) {
            mEmergencyBroadcastStarted = true;
            mEmergencyBroadcastStartedAt = now;
            EV_INFO << "[MCM-BASELINE]"
                << " simTime=" << now
                << " role=emergency-vehicle"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " laneIndex=" << mEgoContext.laneIndex
                << " event=emergency-mcm-suppressed"
                << " reason=emergency-braking-only-baseline"
                << " scheduledStartTime=" << scEmergencyStartTime
                << " duration=" << scEmergencyBroadcastDuration
                << " sendInterval=" << scEmergencyBroadcastInterval
                << '\n';
        }

        if (!mEmergencyBroadcastFinished && emergencyElapsed >= scEmergencyBroadcastDuration) {
            mEmergencyBroadcastFinished = true;
            EV_INFO << "[MCM-BASELINE]"
                << " simTime=" << now
                << " role=emergency-vehicle"
                << " station=" << mEgoContext.stationId
                << " vehicleId=" << mVehicleController->getVehicleId()
                << " route=" << mEgoContext.routeId
                << " laneIndex=" << mEgoContext.laneIndex
                << " event=emergency-mcm-suppression-finished"
                << " elapsed=" << emergencyElapsed
                << " duration=" << scEmergencyBroadcastDuration
                << '\n';
        }
    } else {
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

/*
 * Emits one-time lifecycle diagnostics for the vehicles involved in the
 * safety-critical lane-change scenario. These logs help validate scenario
 * participation and do not alter maneuver decisions.
 */
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

/*
 * Starts high-priority lane-change coordination for an armed follower. The
 * method selects target CVs, constructs the requested trajectory using the
 * current global SUMO coordinate convention and fixed lateral shift, initializes
 * RV negotiation state, and queues the Request.
 */
void McApplication::evaluateSafetyCriticalLaneChangeTrigger(omnetpp::SimTime now)
{
    EV_STATICCONTEXT;

    if (mEmergencyBrakingOnlyBaseline) {
        return;
    }

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
        const bool laneWorkaroundApplied = first.mY > scValidationMapLaneIndexCorrectionThresholdY;
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
    mRvCoordinationFailed = false;
    mRvRequestedTrajectory = laneChangeTrajectory;
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

/*
 * Arms a lane-1 follower after receiving emergency execution evidence from the
 * scenario emergency vehicle. Same-lane/ahead checks gate the arm state, while
 * the logged trajectory-conflict checks remain diagnostic.
 */
void McApplication::handleReceivedEmergencyAsFollower(const ReceivedMcm& received)
{
    EV_STATICCONTEXT;

    if (mEmergencyBrakingOnlyBaseline) {
        return;
    }

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

/*
 * Applies the high-priority fallback path when coordination cannot continue
 * safely. It commands conservative braking, clears pending coordination state,
 * and records the second-request rejection measurement when applicable.
 */
void McApplication::applyEmergencyFallbackBrake(
    const char* event,
    const char* reason,
    uint8_t requestId)
{
    EV_STATICCONTEXT;

    const double currentSpeed = mHasEgoContext ? mEgoContext.speed : 0.0;
    const double targetSpeed = scLaneChangeEmergencyFallbackSpeed;
    const double decelerationTime = scLaneChangeEmergencyFallbackDecelerationTime;

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
    mRvCoordinationFailed = true;
    mRvLastRequestQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastRequestQueuedAt = false;
    mRvLastConfirmQueuedAt = omnetpp::SimTime::ZERO;
    mHasRvLastConfirmQueuedAt = false;
    mRvNegotiationStartedAt = omnetpp::SimTime::ZERO;
    mHasRvNegotiationStartedAt = false;
    mRvRequestedTrajectory.clear();
    mHasRvRequestedTrajectory = false;
    mRvNegotiatedTrajectory.clear();
    mHasRvNegotiatedTrajectory = false;
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

}  // namespace mcm
}  // namespace artery
