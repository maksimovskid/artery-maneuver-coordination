#include "artery/application/mcm/McApplication.h"

#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/traci/VehicleController.h"

#include <libsumo/TraCIConstants.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/velocity.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

/*
 * Implements SUMO vehicle execution control for McApplication.
 *
 * This file applies and monitors RV and CV speed and lane-change commands after
 * negotiation decisions have selected an executable maneuver. It is separate
 * from execution-progress message handling, but all methods still operate on
 * the same McApplication object and shared state declared in McApplication.h.
 *
 * The source split is organizational only; it does not create an independent
 * execution controller.
 */

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scMergingRouteId;
using scenario::scExecutionRestoreMinFrontDistance;
using scenario::scExecutionRestoreMinTtc;
using scenario::scHighwayCvAccelerationTargetSpeed;
using scenario::scLaneChangeExecutionLateralShiftPerStep;
using scenario::scLaneChangeExecutionMinFrontDistance;
using scenario::scLaneChangeExecutionMinTimeGap;
using scenario::scLaneChangeExecutionMinTtc;
using scenario::scLaneChangeExecutionStepCount;
using scenario::scMergingRvExecutionSpeed;
using scenario::scNormalHighwaySpeed;
using scenario::scSafetyCriticalLaneChangeRouteId;
using scenario::scSafetyCriticalTimeGap;
using scenario::scTargetLaneChangeRouteId;
using scenario::scValidationMapLaneIndexCorrectionThresholdY;

} // namespace

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

    // The route_merging_1 RV path uses speedMode 0 only for the merging RV.
    // This prevents SUMO right-of-way logic from stopping/decelerating the RV at
    // the merge. Later ordinary speed-control maneuvers, such as CV
    // acceleration/deceleration, should use speedMode 31 so SUMO safety checks
    // remain active and speed changes stay realistic.
    mVehicleController->setSpeedMode(vehicleId, 0);
    mVehicleController->setMaxSpeed(scMergingRvExecutionSpeed * boost::units::si::meter_per_second);
    mVehicleController->setSpeed(scMergingRvExecutionSpeed * boost::units::si::meter_per_second);

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
        (frontInfo.distance > scExecutionRestoreMinFrontDistance &&
            timeGap >= scSafetyCriticalTimeGap && ttc >= scExecutionRestoreMinTtc);
}

/*
 * Restores normal SUMO speed control only after a conservative leader check.
 * Both RV and CV completion paths call this helper, so the guard prevents a
 * completed maneuver from immediately accelerating into an unsafe gap.
 */
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

/*
 * Applies RV lane-change execution for the safety-critical scenario. The method
 * drives the current fixed-step moveToXY behavior and leaves the negotiated
 * trajectory and protocol state sequencing unchanged.
 */
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
        (frontInfo.distance < scLaneChangeExecutionMinFrontDistance ||
            timeGap < scLaneChangeExecutionMinTimeGap || ttc < scLaneChangeExecutionMinTtc);
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

    if (mLaneChangeMoveStepCounter >= scLaneChangeExecutionStepCount) {
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

    // Apply the configured lane-change displacement incrementally. This physical
    // execution calibration is intentionally separate from the planned shift.
    const double targetX = currentX + scLaneChangeExecutionLateralShiftPerStep;
    const double targetY = currentY - currentSpeed / scLaneChangeExecutionStepCount;

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

    // The CV deceleration path keeps SUMO safety checks enabled with speedMode 31;
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
        mTargetSpeed : scHighwayCvAccelerationTargetSpeed;

    // Highway CV acceleration uses speedMode 31 so SUMO safety checks stay
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

    // Lateral control transitions ChangeLane to LaneChangeExecution and
    // uses the configured incremental moveToXY execution based on live TraCI
    // lane, position, and speed.
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

/*
 * Monitors CV execution after Execute has been received. It logs live TraCI
 * state and restores normal speed when the active trajectory has completed,
 * without creating additional protocol messages.
 */
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
        if (rvPoint.mY > scValidationMapLaneIndexCorrectionThresholdY && rvLane >= 0) {
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
            // Once the RV is in front and faster, CV deceleration is no longer
            // needed, but only if the CV's own leader
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

    const bool hadAccelerationControl = mCvAccelerationControlApplied;
    const bool hadDecelerationControl = mCvDecelerationControlApplied;

    restoreNormalSpeedIfSafe("CV", mCvRequestId);

    EV_INFO << "McApplication applied highway-merging CV speed-control reset"
        << ": station=" << mEgoContext.stationId
        << " vehicleId=" << mVehicleController->getVehicleId()
        << " speedMode=31 normalSpeed=27.77"
        << " hadDecelerationControl=" << hadDecelerationControl
        << " hadAccelerationControl=" << hadAccelerationControl
        << '\n';
}

} // namespace mcm
} // namespace artery
