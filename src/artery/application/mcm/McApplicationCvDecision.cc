#include "artery/application/mcm/McApplication.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McScenarioConfig.h"
#include "artery/application/mcm/McmEnumUtils.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>

#include <algorithm>
#include <exception>

/*
 * Implements CV cooperation decisions for McApplication.
 *
 * This file runs trajectory feasibility checks, cooperation-cost evaluation,
 * priority comparison, and planner-measurement recording for received Requests.
 * The methods still use the shared McApplication negotiation and trajectory
 * state declared in McApplication.h.
 *
 * The source split is organizational only; it does not create an independent CV
 * planner component.
 */

namespace artery
{
namespace mcm
{

namespace
{
using scenario::scMergingTimeGap;
using scenario::scRequestTrajectoryDt;
using scenario::scRequestTrajectorySteps;

}

/*
 * Evaluates whether this CV can cooperate with a received Request. It combines
 * conflict checking, route-reference availability, leader constraints,
 * trajectory-planner results, cooperation cost, and priority gating before a
 * response command is built by the caller.
 */
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
        // Priority-level ordering is numeric severity:
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

        recordCvPlannerEvaluation(
            decision.cooperationCost,
            decision.trajectoryType,
            decision.possiblePriorityLevel);
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

/*
 * Converts the selected planner result into scalar measurements and counters.
 * These records are diagnostics for analysis and do not feed back into the
 * current cooperation decision.
 */
void McApplication::recordCvPlannerEvaluation(
    double trajectoryCost,
    int trajectoryType,
    int possiblePriorityLevel)
{
    enqueuePlannerMeasurement(PlannerMeasurementMetric::TrajectoryCost, trajectoryCost);

    // The raw OMNeT++ counter names are non-contiguous, so the analysis
    // helper adds contiguous public trajectory_category labels for readability.
    // The labels describe the planner categories, including acceleration or
    // deceleration combined with a lane-change trajectory where applicable.
    switch (trajectoryType) {
        case 0:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType0);
            break;
        case 1:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType1);
            break;
        case 2:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType2);
            break;
        case 3:
            break;
        case 4:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType4);
            break;
        case 5:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType5);
            break;
        case 6:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterTrajectoryType6);
            break;
        default:
            break;
    }

    switch (possiblePriorityLevel) {
        case 0:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterCoordPossiblePriorityLow);
            break;
        case 1:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterCoordPossiblePriorityMedium);
            break;
        case 2:
            enqueuePlannerMeasurement(PlannerMeasurementMetric::CounterCoordPossiblePriorityHigh);
            break;
        default:
            break;
    }
}

void McApplication::enqueuePlannerMeasurement(PlannerMeasurementMetric metric, double value)
{
    mPendingPlannerMeasurements.push_back(PlannerMeasurement { metric, value });
}

} // namespace mcm
} // namespace artery
