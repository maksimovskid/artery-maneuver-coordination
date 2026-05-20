#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/application/mcm/TrajectoryCsv.h"
#include "artery/application/mcm/TrajectoryEnvironment.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>

#include <fstream>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <sstream>

namespace artery
{

using namespace omnetpp;

namespace mcm
{

TrajectoryPlanner::TrajectoryPlanner()
: mVehicleController(nullptr), mVehicleDataProvider(nullptr), mLocalEnvironmentModel(nullptr)
{}

TrajectoryPlanner::TrajectoryPlanner(const traci::VehicleController* mvc, const VehicleDataProvider* mvdp, const LocalEnvironmentModel* lem)
: mVehicleController(mvc), mVehicleDataProvider(mvdp), mLocalEnvironmentModel(lem){
}

void TrajectoryPlanner::initialize(const traci::VehicleController* mvc, const VehicleDataProvider* mvdp, const LocalEnvironmentModel* lem)
{
	mVehicleController = mvc;
	mVehicleDataProvider = mvdp;
	mLocalEnvironmentModel = lem;
};
TrajPointMCM TrajectoryPlanner::getPoint(TrajPointMCM calculatedPoint) const
{
	TrajPointMCM point;
	point.mX = calculatedPoint.mX;
	point.mY = calculatedPoint.mY;
	point.mHeading = calculatedPoint.mHeading; 
	point.mTime = calculatedPoint.mTime;
	
	return point;
}


TrajectoryPlanner::Trajectory TrajectoryPlanner::calculateRefTrajectory(
    int steps,
    double dt,
    const Vec_f& cx,
    const Vec_f& cy,
    int mIndex,
    bool constantVelocity,
    float targetSpeed,
    float acceleration,
    float deceleration)
{
    Trajectory trajectory;

    if (steps <= 0 || dt <= 0.0 || cx.empty() || cy.empty()) {
        return trajectory;
    }

    State state;

    state.x = mVehicleController->getPositionSumo().x /
              boost::units::si::meter;

    state.y = mVehicleController->getPositionSumo().y /
              boost::units::si::meter;

    state.yaw = mVehicleDataProvider->heading() /
                boost::units::si::radians;

    state.v = mVehicleDataProvider->speed() /
              boost::units::si::meter_per_second;

    const int ncourse = static_cast<int>(std::min(cx.size(), cy.size()));

    int ind = calc_nearest_index(state, cx, cy, mIndex);

    if (mIndex >= ind) {
        ind = mIndex;
    }

    const float dl = 1.0F;
    float travel = 0.0F;

    for (int i = 0; i < steps; ++i) {

        const float old_v = state.v;

        if (!constantVelocity && i > 0) {

            if (state.v > targetSpeed) {

                state.v -= deceleration * dt;

                if (state.v < targetSpeed) {
                    state.v = targetSpeed;
                }
            }
            else if (state.v < targetSpeed) {

                state.v += acceleration * dt;

                if (state.v > targetSpeed) {
                    state.v = targetSpeed;
                }
            }
        }

        // s = v0*t + 0.5*a*t^2
        // equivalent to average velocity integration
        travel +=
            0.5F * (std::abs(old_v) + std::abs(state.v)) * dt;

        const int dind =
            static_cast<int>(std::round(travel / dl));

        int targetIndex = ind + dind;

        if (targetIndex >= ncourse) {
            targetIndex = ncourse - 1;
        }

        TrajPointMCM calculatedPoint;

        calculatedPoint.mX = cx[targetIndex];
        calculatedPoint.mY = cy[targetIndex];

        calculatedPoint.mHeading =
            mVehicleDataProvider->heading() /
            boost::units::si::radians;

        calculatedPoint.mTime =
            simTime() + (i + 1) * dt;

        trajectory.push_back(getPoint(calculatedPoint));
    }

    return trajectory;
}

int TrajectoryPlanner::calc_nearest_index(State state, const Vec_f& cx, const Vec_f& cy, int pind)
{
    // Work on the common prefix because route CSV columns can be loaded independently.
    const int ncourse = static_cast<int>(std::min(cx.size(), cy.size()));

    if (ncourse <= 0) {
        return 0;
    }

    if (pind < 0) {
        pind = 0;
    }

    if (pind >= ncourse) {
        return ncourse - 1;
    }

    Vec_f distances;
    int ind = 0;

    for (int i = pind; i < ncourse; i++) {
        float idx = cx[i] - state.x;
        float idy = cy[i] - state.y;
        float d_e = idx * idx + idy * idy;
        distances.push_back(d_e);
    }

    auto minElementIndex =
        std::min_element(distances.begin(), distances.end()) - distances.begin();

    ind = pind + minElementIndex;

    if (ind >= ncourse) {
        ind = ncourse - 1;
    }

    return ind;
}

void TrajectoryPlanner::printTrajectory(const Trajectory& trajectory) {
    std::cout << "Printing trajectory for vehicle:" << mVehicleController->getVehicleId() << std::endl;
    for (const auto& point : trajectory) {
        std::cout << "mX: " << point.mX << ", "
                  << "mY: "  << point.mY << ", "
                  << "mYaw: " << point.mHeading << ", "
                  << "mT: " << point.mTime << std::endl;
    }
}

bool TrajectoryPlanner::check_traj_conflict(
    Trajectory traj_ego,
    Trajectory traj_other,
    float time_gap,
    omnetpp::SimTime receivedEteDelay)
{
    const int traj_size = std::min(traj_ego.size(), traj_other.size());

    if (traj_size < 2) {
        return false;
    }

    std::vector<int> indices;
    indices.push_back(0);
    indices.push_back(traj_size / 4);
    indices.push_back(traj_size / 2);
    indices.push_back((3 * traj_size) / 4);
    indices.push_back(traj_size - 1);

    const float current_speed =
        mVehicleDataProvider->speed() /
        boost::units::si::meter_per_second;

    const float distance_gap = time_gap * current_speed;

    const bool compensate_delay =
        receivedEteDelay > 0.05 &&
        traj_other[traj_size - 1].mY <
        traj_other[traj_size - 2].mY;

    const double convertedDelay = receivedEteDelay.dbl();

    for (unsigned int i = 0; i < indices.size(); ++i) {

        const int idx = indices[i];

        float other_y = traj_other[idx].mY;

        if (compensate_delay) {

            int prev_idx = idx - 1;

            if (prev_idx < 0) {
                prev_idx = 0;
            }

            const double dt =
                (traj_other[idx].mTime -
                 traj_other[prev_idx].mTime).dbl();

            float received_speed = 0.0F;

            if (dt > 0.0) {

                const float dx_speed =
                    traj_other[idx].mX -
                    traj_other[prev_idx].mX;

                const float dy_speed =
                    traj_other[idx].mY -
                    traj_other[prev_idx].mY;

                received_speed =
                    std::hypot(dx_speed, dy_speed) / dt;
            }

            other_y =
                traj_other[idx].mY -
                received_speed * convertedDelay;
        }

        const float dx =
            traj_ego[idx].mX - traj_other[idx].mX;

        const float dy =
            traj_ego[idx].mY - other_y;

        const float distance =
            std::hypot(dx, dy);

        if (distance < distance_gap) {
            return true;
        }
    }

    return false;
}

bool TrajectoryPlanner::check_traj_conflict_lane_change(
    Trajectory traj_ego,
    Trajectory traj_other,
    float time_gap,
    omnetpp::SimTime receivedEteDelay)
{
    const int traj_size = std::min(traj_ego.size(), traj_other.size());

    if (traj_size < 2) {
        return false;
    }

    std::vector<int> indices;
    indices.push_back(0);
    indices.push_back(traj_size / 5);
    indices.push_back((2 * traj_size) / 5);
    indices.push_back((3 * traj_size) / 5);
    indices.push_back((4 * traj_size) / 5);
    indices.push_back(traj_size - 1);

    const float current_speed =
        mVehicleDataProvider->speed() /
        boost::units::si::meter_per_second;

    const float distance_gap = time_gap * current_speed;
    const float lane_width_gap = 2.5F;

    const bool compensate_delay =
        receivedEteDelay > 0.05 &&
        traj_other[traj_size - 1].mY <
        traj_other[traj_size - 2].mY;

    const double convertedDelay = receivedEteDelay.dbl();

    for (unsigned int i = 0; i < indices.size(); ++i) {

        const int idx = indices[i];

        float other_y = traj_other[idx].mY;

        if (compensate_delay) {

            int prev_idx = idx - 1;

            if (prev_idx < 0) {
                prev_idx = 0;
            }

            const double dt =
                (traj_other[idx].mTime -
                 traj_other[prev_idx].mTime).dbl();

            float received_speed = 0.0F;

            if (dt > 0.0) {

                const float dx_speed =
                    traj_other[idx].mX -
                    traj_other[prev_idx].mX;

                const float dy_speed =
                    traj_other[idx].mY -
                    traj_other[prev_idx].mY;

                received_speed =
                    std::hypot(dx_speed, dy_speed) / dt;
            }

            other_y =
                traj_other[idx].mY -
                received_speed * convertedDelay;
        }

        const float dx =
            traj_ego[idx].mX -
            traj_other[idx].mX;

        const float dy =
            traj_ego[idx].mY -
            other_y;

        const float distance =
            std::hypot(dx, dy);

        if (std::abs(dx) < lane_width_gap &&
            distance < distance_gap) {
            return true;
        }
    }

    return false;
}

bool TrajectoryPlanner::check_traj_conflict_merging(
    Trajectory traj_ego,
    Trajectory traj_other,
    float time_gap,
    omnetpp::SimTime receivedEteDelay,
    bool forRV)
{
    const int traj_size = std::min(traj_ego.size(), traj_other.size());

    if (traj_size < 2) {
        return false;
    }

    const int t1 = traj_size - 1;
    const int t2 = traj_size - 2;

    const float current_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

    const float distance_gap = time_gap * current_speed;

    const float dx = traj_ego[t1].mX - traj_other[t1].mX;
    const float dy = traj_ego[t1].mY - traj_other[t1].mY;
    const float distance = std::hypot(dx, dy);

    float received_speed = 0.0F;

    const double dt =
        (traj_other[t1].mTime - traj_other[t2].mTime).dbl();

    if (dt > 0.0) {
        const float dx_speed = traj_other[t1].mX - traj_other[t2].mX;
        const float dy_speed = traj_other[t1].mY - traj_other[t2].mY;

        received_speed = std::hypot(dx_speed, dy_speed) / dt;
    }

    if (receivedEteDelay > 0.05) {
        const double convertedDelay = receivedEteDelay.dbl();

        if (forRV && traj_other[t1].mY < traj_other[t2].mY) {
            const float other_y =
                traj_other[t1].mY - received_speed * convertedDelay;

            const float dy_new = traj_ego[t1].mY - other_y;
            const float distance_new = std::hypot(dx, dy_new);

            if (distance_new < distance_gap) {
                return true;
            }
        }
        else if (!forRV && traj_other[t1].mX > traj_other[t2].mX) {
            const float other_x =
                traj_other[t1].mX + received_speed * convertedDelay;

            const float dx_new = traj_ego[t1].mX - other_x;
            const float distance_new = std::hypot(dx_new, dy);

            if (distance_new < distance_gap) {
                return true;
            }
        }
    }

    return distance < distance_gap;
}

double TrajectoryPlanner::getDistance(double x1, double y1, double x2, double y2)
{
    return artery::mcm::getDistance(x1, y1, x2, y2);
}

double TrajectoryPlanner::getDesiredGap()
{
    const double timeGap = 1.0;
    const double ego_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

    return timeGap * ego_speed;
}

double TrajectoryPlanner::getMinGapPriority(int maneuverPriority)
{
    return artery::mcm::getMinGapPriority(maneuverPriority);
}

double TrajectoryPlanner::getGap(double gap)
{
    const double ego_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

    return gap * ego_speed;
}

double TrajectoryPlanner::calculateTimeGap(double distance, double speed)
{
    return artery::mcm::calculateTimeGap(distance, speed);
}

FrontVehicleInfo TrajectoryPlanner::getFrontVehicleInfo()
{
    FrontVehicleInfo FVinfo;

    FVinfo.frontVehID = 1;
    FVinfo.sameEdgeLane = false;
    FVinfo.distance = 999.0;
    FVinfo.speed = 99.0;

    const auto& objects = mLocalEnvironmentModel->allObjects();

    const std::string ego_id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    const std::string ego_RoadID = vehicle_api.getRoadID(ego_id);
    const int ego_LaneIndex = vehicle_api.getLaneIndex(ego_id);

    const auto myPosition = mVehicleController->getPosition();

    for (auto& obj : objects) {
        const std::weak_ptr<artery::EnvironmentModelObject> obj_ptr = obj.first;

        if (obj_ptr.expired()) {
            continue;
        }

        const auto object = obj_ptr.lock();
        const auto other_vehicle_id = object->getExternalId();

        if (other_vehicle_id == ego_id) {
            continue;
        }

        const std::string other_RoadID = vehicle_api.getRoadID(other_vehicle_id);
        const int other_LaneIndex = vehicle_api.getLaneIndex(other_vehicle_id);

        if (!checkIfSameEdgeAndLane(
                ego_RoadID,
                ego_LaneIndex,
                other_RoadID,
                other_LaneIndex)) {
            continue;
        }

        const auto& otherPosition = object->getCentrePoint();

        const double distance = getDistance(
            myPosition.x / boost::units::si::meters,
            myPosition.y / boost::units::si::meters,
            otherPosition.x / boost::units::si::meters,
            otherPosition.y / boost::units::si::meters);

        if (distance < FVinfo.distance) {
            FVinfo.frontVehID = 1;  // dummy, because other_vehicle_id is string
            FVinfo.sameEdgeLane = true;
            FVinfo.distance = distance;
            FVinfo.speed = vehicle_api.getSpeed(other_vehicle_id);
        }
    }

    return FVinfo;
}

bool TrajectoryPlanner::checkIfSameEdgeAndLane(
    const std::string& egoEdge,
    int egoLane,
    const std::string& otherEdge,
    int otherLane)
{
    return artery::mcm::checkIfSameEdgeAndLane(egoEdge, egoLane, otherEdge, otherLane);
} 

bool TrajectoryPlanner::isWithinInterval (double value, double lowerBound, double upperBound){
	return artery::mcm::isWithinInterval(value, lowerBound, upperBound);
}

double TrajectoryPlanner::calculateDecelerationTime(
    double initialVelocity,
    double finalVelocity,
    double deceleration)
{
    return artery::mcm::calculateDecelerationTime(initialVelocity, finalVelocity, deceleration);
}

double TrajectoryPlanner::calculateTTC(
    double mySpeed,
    double frontVehicleSpeed,
    double distance)
{
    return artery::mcm::calculateTTC(mySpeed, frontVehicleSpeed, distance);
}

bool TrajectoryPlanner::check_traj_collision(
    Trajectory traj_ego,
    Trajectory traj_other,
    float min_TTC,
    omnetpp::SimTime receivedEteDelay)
{
    const int traj_size = std::min(traj_ego.size(), traj_other.size());

    if (traj_size < 2) {
        return false;
    }

    std::vector<int> indices;
    indices.push_back(0);
    indices.push_back(traj_size / 4);
    indices.push_back(traj_size / 2);
    indices.push_back((3 * traj_size) / 4);
    indices.push_back(traj_size - 1);

    const bool compensate_delay =
        receivedEteDelay > 0.05 &&
        traj_other[traj_size - 1].mY < traj_other[traj_size - 2].mY;

    const double convertedDelay = receivedEteDelay.dbl();

    for (unsigned int i = 0; i < indices.size(); ++i) {
        const int idx = indices[i];

        int prev_idx = idx - 1;

        if (prev_idx < 0) {
            prev_idx = 0;
        }

        if (idx == 0) {
            prev_idx = 1;
        }

        const double ego_dt = std::abs((traj_ego[idx].mTime - traj_ego[prev_idx].mTime).dbl());

		const double other_dt = std::abs((traj_other[idx].mTime - traj_other[prev_idx].mTime).dbl());

        float ego_speed = 0.0F;
        float other_speed = 0.0F;

        if (ego_dt > 0.0F) {
            const float ego_dx =
                traj_ego[idx].mX - traj_ego[prev_idx].mX;
            const float ego_dy =
                traj_ego[idx].mY - traj_ego[prev_idx].mY;

            ego_speed = std::hypot(ego_dx, ego_dy) / ego_dt;
        }

        if (other_dt > 0.0F) {
            const float other_dx =
                traj_other[idx].mX - traj_other[prev_idx].mX;
            const float other_dy =
                traj_other[idx].mY - traj_other[prev_idx].mY;

            other_speed = std::hypot(other_dx, other_dy) / other_dt;
        }

        const float relative_speed = ego_speed - other_speed;

        if (relative_speed <= 0.0F) {
            continue;
        }

        float other_y = traj_other[idx].mY;

        if (compensate_delay) {
            other_y = traj_other[idx].mY - other_speed * convertedDelay;
        }

        const float dx = traj_ego[idx].mX - traj_other[idx].mX;
        const float dy = traj_ego[idx].mY - other_y;
        const float distance = std::hypot(dx, dy);

        const float ttc = distance / relative_speed;

        if (ttc < min_TTC) {
            return true;
        }
    }

    return false;
}

// Function to iterate over different parameters and find a suitable trajectory
TrajectoryPlanner::TupleSuitableTrajectory TrajectoryPlanner::findSuitableTrajectoryCV(
	Trajectory receivedReqTraj,
	int priority,
	bool mergingReqTraj,
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	double current_speed,
	omnetpp::SimTime receivedEteDelay,
	bool decelerationRequired,
	bool accelerationRequired,
	bool isLaneChangePossible,
	bool isRouteAffected)
{

	bool firstCheckWithLowPriority = true;
	bool secondCheckWithMediumPriority = false;
	bool trajectory_found = false;
	bool acceleration_trajectory_found = false;
	float timeGapMerging = 1.0;
		Trajectory currentTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, 0.0, 0.0, 0.0);    // PT  
		Trajectory finalTrajectory = currentTraj;
		Trajectory newTraj; 
		// Empty candidates mean planning failed; do not let conflict helpers classify them as acceptable.
		auto newTrajMergingHasConflict = [&](float timeGap) {
			return !newTraj.empty() &&
				check_traj_conflict_merging(newTraj, receivedReqTraj, timeGap, receivedEteDelay, false);
		};
		auto newTrajMergingConflictFree = [&](float timeGap) {
			return !newTraj.empty() &&
				!check_traj_conflict_merging(newTraj, receivedReqTraj, timeGap, receivedEteDelay, false);
		};
		auto newTrajLaneChangeHasConflict = [&](float timeGap) {
			return !newTraj.empty() &&
				check_traj_conflict_lane_change(newTraj, receivedReqTraj, timeGap, receivedEteDelay);
		};
		auto newTrajLaneChangeConflictFree = [&](float timeGap) {
			return !newTraj.empty() &&
				!check_traj_conflict_lane_change(newTraj, receivedReqTraj, timeGap, receivedEteDelay);
		};
		PlannedTrajValues plannedTrajValues{0.0, 0.0, 0.0, false, 1.0, 3.0};
	double foundTrajectoryCost = 0.0;
	int typeOfTrajectory = 0;
	int possiblePriorityLevelForAccept = 10;  // related to costs to accept request 0 = low, 1 = meidum, 2 = high priority, 10 shows no priority possible

	// Define parameters for iteration for low priority up to 20% change of plan
    std::vector<double> speedReductionPercentagesLowPriority = {0.05, 0.1, 0.15, 0.2};
    std::vector<double> decelerationValuesLowPriority = {0.5, 1.0, 1.5, 2.0};
    std::vector<double> timeGapReductionsLowPriority = {1.0, 0.9};
    std::vector<double> timeToCollisionReductionsLowPriority = {2.9, 2.8};

	// Define parameters for iteration for medium priority up to 40% change of plan
    std::vector<double> speedReductionPercentagesMediumPriority = {0.25, 0.3, 0.35, 0.4};
    std::vector<double> decelerationValuesMediumPriority = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    std::vector<double> timeGapReductionsMediumPriority = {1.0, 0.9, 0.7, 0.6};
    std::vector<double> timeToCollisionReductionsMediumPriority = {2.9, 2.8, 2.7, 2.6};
	
	// Define parameters for iteration for high priority up to 60% change of plan
    std::vector<double> speedReductionPercentagesHighPriority = {0.45, 0.5, 0.55, 0.6};
    std::vector<double> decelerationValuesHighPriority = {3.5, 4.0, 4.5, 5.0};
    std::vector<double> timeGapReductionsHighPriority = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
    std::vector<double> timeToCollisionReductionsHighPriority = {2.9, 2.8, 2.7, 2.6, 2.5, 2.4, 2.3, 2.2, 2.1, 2.0};

	/// @note added for Acceleration up to 120 kmh - same for all priorities as it is only 20 percent
	std::vector<double> speedIncreasePercentagesAllPriorities = {0.05, 0.1, 0.15, 0.2};
	std::vector<double> accelerationValuesAllPriorities = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};

    double foundSpeedReduction = 0.0;
	double foundSpeedIncrease = 0.0;
	double foundAcceleration = 0.0;
    double foundDeceleration = 0.0;
	bool lane_changed = false;
	double foundTimeGap = 1.0;
	
	if (priority == 1 or priority == 2){
		secondCheckWithMediumPriority = true;
	}
	if (mergingReqTraj == true){
		if (check_traj_conflict_merging(currentTraj, receivedReqTraj, timeGapMerging, receivedEteDelay, false) == true){
			// if there is a conflict start iteration first for low priority, then medium priority
			if (firstCheckWithLowPriority == true){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with low priority" << std::endl;
				
				if (accelerationRequired == true){
					/// @note below added for acceleration  - for merging for lowP
					// first increasing efficiency
					// Iterating over different speed increases and accelerations
					for (double acceleration : accelerationValuesAllPriorities) {
						if (trajectory_found) break;

						for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and acceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

							// Check for conflict
							if (newTrajMergingConflictFree(timeGapMerging)) {
								foundSpeedIncrease = speedIncrease;
								foundAcceleration = acceleration;
								trajectory_found  = true;
								acceleration_trajectory_found = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 0;
								break;  // Exit the loop when a suitable trajectory is found
							}
						}
					}
					
					if (trajectory_found == false && newTrajMergingHasConflict(timeGapMerging) && isLaneChangePossible == true && isRouteAffected == false){
						// if acc is not possible first, a new trajectory can be checked with constant speed but reduced time gap
						for (double timeGapReduction : timeGapReductionsLowPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

							// Check for conflict
							if (newTrajMergingConflictFree(timeGapReduction)) {
								foundTimeGap = timeGapReduction;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 0;
								break;  // Exit the loop when a suitable trajectory is found
							}
						}	
					}

					// try if lane change is possible 
						if (trajectory_found == false and isLaneChangePossible == true){					
							std::vector<float> cx_new(cx.size());
							std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
							newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);
							// Lane-change fallback remains a candidate only if the shifted trajectory is conflict-free.
							if (newTrajMergingConflictFree(timeGapMerging)) {
								trajectory_found  = true;
								lane_changed = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 0;
							}
						}

					// if reduced time gap and lane change are not possible, then try with deceleration
					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				// for decelerating trajectory
				if (decelerationRequired == true){
					// first reducing efficiency
					// Iterating over different speed reductions and decelerations
					for (double deceleration : decelerationValuesLowPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesLowPriority) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and deceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							// Check for conflict
							if (newTrajMergingConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 0;
								break;  // Exit the loop when a suitable trajectory is found
							}
						}
					}

					// if not possible with 20 % then check if laneChange is possible
					if (trajectory_found == false &&
						newTrajMergingHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == false){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajMergingConflictFree(timeGapMerging)) {
							trajectory_found = true;
							lane_changed = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 0;
						}
					}

					else if (trajectory_found == false &&
							 newTrajMergingHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						// Iterating over different speed reductions and decelerations and Time Gaps
						for (double deceleration : decelerationValuesLowPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesLowPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsLowPriority) {
									if (trajectory_found) break;

									// Calculate reference trajectory with adjusted speed and deceleration
									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false,
										current_speed - speedReduction * current_speed,
										0.0,
										deceleration);

									// Check for conflict
									if (newTrajMergingConflictFree(timeGapReduction)) {

										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;

										trajectory_found = true;
										finalTrajectory = newTraj;
										possiblePriorityLevelForAccept = 0;

										break;
									}
								}
							}
						}
					}
				}
			}
			if (secondCheckWithMediumPriority == true and trajectory_found == false){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with medium priority" << std::endl;
				
				if (accelerationRequired == true){
					// since acc is required first, a new trajectory can be checked with constant speed but reduced time gap
					for (double timeGapReduction : timeGapReductionsMediumPriority) {
						if (trajectory_found) break;

						newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

						// Check for conflict
						if (newTrajMergingConflictFree(timeGapReduction)) {
							foundTimeGap = timeGapReduction;
							trajectory_found  = true;
							finalTrajectory = newTraj;  // Found suitable trajectory
							accelerationRequired = false;
							possiblePriorityLevelForAccept = 1;
							break;
						}
					}	

					// try if lane change is possible 
						if (trajectory_found == false and isLaneChangePossible == true and isRouteAffected == true){					
							std::vector<float> cx_new(cx.size());
							std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
							newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);
							if (newTrajMergingConflictFree(timeGapMerging)) {
								trajectory_found  = true;
								lane_changed = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 1;
							}
						}

					// if reduced time gap and lane change are not possible, then try with deceleration
					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				if (decelerationRequired == true){
					// first reducing efficiency
					// Iterating over different speed reductions and decelerations
					for (double deceleration : decelerationValuesMediumPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesMediumPriority) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and deceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							// Check for conflict
							if (newTrajMergingConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 1;
								break;
							}
						}
					}

					// if not possible with 40 % then check if laneChange is possible if it affects the route
					if (trajectory_found == false &&
						newTrajMergingHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == true){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajMergingConflictFree(timeGapMerging)) {
							trajectory_found = true;
							lane_changed = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 1;
						}
					}

					else if (trajectory_found == false &&
							 newTrajMergingHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						// Iterating over different speed reductions and decelerations and Time Gaps
						for (double deceleration : decelerationValuesMediumPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesMediumPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsMediumPriority) {
									if (trajectory_found) break;

									// Calculate reference trajectory with adjusted speed and deceleration
									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

									// Check for conflict
									if (newTrajMergingConflictFree(timeGapReduction)) {
										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;  // Found suitable trajectory
										possiblePriorityLevelForAccept = 1;
										break;
									}
								}	
							}
						}	
					}
				}
			}

			// here for high priority
			if (priority == 2 and trajectory_found == false){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with high priority" << std::endl;
				
				if (accelerationRequired == true){
					// since acc is required first, a new trajectory can be checked with constant speed but reduced time gap
					for (double timeGapReduction : timeGapReductionsHighPriority) {
						if (trajectory_found) break;

						newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

						// Check for conflict
						if (newTrajMergingConflictFree(timeGapReduction)) {
							foundTimeGap = timeGapReduction;
							trajectory_found  = true;
							finalTrajectory = newTraj;  // Found suitable trajectory
							accelerationRequired = false;
							possiblePriorityLevelForAccept = 2;
							break;
						}
					}	

					// try if lane change is possible 
						if (trajectory_found == false and isLaneChangePossible == true and isRouteAffected == true){					
							std::vector<float> cx_new(cx.size());
							std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
							newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);
							if (newTrajMergingConflictFree(timeGapMerging)) {
								trajectory_found  = true;
								lane_changed = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 2;
							}
						}

					// if reduced time gap and lane change are not possible, then try with deceleration
					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				if (decelerationRequired == true){
					// first reducing efficiency
					// Iterating over different speed reductions and decelerations
					for (double deceleration : decelerationValuesHighPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesHighPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							// Check for conflict
							if (newTrajMergingConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 2;
								break;
							}
						}
					}

					// if not possible with 60 % then check if laneChange is possible if it affects the route
					if (trajectory_found == false &&
						newTrajMergingHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == true){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajMergingConflictFree(timeGapMerging)) {
							trajectory_found  = true;
							lane_changed = true;
							finalTrajectory = newTraj;  // Found suitable trajectory
							possiblePriorityLevelForAccept = 2;
						}
					}

					else if (trajectory_found == false &&
							 newTrajMergingHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						// Iterating over different speed reductions and decelerations and Time Gaps
						for (double deceleration : decelerationValuesHighPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesHighPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsHighPriority) {
									if (trajectory_found) break;

									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

									// Check for conflict
									if (newTrajMergingConflictFree(timeGapReduction)) {
										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;  // Found suitable trajectory
										possiblePriorityLevelForAccept = 2;
										break;
									}
								}	
							}
						}	
					}
				}
			}
		}
		else {
			trajectory_found = true;
			finalTrajectory = currentTraj;  // Found suitable trajectory
			possiblePriorityLevelForAccept = 0;
			std::cout << mVehicleController->getVehicleId() << " there is no conflict with received Req Traj " << std::endl;
		}

	}

	/// @note ------------------------------------------------------------------------here starts for received lane change request ----------------------------------------------------------------
	if (mergingReqTraj == false){
		if (check_traj_conflict_lane_change(currentTraj, receivedReqTraj, timeGapMerging, receivedEteDelay) == true){
			//double timeGapReduction = 0;
			// if there is a conflict start iteration first for low priority, then medium priority
			if (firstCheckWithLowPriority == true){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with low priority" << std::endl;
				
				if (accelerationRequired == true){
				
					/// @note below added for acceleration
					// first increasing efficiency
					// Iterating over different speed increases and accelerations
					for (double acceleration : accelerationValuesAllPriorities) {
						if (trajectory_found) break;

						for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and acceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);
	
							// Check for conflict
							if (newTrajLaneChangeConflictFree(timeGapMerging))  {
								foundSpeedIncrease = speedIncrease;
								foundAcceleration = acceleration;
								acceleration_trajectory_found = true;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 0;
								break;
							}
						}
					}

					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == false){

						// Iterating over different speed increases, accelerations and Time Gaps
						for (double acceleration : accelerationValuesAllPriorities) {
							if (trajectory_found) break;

							for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsLowPriority) {
									if (trajectory_found) break;

									// Calculate reference trajectory with adjusted speed and acceleration
									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

									// Check for conflict
									if (newTrajLaneChangeConflictFree(timeGapReduction)) {
										foundSpeedIncrease = speedIncrease;
										foundAcceleration = acceleration;
										acceleration_trajectory_found = true;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;  // Found suitable trajectory
										possiblePriorityLevelForAccept = 0;
										break;
									}
								}	
							}
						}
					}

					if (trajectory_found == false){
						// since acc is required first, a new trajectory can be checked with constant speed but reduced time gap
						for (double timeGapReduction : timeGapReductionsLowPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

							// Check for conflict
							if (newTrajLaneChangeConflictFree(timeGapReduction)) {
								foundTimeGap = timeGapReduction;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 0;
								break;
							}
						}	
					}

					// try if lane change is possible 
						if (trajectory_found == false and isLaneChangePossible == true){					
							std::vector<float> cx_new(cx.size());
							std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
							newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);
							if (newTrajLaneChangeConflictFree(timeGapMerging)) {
								trajectory_found  = true;
								lane_changed = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 0;
							}
						}

					// if reduced time gap and lane change are not possible, then try with deceleration
					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				// for decelerating trajectory
				if (decelerationRequired == true){
					// first reducing efficiency
					// Iterating over different speed reductions and decelerations
					for (double deceleration : decelerationValuesLowPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesLowPriority) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and deceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							// Check for conflict
							if (newTrajLaneChangeConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 0;
								break;
							}
						}
					}

					// if not possible with 20 % then check if laneChange is possible
					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == false){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajLaneChangeConflictFree(timeGapMerging)) {
							trajectory_found = true;
							lane_changed = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 0;
						}
					}

					else if (trajectory_found == false &&
							 newTrajLaneChangeHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						// Iterating over different speed reductions and decelerations and Time Gaps
						for (double deceleration : decelerationValuesLowPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesLowPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsLowPriority) {
									if (trajectory_found) break;

									// Calculate reference trajectory with adjusted speed and deceleration
									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

									// Check for conflict
									if (newTrajLaneChangeConflictFree(timeGapReduction)) {
										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;  // Found suitable trajectory
										possiblePriorityLevelForAccept = 0;
										break;
									}
								}	
							}
						}	
					}
				}
			}
			if (secondCheckWithMediumPriority == true and trajectory_found == false){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with medium priority" << std::endl;
				
				if (accelerationRequired == true){

					for (double acceleration : accelerationValuesAllPriorities) {
						if (trajectory_found) break;

						for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
							if (trajectory_found) break;

							// Calculate reference trajectory with adjusted speed and acceleration
							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);
	
							// Check for conflict
							if (newTrajLaneChangeConflictFree(timeGapMerging))  {
								foundSpeedIncrease = speedIncrease;
								foundAcceleration = acceleration;
								acceleration_trajectory_found = true;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								possiblePriorityLevelForAccept = 0;
								break;
							}
						}
					}

					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == false){

						// Iterating over different speed increases, accelerations and Time Gaps
						for (double acceleration : accelerationValuesAllPriorities) {
							if (trajectory_found) break;

							for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsMediumPriority) {
									if (trajectory_found) break;

									// Calculate reference trajectory with adjusted speed and acceleration
									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

									// Check for conflict
									if (newTrajLaneChangeConflictFree(timeGapReduction)) {

										foundSpeedIncrease = speedIncrease;
										foundAcceleration = acceleration;
										acceleration_trajectory_found = true;
										foundTimeGap = timeGapReduction;

										trajectory_found  = true;
										finalTrajectory = newTraj;  // Found suitable trajectory

										if (foundTimeGap < 0.8){
											possiblePriorityLevelForAccept = 1;
										}
										else{
											possiblePriorityLevelForAccept = 0;
										}

										break;
									}
								}	
							}
						}
					}

					if (trajectory_found == false){

						// since acc is required first, a new trajectory can be checked with constant speed but reduced time gap
						for (double timeGapReduction : timeGapReductionsMediumPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

							// Check for conflict
							if (newTrajLaneChangeConflictFree(timeGapReduction)) {

								foundTimeGap = timeGapReduction;
								trajectory_found  = true;
								finalTrajectory = newTraj;  // Found suitable trajectory
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 1;

								break;
							}
						}	
					}

					// try if lane change is possible 
					if (trajectory_found == false and isLaneChangePossible == true and isRouteAffected == true){					

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								

						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajLaneChangeConflictFree(timeGapMerging)) {
							trajectory_found  = true;
							lane_changed = true;
							finalTrajectory = newTraj;  // Found suitable trajectory
							accelerationRequired = false;
							possiblePriorityLevelForAccept = 1;
						}
					}

					// if reduced time gap and lane change are not possible, then try with deceleration
					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				if (decelerationRequired == true){
					// first reducing efficiency
					// Iterating over different speed reductions and decelerations
					for (double deceleration : decelerationValuesMediumPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesMediumPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							if (newTrajLaneChangeConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;
								possiblePriorityLevelForAccept = 1;
								break;
							}
						}
					}

					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == true){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

						if (newTrajLaneChangeConflictFree(timeGapMerging)) {
							trajectory_found  = true;
							lane_changed = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 1;
						}
					}

					else if (trajectory_found == false &&
							 newTrajLaneChangeHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						for (double deceleration : decelerationValuesMediumPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesMediumPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsMediumPriority) {
									if (trajectory_found) break;

									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

									if (newTrajLaneChangeConflictFree(timeGapReduction)) {
										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;
										possiblePriorityLevelForAccept = 1;
										break;
									}
								}	
							}
						}	
					}
				}
			}

			// here for high priority
			if (priority == 2 and trajectory_found == false){
				//std::cout << mVehicleController->getVehicleId() << ": searching for a suitable trajectory with high priority" << std::endl;
				
				if (accelerationRequired == true){

					for (double acceleration : accelerationValuesAllPriorities) {
						if (trajectory_found) break;

						for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);
	
							if (newTrajLaneChangeConflictFree(timeGapMerging)) {
								foundSpeedIncrease = speedIncrease;
								foundAcceleration = acceleration;
								acceleration_trajectory_found = true;
								trajectory_found  = true;
								finalTrajectory = newTraj;
								possiblePriorityLevelForAccept = 0;
								break;
							}
						}
					}

					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == false){

						for (double acceleration : accelerationValuesAllPriorities) {
							if (trajectory_found) break;

							for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsHighPriority) {
									if (trajectory_found) break;

									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

									if (newTrajLaneChangeConflictFree(timeGapReduction)) {
										foundSpeedIncrease = speedIncrease;
										foundAcceleration = acceleration;
										acceleration_trajectory_found = true;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;

										if (foundTimeGap < 0.8 and foundTimeGap > 0.6){
											possiblePriorityLevelForAccept = 1;
										}
										else if (foundTimeGap < 0.6){
											possiblePriorityLevelForAccept = 2;
										}
										else{
											possiblePriorityLevelForAccept = 0;
										}

										break;
									}
								}	
							}
						}
					}

					if (trajectory_found == false){
						for (double timeGapReduction : timeGapReductionsHighPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

							if (newTrajLaneChangeConflictFree(timeGapReduction)) {
								foundTimeGap = timeGapReduction;
								trajectory_found  = true;
								finalTrajectory = newTraj;
								accelerationRequired = false;
								possiblePriorityLevelForAccept = 2;
								break;
							}
						}	
					}

					if (trajectory_found == false and isLaneChangePossible == true and isRouteAffected == true){					
						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
						newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);
						if (newTrajLaneChangeConflictFree(timeGapMerging)) {
							trajectory_found  = true;
							lane_changed = true;
							finalTrajectory = newTraj;
							accelerationRequired = false;
							possiblePriorityLevelForAccept = 2;
						}
					}

					if (trajectory_found == false and isLaneChangePossible == false){
						decelerationRequired = true;
					}
				}

				if (decelerationRequired == true and mVehicleDataProvider->speed()/boost::units::si::meter_per_second > 20.0){

					for (double deceleration : decelerationValuesHighPriority) {
						if (trajectory_found) break;

						for (double speedReduction : speedReductionPercentagesHighPriority) {
							if (trajectory_found) break;

							newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

							if (newTrajLaneChangeConflictFree(timeGapMerging)) {
								foundSpeedReduction = speedReduction;
								foundDeceleration = deceleration;
								trajectory_found  = true;
								finalTrajectory = newTraj;
								possiblePriorityLevelForAccept = 2;
								break;
							}
						}
					}

					if (trajectory_found == false &&
						newTrajLaneChangeHasConflict(timeGapMerging) &&
						isLaneChangePossible == true &&
						isRouteAffected == true){

						foundSpeedReduction = 0.0;
						foundDeceleration = 0.0;

						std::vector<float> cx_new(cx.size());
						std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });								
					newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

					if (newTrajLaneChangeConflictFree(timeGapMerging)) {
						trajectory_found  = true;
						lane_changed = true;
						finalTrajectory = newTraj;
						possiblePriorityLevelForAccept = 2;
					}
				}

					else if (trajectory_found == false &&
							 newTrajLaneChangeHasConflict(timeGapMerging) &&
							 isLaneChangePossible == false){

						for (double deceleration : decelerationValuesHighPriority) {
							if (trajectory_found) break;

							for (double speedReduction : speedReductionPercentagesHighPriority) {
								if (trajectory_found) break;

								for (double timeGapReduction : timeGapReductionsHighPriority) {
									if (trajectory_found) break;

									newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

									if (newTrajLaneChangeConflictFree(timeGapReduction)) {
										foundSpeedReduction = speedReduction;
										foundDeceleration = deceleration;
										foundTimeGap = timeGapReduction;
										trajectory_found  = true;
										finalTrajectory = newTraj;
										possiblePriorityLevelForAccept = 2;
										break;
									}
								}	
							}
						}	
					}
				}
			}
		}	
		else {
			trajectory_found = true;
			finalTrajectory = currentTraj;  // Found suitable trajectory
			possiblePriorityLevelForAccept = 0;
			//std::cout << mVehicleController->getVehicleId() << " there is no conflict with received Req Traj " << std::endl;
		}
		
	}
	/// @note  -----------------------------------------------------------------------here ends for received lane change request ---------------------------------------------------------------------
		if (trajectory_found == true){
		if (acceleration_trajectory_found == true){
			plannedTrajValues.speed_change = foundSpeedIncrease;
			std::cout << mVehicleController->getVehicleId() << " ------------ CV found speed increase is: " << foundSpeedIncrease << std::endl; 
		}
		else {
			plannedTrajValues.speed_change = foundSpeedReduction;
			std::cout << mVehicleController->getVehicleId() << " ------------ CV found speed reduction is: " << foundSpeedReduction << std::endl; 
		}

		plannedTrajValues.acc_change = foundAcceleration;
		plannedTrajValues.deceleration_change = foundDeceleration;
		plannedTrajValues.lane_change = lane_changed;
		plannedTrajValues.time_gap_change = foundTimeGap;
		plannedTrajValues.TTC_change = 3.0; 

		foundTrajectoryCost = calculateTrajectoryCost(plannedTrajValues, false);	
		
		if (plannedTrajValues.speed_change == 0.0 and plannedTrajValues.acc_change == 0.0 and plannedTrajValues.deceleration_change == 0.0 and plannedTrajValues.lane_change == false and plannedTrajValues.time_gap_change >= 1.0){
			typeOfTrajectory = 0;
		}
		else if (plannedTrajValues.speed_change == 0.0 and plannedTrajValues.acc_change == 0.0 and plannedTrajValues.deceleration_change == 0.0 and plannedTrajValues.lane_change == false and plannedTrajValues.time_gap_change < 1.0){
			typeOfTrajectory = 4;
		}
		else if (plannedTrajValues.deceleration_change > 0.0){
			typeOfTrajectory = 1;
		}
		else if (plannedTrajValues.lane_change == true){
			typeOfTrajectory = 2;
		}	
		else if (acceleration_trajectory_found == true and plannedTrajValues.time_gap_change >= 1.0){
			typeOfTrajectory = 5;
		}	
		else if (acceleration_trajectory_found == true and plannedTrajValues.time_gap_change < 1.0){
			typeOfTrajectory = 6;
		}	

		std::cout << mVehicleController->getVehicleId() << " ------------ CV trajectory possible to accept with cost values for priority level: " << possiblePriorityLevelForAccept << std::endl; 
		
		std::cout << mVehicleController->getVehicleId() << " ------------ CV trajectory found with following values: " 
			<< " speed change: " << plannedTrajValues.speed_change
			<< " acceleration: " << plannedTrajValues.acc_change
			<< " deceleration: " << plannedTrajValues.deceleration_change
			<< " lane changed:" << plannedTrajValues.lane_change
			<< " time gap: " << plannedTrajValues.time_gap_change
			<< " TTC: " << plannedTrajValues.TTC_change
			<< " ------- TOTAL TRAJECTORY COST: " << foundTrajectoryCost
			<< std::endl; 
	}

 	return std::make_tuple(trajectory_found, finalTrajectory, plannedTrajValues, foundTrajectoryCost, typeOfTrajectory, possiblePriorityLevelForAccept);
}		

double TrajectoryPlanner::calculateTrajectoryCost(PlannedTrajValues plannedTrajValues, bool isRouteAffected)
{
	double speedCost = plannedTrajValues.speed_change;
	double accCost = 0.0;
	double laneChangeCost = 0.0;
	double timeGapCost = 0.0;
	double ttcCost = 0.0;

	if (plannedTrajValues.deceleration_change > 0.0) {
		accCost = plannedTrajValues.deceleration_change / 10.0;
	}
	else if (plannedTrajValues.acc_change > 0.0) {
		accCost = plannedTrajValues.acc_change / 10.0;
	}

	if (plannedTrajValues.lane_change == true) {
		laneChangeCost = isRouteAffected ? 0.4 : 0.2;
	}

	if (plannedTrajValues.time_gap_change < 1.0) {
		timeGapCost = 1.0 - plannedTrajValues.time_gap_change;
	}

	const double trajectoryCost = (speedCost + accCost + laneChangeCost + timeGapCost + ttcCost) / 5.0;

	return trajectoryCost;
}


TrajectoryPlanner::Trajectory TrajectoryPlanner::calculateExecuteTrajectory(
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	bool constantVelocity,
	float targetSpeedNeg,
	float accNeg,
	float decelerationNeg,
	Trajectory negotiatedTrajectory,
	float targetSpeedAfterNeg,
	float targetAccAfterNeg,
	float targetDecelerationAfterNeg)
{
	Trajectory trajectory;
	TrajPointMCM calculatedPoint;
	State state;

	state.x = mVehicleController->getPositionSumo().x / boost::units::si::meter;
	state.y = mVehicleController->getPositionSumo().y / boost::units::si::meter;
	state.yaw = mVehicleDataProvider->heading() / boost::units::si::radians;
	state.v = mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

	const int ncourse = static_cast<int>(std::min(cx.size(), cy.size()));

	if (ncourse == 0 || steps <= 0 || dt <= 0.0) {
		return trajectory;
	}

	int ind = calc_nearest_index(state, cx, cy, mIndex);

	if (mIndex >= ind) {
		ind = mIndex;
	}

	const int indNegTrajLastPoint = calc_nearest_index_NegTraj(negotiatedTrajectory, cx, cy, mIndex);

	double travel = 0.0;
	const double dl = 1.0;

	for (int i = 0; i < steps; i++) {

		const float old_v = state.v;
		const int dindBeforeSpeedUpdate = static_cast<int>(std::round(travel / dl));
		const bool afterNegotiatedTrajectory = (ind + dindBeforeSpeedUpdate) >= indNegTrajLastPoint;

		if (!constantVelocity && i > 0) {
			const float targetSpeed = afterNegotiatedTrajectory ? targetSpeedAfterNeg : targetSpeedNeg;
			const float acceleration = afterNegotiatedTrajectory ? targetAccAfterNeg : accNeg;
			const float deceleration = afterNegotiatedTrajectory ? targetDecelerationAfterNeg : decelerationNeg;

			if (state.v > targetSpeed) {
				state.v -= deceleration * dt;

				if (state.v < targetSpeed) {
					state.v = targetSpeed;
				}
			}
			else if (state.v < targetSpeed) {
				state.v += acceleration * dt;

				if (state.v > targetSpeed) {
					state.v = targetSpeed;
				}
			}
		}

		travel += 0.5 * (std::abs(old_v) + std::abs(state.v)) * dt;

		const int dind = static_cast<int>(std::round(travel / dl));
		const int targetIndex = std::min(ind + dind, ncourse - 1);

		calculatedPoint.mX = cx[targetIndex];
		calculatedPoint.mY = cy[targetIndex];
		calculatedPoint.mHeading = mVehicleDataProvider->heading() / boost::units::si::radians;
		calculatedPoint.mTime = simTime() + (i + 1) * dt;

		trajectory.push_back(getPoint(calculatedPoint));
	}

	mIndex = ind;

	return trajectory;
}

int TrajectoryPlanner::calc_nearest_index_NegTraj(
	Trajectory negotiatedTrajectory,
	const Vec_f& cx,
	const Vec_f& cy,
	int pind)
{
	if (negotiatedTrajectory.empty() || cx.empty() || cy.empty()) {
		return 0;
	}

	const int startIndex = std::max(0, pind);
	const int endIndex = static_cast<int>(std::min(cx.size(), cy.size()));

	if (startIndex >= endIndex) {
		return endIndex - 1;
	}

	int nearestIndex = startIndex;
	float minDistance = std::numeric_limits<float>::max();

	const auto& lastPoint = negotiatedTrajectory.back();

	for (int i = startIndex; i < endIndex; i++) {
		const float dx = cx[i] - lastPoint.mX;
		const float dy = cy[i] - lastPoint.mY;
		const float distance = dx * dx + dy * dy;

		if (distance < minDistance) {
			minDistance = distance;
			nearestIndex = i;
		}
	}

	return nearestIndex;
}

TrajectoryPlanner::Trajectory TrajectoryPlanner::estimateOtherDecelerationTrajectory(
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	bool constantVelocity,
	float targetSpeed,
	float acceleration,
	float deceleration,
	Trajectory traj_other)
{
	Trajectory trajectory;

	if (steps <= 0 || dt <= 0.0 || cx.empty() || cy.empty() || traj_other.size() < 2) {
		return trajectory;
	}

	TrajPointMCM calculatedPoint;
	State state;

	const double timeDiff = (traj_other[1].mTime - traj_other[0].mTime).dbl();

	if (timeDiff <= 0.0) {
		return trajectory;
	}

	state.v = std::hypot(
		traj_other[1].mX - traj_other[0].mX,
		traj_other[1].mY - traj_other[0].mY
	) / timeDiff;

	state.x = traj_other[0].mX;
	state.y = traj_other[0].mY + state.v * dt;
	state.yaw = traj_other[0].mHeading;

	Vec_f cx_new(cx.size());
	std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

	const int ncourse = static_cast<int>(std::min(cx_new.size(), cy.size()));

	int ind = calc_nearest_index(state, cx_new, cy, mIndex);

	if (mIndex >= ind) {
		ind = mIndex;
	}

	double travel = 0.0;
	const double dl = 1.0;

	for (int i = 0; i < steps; i++) {
		const float old_v = state.v;

		if (!constantVelocity && i > 0) {
			if (state.v > targetSpeed) {
				state.v -= deceleration * dt;

				if (state.v < targetSpeed) {
					state.v = targetSpeed;
				}
			}
			else if (state.v < targetSpeed) {
				state.v += acceleration * dt;

				if (state.v > targetSpeed) {
					state.v = targetSpeed;
				}
			}
		}

		travel += 0.5 * (std::abs(old_v) + std::abs(state.v)) * dt;

		const int dind = static_cast<int>(std::round(travel / dl));
		const int targetIndex = std::min(ind + dind, ncourse - 1);

		calculatedPoint.mX = cx_new[targetIndex];
		calculatedPoint.mY = cy[targetIndex];
		calculatedPoint.mHeading = mVehicleDataProvider->heading() / boost::units::si::radians;
		calculatedPoint.mTime = simTime() + (i + 1) * dt;

		trajectory.push_back(getPoint(calculatedPoint));
	}

	return trajectory;
}

int TrajectoryPlanner::findIndexTrajsConflictFree(
	Trajectory traj_ego,
	Trajectory traj_other,
	float time_gap,
	const Vec_f& cx,
	const Vec_f& cy,
	int pind)
{
	if (traj_ego.empty() || traj_other.empty() || cx.empty() || cy.empty()) {
		return 0;
	}

	const float current_speed =
		mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

	const float min_distance = time_gap * current_speed;

	const int trajSize = static_cast<int>(std::min(traj_ego.size(), traj_other.size()));
	const int startIndex = std::max(0, pind);
	const int endIndex = static_cast<int>(std::min(cx.size(), cy.size()));

	const auto nearestRouteIndex = [&](const TrajPointMCM& point) {
		if (startIndex >= endIndex) {
			return endIndex - 1;
		}

		int nearestIndex = startIndex;
		float minDistance = std::numeric_limits<float>::max();

		for (int i = startIndex; i < endIndex; ++i) {
			const float dx = cx[i] - point.mX;
			const float dy = cy[i] - point.mY;
			const float distance = dx * dx + dy * dy;

			if (distance < minDistance) {
				minDistance = distance;
				nearestIndex = i;
			}
		}

		return nearestIndex;
	};

	for (int t = 0; t < trajSize; ++t) {
		const float dx = traj_ego[t].mX - traj_other[t].mX;
		const float dy = traj_ego[t].mY - traj_other[t].mY;
		const float dist = std::hypot(dx, dy);

		if (dist >= min_distance) {
			// calculateSecondReqTraj compares this value with route indices, not trajectory sample indices.
			return nearestRouteIndex(traj_ego[t]);
		}
	}

	const TrajPointMCM& lastEgoPoint = traj_ego[trajSize - 1];

	if (startIndex >= endIndex) {
		return endIndex - 1;
	}

	int nearestIndex = startIndex;
	float minDistance = std::numeric_limits<float>::max();

	for (int i = startIndex; i < endIndex; ++i) {
		const float dx = cx[i] - lastEgoPoint.mX;
		const float dy = cy[i] - lastEgoPoint.mY;
		const float distance = dx * dx + dy * dy;

		if (distance < minDistance) {
			minDistance = distance;
			nearestIndex = i;
		}
	}

	return nearestIndex;
}

TrajectoryPlanner::TupleSecondRequestTrajRv TrajectoryPlanner::findSecondRequestTrajRV(
	Trajectory estimatedOtherTraj,
	int requestPriority,
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	double current_speed,
	omnetpp::SimTime receivedEteDelay,
	bool decelerationRequired,
	bool accelerationRequired)
{
	bool trajectory_found = false;
	bool acceleration_trajectory_found = false;

	const float timeGapMerging = 1.0;

	Trajectory currentTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, 0.0, 0.0, 0.0);
	Trajectory finalTrajectory = currentTraj;
	Trajectory newTraj;
	// Empty candidates mean planning failed; do not let conflict helpers classify them as acceptable.
	auto newTrajLaneChangeHasConflict = [&](float timeGap) {
		return !newTraj.empty() &&
			check_traj_conflict_lane_change(newTraj, estimatedOtherTraj, timeGap, receivedEteDelay);
	};
	auto newTrajLaneChangeConflictFree = [&](float timeGap) {
		return !newTraj.empty() &&
			!check_traj_conflict_lane_change(newTraj, estimatedOtherTraj, timeGap, receivedEteDelay);
	};

	PlannedTrajValues plannedTrajValues{0.0, 0.0, 0.0, false, 1.0, 3.0};
	double foundTrajectoryCost = 0.0;
	int typeOfTrajectory = 0;
	int possiblePriorityLevelForAccept = 10;

	std::vector<double> speedReductionPercentagesHighPriority = {0.1, 0.2, 0.3, 0.4, 0.45, 0.5, 0.55, 0.6};
	std::vector<double> decelerationValuesHighPriority = {1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
	std::vector<double> timeGapReductionsHighPriority = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};

	std::vector<double> speedIncreasePercentagesAllPriorities = {0.05, 0.1, 0.15, 0.2};
	std::vector<double> accelerationValuesAllPriorities = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};

	double foundSpeedIncrease = 0.0;
	double foundSpeedReduction = 0.0;
	double foundAcceleration = 0.0;
	double foundDeceleration = 0.0;
	double foundTimeGap = 1.0;

	bool lane_changed = false;
	int indexToChangeLane = mIndex;

	if (accelerationRequired == true) {
		for (double acceleration : accelerationValuesAllPriorities) {
			if (trajectory_found) break;

			for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
				if (trajectory_found) break;

				newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

				indexToChangeLane = findIndexTrajsConflictFree(newTraj, estimatedOtherTraj, timeGapMerging, cx, cy, mIndex);

				newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0, indexToChangeLane);

				if (newTrajLaneChangeConflictFree(timeGapMerging)) {
					foundSpeedIncrease = speedIncrease;
					foundAcceleration = acceleration;
					acceleration_trajectory_found = true;
					trajectory_found = true;
					finalTrajectory = newTraj;
					possiblePriorityLevelForAccept = 0;
					break;
				}
			}
		}

		if (trajectory_found == false) {
			for (double acceleration : accelerationValuesAllPriorities) {
				if (trajectory_found) break;

				for (double speedIncrease : speedIncreasePercentagesAllPriorities) {
					if (trajectory_found) break;

					for (double timeGapReduction : timeGapReductionsHighPriority) {
						if (trajectory_found) break;

						newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0);

						indexToChangeLane = findIndexTrajsConflictFree(newTraj, estimatedOtherTraj, timeGapReduction, cx, cy, mIndex);

						newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, false, current_speed + speedIncrease * current_speed, acceleration, 0.0, indexToChangeLane);

						if (newTrajLaneChangeConflictFree(timeGapReduction)) {
							foundSpeedIncrease = speedIncrease;
							foundAcceleration = acceleration;
							foundTimeGap = timeGapReduction;
							acceleration_trajectory_found = true;
							trajectory_found = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 0;
							break;
						}
					}
				}
			}
		}

		if (trajectory_found == false) {
			for (double timeGapReduction : timeGapReductionsHighPriority) {
				if (trajectory_found) break;

				indexToChangeLane = findIndexTrajsConflictFree(currentTraj, estimatedOtherTraj, timeGapReduction, cx, cy, mIndex);

				newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0, indexToChangeLane);

				if (newTrajLaneChangeConflictFree(timeGapReduction)) {
					foundTimeGap = timeGapReduction;
					trajectory_found = true;
					finalTrajectory = newTraj;
					accelerationRequired = false;
					possiblePriorityLevelForAccept = 2;
					break;
				}
			}
		}

		if (trajectory_found == false) {
			decelerationRequired = true;
		}
	}

	if (decelerationRequired == true) {
		for (double deceleration : decelerationValuesHighPriority) {
			if (trajectory_found) break;

			for (double speedReduction : speedReductionPercentagesHighPriority) {
				if (trajectory_found) break;

				newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

				indexToChangeLane = findIndexTrajsConflictFree(newTraj, estimatedOtherTraj, timeGapMerging, cx, cy, mIndex);

				newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration, indexToChangeLane);

				if (newTrajLaneChangeConflictFree(timeGapMerging)) {
					foundSpeedReduction = speedReduction;
					foundDeceleration = deceleration;
					trajectory_found = true;
					finalTrajectory = newTraj;
					possiblePriorityLevelForAccept = 2;
					break;
				}
			}
		}

		if (trajectory_found == false && newTrajLaneChangeHasConflict(timeGapMerging)) {
			for (double deceleration : decelerationValuesHighPriority) {
				if (trajectory_found) break;

				for (double speedReduction : speedReductionPercentagesHighPriority) {
					if (trajectory_found) break;

					for (double timeGapReduction : timeGapReductionsHighPriority) {
						if (trajectory_found) break;

						newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration);

						indexToChangeLane = findIndexTrajsConflictFree(newTraj, estimatedOtherTraj, timeGapReduction, cx, cy, mIndex);

						newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, false, current_speed - speedReduction * current_speed, 0.0, deceleration, indexToChangeLane);

						if (newTrajLaneChangeConflictFree(timeGapReduction)) {
							foundSpeedReduction = speedReduction;
							foundDeceleration = deceleration;
							foundTimeGap = timeGapReduction;
							trajectory_found = true;
							finalTrajectory = newTraj;
							possiblePriorityLevelForAccept = 2;
							break;
						}
					}
				}
			}
		}
	}

	if (trajectory_found == true) {
		if (acceleration_trajectory_found == true) {
			plannedTrajValues.speed_change = foundSpeedIncrease;
		}
		else {
			plannedTrajValues.speed_change = foundSpeedReduction;
		}

		plannedTrajValues.acc_change = foundAcceleration;
		plannedTrajValues.deceleration_change = foundDeceleration;
		plannedTrajValues.lane_change = lane_changed;
		plannedTrajValues.time_gap_change = foundTimeGap;
		plannedTrajValues.TTC_change = 3.0;

		foundTrajectoryCost = calculateTrajectoryCost(plannedTrajValues, false);

		if (plannedTrajValues.speed_change == 0.0 && plannedTrajValues.acc_change == 0.0 && plannedTrajValues.deceleration_change == 0.0 && plannedTrajValues.lane_change == false && plannedTrajValues.time_gap_change >= 1.0) {
			typeOfTrajectory = 0;
		}
		else if (plannedTrajValues.speed_change == 0.0 && plannedTrajValues.acc_change == 0.0 && plannedTrajValues.deceleration_change == 0.0 && plannedTrajValues.lane_change == false && plannedTrajValues.time_gap_change < 1.0) {
			typeOfTrajectory = 4;
		}
		else if (plannedTrajValues.deceleration_change > 0.0) {
			typeOfTrajectory = 1;
		}
		else if (plannedTrajValues.lane_change == true) {
			typeOfTrajectory = 2;
		}
		else if (acceleration_trajectory_found == true && plannedTrajValues.time_gap_change >= 1.0) {
			typeOfTrajectory = 5;
		}
		else if (acceleration_trajectory_found == true && plannedTrajValues.time_gap_change < 1.0) {
			typeOfTrajectory = 6;
		}

		std::cout << mVehicleController->getVehicleId() << " RV trajectory found with following values: "
			<< " speed change: " << plannedTrajValues.speed_change
			<< " acceleration: " << plannedTrajValues.acc_change
			<< " deceleration: " << plannedTrajValues.deceleration_change
			<< " lane changed: " << plannedTrajValues.lane_change
			<< " time gap: " << plannedTrajValues.time_gap_change
			<< " TTC: " << plannedTrajValues.TTC_change
			<< " ---- Total RV second traj cost: " << foundTrajectoryCost
			<< std::endl;

		std::cout << mVehicleController->getVehicleId()
			<< " trajectory possible to accept with cost values for priority level: "
			<< possiblePriorityLevelForAccept << std::endl;
	}
	else if (trajectory_found == false && decelerationRequired == true) {
		const double decelerationFinal = 4.0;
		const double speedReductionFinal = 0.4;

		foundDeceleration = decelerationFinal;
		foundSpeedReduction = speedReductionFinal;

		newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, current_speed - speedReductionFinal * current_speed, 0.0, decelerationFinal);

		indexToChangeLane = findIndexTrajsConflictFree(newTraj, estimatedOtherTraj, timeGapMerging, cx, cy, mIndex);

		newTraj = calculateSecondReqTraj(steps, dt, cx, cy, mIndex, false, current_speed - speedReductionFinal * current_speed, 0.0, decelerationFinal, indexToChangeLane);

		finalTrajectory = newTraj;

		plannedTrajValues.speed_change = foundSpeedReduction;
		plannedTrajValues.acc_change = foundAcceleration;
		plannedTrajValues.deceleration_change = foundDeceleration;
		plannedTrajValues.lane_change = lane_changed;
		plannedTrajValues.time_gap_change = foundTimeGap;
		plannedTrajValues.TTC_change = 3.0;

		foundTrajectoryCost = calculateTrajectoryCost(plannedTrajValues, false);

		if (plannedTrajValues.deceleration_change > 0.0) {
			typeOfTrajectory = 1;
		}
		else if (plannedTrajValues.speed_change == 0.0 && plannedTrajValues.acc_change == 0.0 && plannedTrajValues.deceleration_change == 0.0 && plannedTrajValues.lane_change == false && plannedTrajValues.time_gap_change < 1.0) {
			typeOfTrajectory = 4;
		}

		std::cout << mVehicleController->getVehicleId() << " -------RV trajectory NOT found but sent with following values: "
			<< " speed reduction: " << plannedTrajValues.speed_change
			<< " acceleration: " << plannedTrajValues.acc_change
			<< " deceleration: " << plannedTrajValues.deceleration_change
			<< " lane changed: " << plannedTrajValues.lane_change
			<< " time gap: " << plannedTrajValues.time_gap_change
			<< " TTC: " << plannedTrajValues.TTC_change
			<< " -----RV second traj cost: " << foundTrajectoryCost
			<< std::endl;
	}

	return std::make_tuple(trajectory_found, finalTrajectory, plannedTrajValues, foundTrajectoryCost, typeOfTrajectory);
}

TrajectoryPlanner::Trajectory TrajectoryPlanner::calculateSecondReqTraj(
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	bool constantVelocity,
	float targetSpeed,
	float acceleration,
	float deceleration,
	int indexToChangeLane)
{
	Trajectory trajectory;

	if (steps <= 0 || cx.empty() || cy.empty()) {
		return trajectory;
	}

	TrajPointMCM calculatedPoint;
	State state;

	state.x = mVehicleController->getPositionSumo().x / boost::units::si::meter;
	state.y = mVehicleController->getPositionSumo().y / boost::units::si::meter;
	state.yaw = mVehicleDataProvider->heading() / boost::units::si::radians;
	state.v = mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

	Vec_f cx_new(cx.size());
	std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });

	const int ncourse = std::min(cx.size(), cy.size());

	int ind = calc_nearest_index(state, cx, cy, mIndex);

	if (mIndex >= ind) {
		ind = mIndex;
	}

	double travel = 0.0;
	const double dl = 1.0;

	for (int i = 0; i < steps; i++) {

		// store previous velocity before updating
		const float old_v = state.v;

		if (!constantVelocity && i > 0) {

			// decelerate
			if (state.v > targetSpeed) {
				state.v -= deceleration * dt;

				if (state.v < targetSpeed) {
					state.v = targetSpeed;
				}
			}

			// accelerate
			else if (state.v < targetSpeed) {
				state.v += acceleration * dt;

				if (state.v > targetSpeed) {
					state.v = targetSpeed;
				}
			}
		}

		// integrate traveled distance using average velocity
		travel += 0.5 * (std::abs(old_v) + std::abs(state.v)) * dt;

		const int dind = static_cast<int>(std::round(travel / dl));
		const int targetIndex = std::min(ind + dind, ncourse - 1);

		// perform lane change after selected index
		if (targetIndex < indexToChangeLane + 2) {
			calculatedPoint.mX = cx[targetIndex];
		}
		else {
			calculatedPoint.mX = cx_new[targetIndex];
		}

		calculatedPoint.mY = cy[targetIndex];

		calculatedPoint.mHeading =
			mVehicleDataProvider->heading() /
			boost::units::si::radians;

		calculatedPoint.mTime =
			simTime() + (i + 1) * dt;

		trajectory.push_back(getPoint(calculatedPoint));
	}

	mIndex = ind;

	return trajectory;
}

TrajectoryPlanner::TupleSuitableTrajectory TrajectoryPlanner::newPlannedTrajRV(
	Trajectory receivedConflictedTraj,
	int requestPriority,
	int steps,
	double dt,
	const Vec_f& cx,
	const Vec_f& cy,
	int mIndex,
	double current_speed,
	omnetpp::SimTime receivedEteDelay,
	bool decelerationRequired,
	bool accelerationRequired)
{
	bool trajectory_found = false;
	const float timeGapMerging = 1.0F;

	Trajectory currentTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, 0.0, 0.0, 0.0);
	Trajectory finalTrajectory = currentTraj;
	Trajectory newTraj;
	// Empty candidates mean planning failed; do not let conflict helpers classify them as acceptable.
	auto newTrajLaneChangeHasConflict = [&](float timeGap) {
		return !newTraj.empty() &&
			check_traj_conflict_lane_change(newTraj, receivedConflictedTraj, timeGap, receivedEteDelay);
	};
	auto newTrajLaneChangeConflictFree = [&](float timeGap) {
		return !newTraj.empty() &&
			!check_traj_conflict_lane_change(newTraj, receivedConflictedTraj, timeGap, receivedEteDelay);
	};

	PlannedTrajValues plannedTrajValues{0.0, 0.0, 0.0, false, 1.0, 3.0};
	double foundTrajectoryCost = 0.0;

	int typeOfTrajectory = 0;
	int possiblePriorityLevelForAccept = 10;

	std::vector<double> speedReductionPercentagesHighPriority = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
	std::vector<double> decelerationValuesHighPriority = {1.0, 2.0, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0};
	std::vector<double> timeGapReductionsHighPriority = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

	double foundSpeedReduction = 0.0;
	double foundAcceleration = 0.0;
	double foundDeceleration = 0.0;
	double foundTimeGap = 1.0;
	bool lane_changed = false;

	if (accelerationRequired == true) {
		for (double timeGapReduction : timeGapReductionsHighPriority) {
			if (trajectory_found) break;

			newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, true, current_speed, 0.0, 0.0);

			if (newTrajLaneChangeConflictFree(timeGapReduction)) {
				foundTimeGap = timeGapReduction;
				trajectory_found = true;
				finalTrajectory = newTraj;
				accelerationRequired = false;
				break;
			}
		}

		if (trajectory_found == false) {
			decelerationRequired = true;
		}
	}

	if (decelerationRequired == true) {
		for (double deceleration : decelerationValuesHighPriority) {
			if (trajectory_found) break;

			for (double speedReduction : speedReductionPercentagesHighPriority) {
				if (trajectory_found) break;

				const double targetSpeed = current_speed - speedReduction * current_speed;

				newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, targetSpeed, 0.0, deceleration);

				if (newTrajLaneChangeConflictFree(timeGapMerging)) {
					foundSpeedReduction = speedReduction;
					foundDeceleration = deceleration;
					trajectory_found = true;
					finalTrajectory = newTraj;
					break;
				}
			}
		}

		if (trajectory_found == false && newTrajLaneChangeHasConflict(timeGapMerging)) {
			for (double deceleration : decelerationValuesHighPriority) {
				if (trajectory_found) break;

				for (double speedReduction : speedReductionPercentagesHighPriority) {
					if (trajectory_found) break;

					for (double timeGapReduction : timeGapReductionsHighPriority) {
						if (trajectory_found) break;

						const double targetSpeed = current_speed - speedReduction * current_speed;

						newTraj = calculateRefTrajectory(steps, dt, cx, cy, mIndex, false, targetSpeed, 0.0, deceleration);

						if (newTrajLaneChangeConflictFree(timeGapReduction)) {
							foundSpeedReduction = speedReduction;
							foundDeceleration = deceleration;
							foundTimeGap = timeGapReduction;
							trajectory_found = true;
							finalTrajectory = newTraj;
							break;
						}
					}
				}
			}
		}
	}

	if (trajectory_found == true) {
		plannedTrajValues.speed_change = foundSpeedReduction;
		plannedTrajValues.acc_change = foundAcceleration;
		plannedTrajValues.deceleration_change = foundDeceleration;
		plannedTrajValues.lane_change = lane_changed;
		plannedTrajValues.time_gap_change = foundTimeGap;
		plannedTrajValues.TTC_change = 3.0;

		if (foundSpeedReduction <= 0.2) {
			possiblePriorityLevelForAccept = 0;
		}
		else if (foundSpeedReduction <= 0.4) {
			possiblePriorityLevelForAccept = 1;
		}
		else {
			possiblePriorityLevelForAccept = 2;
		}

		foundTrajectoryCost = calculateTrajectoryCost(plannedTrajValues, false);

		if (plannedTrajValues.speed_change == 0.0 && plannedTrajValues.acc_change == 0.0 && plannedTrajValues.deceleration_change == 0.0 && plannedTrajValues.lane_change == false && plannedTrajValues.time_gap_change >= 1.0) {
			typeOfTrajectory = 0;
		}
		else if (plannedTrajValues.speed_change == 0.0 && plannedTrajValues.acc_change == 0.0 && plannedTrajValues.deceleration_change == 0.0 && plannedTrajValues.lane_change == false && plannedTrajValues.time_gap_change < 1.0) {
			typeOfTrajectory = 4;
		}
		else if (plannedTrajValues.deceleration_change > 0.0) {
			typeOfTrajectory = 1;
		}
		else if (plannedTrajValues.lane_change == true) {
			typeOfTrajectory = 2;
		}
	}
	else if (decelerationRequired == true) {
		std::cout << mVehicleController->getVehicleId()
			<< " -------RV trajectory NOT found - emergency deceleration required"
			<< std::endl;
	}

	return std::make_tuple(trajectory_found, finalTrajectory, plannedTrajValues, foundTrajectoryCost, typeOfTrajectory, possiblePriorityLevelForAccept);
}


std::vector<std::string> TrajectoryPlanner::splitString(const std::string& line, char delimiter)
{
	return artery::mcm::splitString(line, delimiter);
}

std::vector<float> TrajectoryPlanner::readCX(const std::string& filename, char delimiter, const std::string& column_name)
{
	return artery::mcm::readCX(filename, delimiter, column_name);
}

bool TrajectoryPlanner::checkColumnExists(const std::string& filename, const std::string& columnName, char delimiter)
{
	return artery::mcm::checkColumnExists(filename, columnName, delimiter);
}

} // namespace mcm
} // namespace artery
