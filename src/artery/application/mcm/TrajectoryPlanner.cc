#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/application/mcm/TrajectoryGeneration.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace artery
{

using namespace omnetpp;

namespace mcm
{

namespace
{

int classifyTrajectoryType(const PlannedTrajValues& plannedTrajValues, bool accelerationTrajectoryFound)
{
	if (plannedTrajValues.speed_change == 0.0 and plannedTrajValues.acc_change == 0.0 and plannedTrajValues.deceleration_change == 0.0 and plannedTrajValues.lane_change == false and plannedTrajValues.time_gap_change >= 1.0){
		return 0;
	}
	else if (plannedTrajValues.speed_change == 0.0 and plannedTrajValues.acc_change == 0.0 and plannedTrajValues.deceleration_change == 0.0 and plannedTrajValues.lane_change == false and plannedTrajValues.time_gap_change < 1.0){
		return 4;
	}
	else if (plannedTrajValues.deceleration_change > 0.0){
		return 1;
	}
	else if (plannedTrajValues.lane_change == true){
		return 2;
	}
	else if (accelerationTrajectoryFound == true and plannedTrajValues.time_gap_change >= 1.0){
		return 5;
	}
	else if (accelerationTrajectoryFound == true and plannedTrajValues.time_gap_change < 1.0){
		return 6;
	}

	return 0;
}

void finalizeSuitableTrajectoryCvResult(
	TrajectoryPlanner& planner,
	bool accelerationTrajectoryFound,
	double foundSpeedIncrease,
	double foundSpeedReduction,
	double foundAcceleration,
	double foundDeceleration,
	bool laneChanged,
	double foundTimeGap,
	int possiblePriorityLevelForAccept,
	PlannedTrajValues& plannedTrajValues,
	double& foundTrajectoryCost,
	int& typeOfTrajectory)
{
	if (accelerationTrajectoryFound == true){
		plannedTrajValues.speed_change = foundSpeedIncrease;
		std::cout << planner.mVehicleController->getVehicleId() << " ------------ CV found speed increase is: " << foundSpeedIncrease << std::endl;
	}
	else {
		plannedTrajValues.speed_change = foundSpeedReduction;
		std::cout << planner.mVehicleController->getVehicleId() << " ------------ CV found speed reduction is: " << foundSpeedReduction << std::endl;
	}

	plannedTrajValues.acc_change = foundAcceleration;
	plannedTrajValues.deceleration_change = foundDeceleration;
	plannedTrajValues.lane_change = laneChanged;
	plannedTrajValues.time_gap_change = foundTimeGap;
	plannedTrajValues.TTC_change = 3.0;

	foundTrajectoryCost = planner.calculateTrajectoryCost(plannedTrajValues, false);

	typeOfTrajectory = classifyTrajectoryType(plannedTrajValues, accelerationTrajectoryFound);

	std::cout << planner.mVehicleController->getVehicleId() << " ------------ CV trajectory possible to accept with cost values for priority level: " << possiblePriorityLevelForAccept << std::endl;

	std::cout << planner.mVehicleController->getVehicleId() << " ------------ CV trajectory found with following values: "
		<< " speed change: " << plannedTrajValues.speed_change
		<< " acceleration: " << plannedTrajValues.acc_change
		<< " deceleration: " << plannedTrajValues.deceleration_change
		<< " lane changed:" << plannedTrajValues.lane_change
		<< " time gap: " << plannedTrajValues.time_gap_change
		<< " TTC: " << plannedTrajValues.TTC_change
		<< " ------- TOTAL TRAJECTORY COST: " << foundTrajectoryCost
		<< std::endl;
}

} // namespace

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
	// Reserved for future TTC-based adaptation; currently not used in the trajectory search.
	std::vector<double> timeToCollisionReductionsLowPriority = {2.9, 2.8};

	// Define parameters for iteration for medium priority up to 40% change of plan
	std::vector<double> speedReductionPercentagesMediumPriority = {0.25, 0.3, 0.35, 0.4};
	std::vector<double> decelerationValuesMediumPriority = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
	std::vector<double> timeGapReductionsMediumPriority = {1.0, 0.9, 0.7, 0.6};
	// Reserved for future TTC-based adaptation; currently not used in the trajectory search.
	std::vector<double> timeToCollisionReductionsMediumPriority = {2.9, 2.8, 2.7, 2.6};
	
	// Define parameters for iteration for high priority up to 60% change of plan
	std::vector<double> speedReductionPercentagesHighPriority = {0.45, 0.5, 0.55, 0.6};
	std::vector<double> decelerationValuesHighPriority = {3.5, 4.0, 4.5, 5.0};
	std::vector<double> timeGapReductionsHighPriority = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
	// Reserved for future TTC-based adaptation; currently not used in the trajectory search.
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
	auto tryShiftedLaneFallback = [&](auto conflictFree, int priorityLevel, bool clearAccelerationRequired) {
		std::vector<float> cx_new = makeShiftedLaneCx(cx);
		newTraj = calculateRefTrajectory(steps, dt, cx_new, cy, mIndex, true, current_speed, 0.0, 0.0);

		if (conflictFree(timeGapMerging)) {
			trajectory_found = true;
			lane_changed = true;
			finalTrajectory = newTraj;
			if (clearAccelerationRequired) {
				accelerationRequired = false;
			}
			possiblePriorityLevelForAccept = priorityLevel;
		}
	};
	
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
						tryShiftedLaneFallback(newTrajMergingConflictFree, 0, true);
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

						tryShiftedLaneFallback(newTrajMergingConflictFree, 0, false);
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
						tryShiftedLaneFallback(newTrajMergingConflictFree, 1, true);
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

						tryShiftedLaneFallback(newTrajMergingConflictFree, 1, false);
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
						tryShiftedLaneFallback(newTrajMergingConflictFree, 2, true);
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

						tryShiftedLaneFallback(newTrajMergingConflictFree, 2, false);
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
		else if (!currentTraj.empty()) {
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
						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 0, true);
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

						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 0, false);
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
						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 1, true);
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

						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 1, false);
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
						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 2, true);
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

						tryShiftedLaneFallback(newTrajLaneChangeConflictFree, 2, false);
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
		else if (!currentTraj.empty()) {
			trajectory_found = true;
			finalTrajectory = currentTraj;  // Found suitable trajectory
			possiblePriorityLevelForAccept = 0;
			//std::cout << mVehicleController->getVehicleId() << " there is no conflict with received Req Traj " << std::endl;
		}

	}
	/// @note  -----------------------------------------------------------------------here ends for received lane change request ---------------------------------------------------------------------
	if (trajectory_found == true){
		finalizeSuitableTrajectoryCvResult(
			*this,
			acceleration_trajectory_found,
			foundSpeedIncrease,
			foundSpeedReduction,
			foundAcceleration,
			foundDeceleration,
			lane_changed,
			foundTimeGap,
			possiblePriorityLevelForAccept,
			plannedTrajValues,
			foundTrajectoryCost,
			typeOfTrajectory);
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
	// calculateSecondReqTraj may keep original geometry if samples never reach indexToChangeLane + 2.
	auto candidateContainsShiftedLane = [&](const Trajectory& candidate, int indexToChangeLane) {
		const int startIndex = std::max(0, indexToChangeLane + 2);
		const int endIndex = static_cast<int>(std::min(cx.size(), cy.size()));

		if (candidate.empty() || startIndex >= endIndex) {
			return false;
		}

		constexpr double coordinateTolerance = 1e-4;

		for (const auto& point : candidate) {
			for (int i = startIndex; i < endIndex; ++i) {
				if (std::abs(point.mX - (cx[i] + 3.0F)) <= coordinateTolerance &&
						std::abs(point.mY - cy[i]) <= coordinateTolerance) {
					return true;
				}
			}
		}

		return false;
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
					lane_changed = candidateContainsShiftedLane(newTraj, indexToChangeLane);
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
							lane_changed = candidateContainsShiftedLane(newTraj, indexToChangeLane);
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
					lane_changed = candidateContainsShiftedLane(newTraj, indexToChangeLane);
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
					lane_changed = candidateContainsShiftedLane(newTraj, indexToChangeLane);
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
							lane_changed = candidateContainsShiftedLane(newTraj, indexToChangeLane);
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

		typeOfTrajectory = classifyTrajectoryType(plannedTrajValues, acceleration_trajectory_found);

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


} // namespace mcm
} // namespace artery
