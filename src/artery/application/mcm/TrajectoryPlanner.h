#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/mcm/TrajPointMCM.h"

#include <artery/envmod/EnvironmentModelObject.h>
#include <artery/envmod/LocalEnvironmentModel.h>

#include <omnetpp/simtime.h>

#include <string>
#include <tuple>
#include <vector>


namespace traci { class VehicleController; }

namespace artery
{

class VehicleDataProvider;

struct State {
	float x;
	float y;
	float yaw;
	float v;
};

struct FrontVehicleInfo {
	int frontVehID;
	std::string frontVehicleId;
	bool sameEdgeLane;
	double distance;
	double speed;
};

struct PlannedTrajValues {
	double speed_change;
	double acc_change;
	double deceleration_change;
	bool lane_change;
	double time_gap_change;
	double TTC_change;
};

namespace mcm
{

class TrajectoryPlanner
{
public:
	TrajectoryPlanner();
	TrajectoryPlanner(const traci::VehicleController* mvc, const VehicleDataProvider* mvdp, const LocalEnvironmentModel* lem);

	void initialize(const traci::VehicleController*, const VehicleDataProvider*, const LocalEnvironmentModel*);

	const traci::VehicleController* mVehicleController;
	const VehicleDataProvider* mVehicleDataProvider;
	const LocalEnvironmentModel* mLocalEnvironmentModel;

	using Trajectory = std::vector<TrajPointMCM>;
	using Vec_f = std::vector<float>;

	using TupleSuitableTrajectory = std::tuple<bool, Trajectory, PlannedTrajValues, double, int, int>;
	using TupleSecondRequestTrajRv = std::tuple<bool, Trajectory, PlannedTrajValues, double, int>;

	TrajPointMCM getPoint(TrajPointMCM calculatedPoint) const;

	Trajectory calculateRefTrajectory(int steps, double dt, const Vec_f& cx, const Vec_f& cy, int target_ind, bool constantVelocity, float targetSpeed, float acceleration, float deceleration);

	int calc_nearest_index(State state, const Vec_f& cx, const Vec_f& cy, int pind);

	bool check_traj_conflict(Trajectory traj_ego, Trajectory traj_other, float time_gap, omnetpp::SimTime receivedEteDelay);
	bool check_traj_conflict_merging(Trajectory traj_ego, Trajectory traj_other, float time_gap, omnetpp::SimTime receivedEteDelay, bool forRV);
	bool check_traj_conflict_lane_change(Trajectory traj_ego, Trajectory traj_other, float time_gap, omnetpp::SimTime receivedEteDelay);
	bool check_traj_collision(Trajectory traj_ego, Trajectory traj_other, float min_TTC, omnetpp::SimTime receivedEteDelay);

	void printTrajectory(const Trajectory& trajectory);

	double getDesiredGap();
	double getGap(double gap);

	FrontVehicleInfo getFrontVehicleInfo();

	TupleSuitableTrajectory findSuitableTrajectoryCV(Trajectory received_ReqTraj, int receivedPriority, bool mergingReqTraj, int steps, double dt, const Vec_f& cx, const Vec_f& cy, int mIndex, double current_speed, omnetpp::SimTime receivedEteDelay, bool decelerationRequired, bool accelerationRequired, bool isLaneChangePossible, bool isRouteAffected);

	double calculateTrajectoryCost(PlannedTrajValues plannedValues, bool isRouteAffected);

    Trajectory calculateExecuteTrajectory(int steps, double dt, const Vec_f& cx, const Vec_f& cy, int mIndex, bool constantVelocity, float targetSpeedNeg, float accNeg, float decelerationNeg, Trajectory negotiatedTrajectory, float targetSpeedAfterNeg, float targetAccAfterNeg, float targetDecelerationAfterNeg);

	int calc_nearest_index_NegTraj(Trajectory negotiatedTrajectory, const Vec_f& cx, const Vec_f& cy, int pind);

	Trajectory estimateOtherDecelerationTrajectory(int steps, double dt, const Vec_f& cx, const Vec_f& cy, int target_ind, bool constantVelocity, float targetSpeed, float acceleration, float deceleration, Trajectory traj_other);
	
	int findIndexTrajsConflictFree(Trajectory traj_ego, Trajectory traj_other, float time_gap, const Vec_f& cx, const Vec_f& cy, int pind);

    TupleSecondRequestTrajRv findSecondRequestTrajRV(Trajectory estimatedOtherTraj, int requestPriority, int steps, double dt, const Vec_f& cx, const Vec_f& cy, int mIndex, double current_speed, omnetpp::SimTime receivedEteDelay, bool decelerationRequired, bool accelerationRequired);
	
    Trajectory calculateSecondReqTraj(int steps, double dt, const Vec_f& cx, const Vec_f& cy, int target_ind, bool constantVelocity, float targetSpeed, float acceleration, float deceleration, int indexToChangeLane);
	
	TupleSuitableTrajectory newPlannedTrajRV(Trajectory receivedConflictedTraj, int requestPriority, int steps, double dt, const Vec_f& cx, const Vec_f& cy, int mIndex, double current_speed, omnetpp::SimTime receivedEteDelay, bool decelerationRequired, bool accelerationRequired);

};

} // namespace mcm
} // namespace artery

#endif /* TRAJECTORYPLANNER_H_ */
