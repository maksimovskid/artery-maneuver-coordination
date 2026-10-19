#include "artery/application/mcm/TrajectoryConflict.h"
#include "artery/application/mcm/McmTrajectory.h"
#include "artery/application/VehicleDataProvider.h"

#include <boost/units/systems/si/length.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace artery
{
namespace mcm
{

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

    for (unsigned int i = 0; i < indices.size(); ++i) {

        const int idx = indices[i];

        const auto corrected = compensateTrajectoryPoint(traj_other, idx,
            receivedEteDelay, CompensationDirection::NegativeY);
        const float other_y = corrected.mY;

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

    for (unsigned int i = 0; i < indices.size(); ++i) {

        const int idx = indices[i];

        const auto corrected = compensateTrajectoryPoint(traj_other, idx,
            receivedEteDelay, CompensationDirection::NegativeY);
        const float other_y = corrected.mY;

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
    const float current_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;
    const float distance_gap = time_gap * current_speed;
    const auto corrected = compensateTrajectoryPoint(traj_other, t1, receivedEteDelay,
        forRV ? CompensationDirection::NegativeY : CompensationDirection::PositiveX);
    return std::hypot(traj_ego[t1].mX - corrected.mX,
        traj_ego[t1].mY - corrected.mY) < distance_gap;
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

    for (unsigned int i = 0; i < indices.size(); ++i) {
        const int idx = indices[i];

        const float ego_speed = trajectorySegmentSpeed(traj_ego, idx);
        const float other_speed = trajectorySegmentSpeed(traj_other, idx);

        const float relative_speed = ego_speed - other_speed;

        if (relative_speed <= 0.0F) {
            continue;
        }

        const auto corrected = compensateTrajectoryPoint(traj_other, idx,
            receivedEteDelay, CompensationDirection::NegativeY);
        const float other_y = corrected.mY;

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

int TrajectoryPlanner::findIndexTrajsConflictFree(
	Trajectory traj_ego,
	Trajectory traj_other,
	float time_gap,
	const Vec_f& cx,
	const Vec_f& cy,
	int pind,
	omnetpp::SimTime informationAge)
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
        const auto corrected = compensateTrajectoryPoint(traj_other, t,
            informationAge, CompensationDirection::NegativeY);
		const float dx = traj_ego[t].mX - corrected.mX;
		const float dy = traj_ego[t].mY - corrected.mY;
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

} // namespace mcm
} // namespace artery
