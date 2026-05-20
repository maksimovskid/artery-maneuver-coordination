#include "artery/application/mcm/TrajectoryConflict.h"
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

} // namespace mcm
} // namespace artery
