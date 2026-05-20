#include "artery/application/mcm/TrajectoryGeneration.h"
#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

#include <boost/units/systems/si/length.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace artery
{

using namespace omnetpp;

namespace mcm
{

std::vector<float> makeShiftedLaneCx(const std::vector<float>& cx)
{
	std::vector<float> cx_new(cx.size());
	// In the evaluated scenario, lanes are 3 m wide and the road geometry is aligned such that a +3 m x-offset represents the adjacent lane.
	std::transform(cx.begin(), cx.end(), cx_new.begin(), [](float x) { return x + 3.0F; });
	return cx_new;
}

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

	Vec_f cx_new = makeShiftedLaneCx(cx);

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

	if (steps <= 0 || dt <= 0.0 || cx.empty() || cy.empty()) {
		return trajectory;
	}

	TrajPointMCM calculatedPoint;
	State state;

	state.x = mVehicleController->getPositionSumo().x / boost::units::si::meter;
	state.y = mVehicleController->getPositionSumo().y / boost::units::si::meter;
	state.yaw = mVehicleDataProvider->heading() / boost::units::si::radians;
	state.v = mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

	Vec_f cx_new = makeShiftedLaneCx(cx);

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

		// Use shifted lane geometry once route samples reach indexToChangeLane + 2.
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

} // namespace mcm
} // namespace artery
