#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

#include <boost/units/systems/si/length.hpp>

#include <iostream>
#include <string>

namespace artery
{
namespace mcm
{

void TrajectoryPlanner::printTrajectory(const Trajectory& trajectory) {
    std::cout << "Printing trajectory for vehicle:" << mVehicleController->getVehicleId() << std::endl;
    for (const auto& point : trajectory) {
        std::cout << "mX: " << point.mX << ", "
                  << "mY: "  << point.mY << ", "
                  << "mYaw: " << point.mHeading << ", "
                  << "mT: " << point.mTime << std::endl;
    }
}

double TrajectoryPlanner::getDesiredGap()
{
    const double timeGap = 1.0;
    const double ego_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

    return timeGap * ego_speed;
}

double TrajectoryPlanner::getGap(double gap)
{
    const double ego_speed =
        mVehicleDataProvider->speed() / boost::units::si::meter_per_second;

    return gap * ego_speed;
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

} // namespace mcm
} // namespace artery
