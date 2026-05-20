#include "artery/application/mcm/TrajectoryEnvironment.h"

#include <cmath>
#include <limits>

namespace artery
{
namespace mcm
{

double getDistance(double x1, double y1, double x2, double y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}

double getMinGapPriority(int maneuverPriority)
{
    if (maneuverPriority == 0) {
        return 0.8;
    }
    else if (maneuverPriority == 1) {
        return 0.6;
    }
    else if (maneuverPriority == 2) {
        return 0.3;
    }

    return 1.0;
}

double calculateTimeGap(double distance, double speed)
{
    if (speed <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    return distance / speed;
}

double calculateDecelerationTime(
    double initialVelocity,
    double finalVelocity,
    double deceleration)
{
    if (deceleration <= 0.0) {
        return 0.0;
    }

    if (initialVelocity <= finalVelocity) {
        return 0.0;
    }

    return (initialVelocity - finalVelocity) / deceleration;
}

double calculateTTC(
    double mySpeed,
    double frontVehicleSpeed,
    double distance)
{
    const double relativeSpeed = mySpeed - frontVehicleSpeed;

    if (relativeSpeed <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    if (distance <= 0.0) {
        return 0.0;
    }

    return distance / relativeSpeed;
}

bool isWithinInterval(double value, double lowerBound, double upperBound)
{
    return value >= lowerBound && value <= upperBound;
}

bool checkIfSameEdgeAndLane(
    const std::string& egoEdge,
    int egoLane,
    const std::string& otherEdge,
    int otherLane)
{
    return egoEdge == otherEdge && egoLane == otherLane;
}

} // namespace mcm
} // namespace artery
