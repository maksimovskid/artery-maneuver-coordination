#ifndef TRAJECTORYENVIRONMENT_H_
#define TRAJECTORYENVIRONMENT_H_

#include <string>

namespace artery
{
namespace mcm
{

double getDistance(double x1, double y1, double x2, double y2);
double getMinGapPriority(int maneuverPriority);
double calculateTimeGap(double distance, double speed);
double calculateDecelerationTime(double initialVelocity, double finalVelocity, double deceleration);
double calculateTTC(double mySpeed, double frontVehicleSpeed, double distance);
bool isWithinInterval(double value, double lowerBound, double upperBound);
bool checkIfSameEdgeAndLane(const std::string& egoEdge, int egoLane, const std::string& otherEdge, int otherLane);

} // namespace mcm
} // namespace artery

#endif /* TRAJECTORYENVIRONMENT_H_ */
