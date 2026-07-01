#ifndef TRAJECTORYCSV_H_
#define TRAJECTORYCSV_H_

#include <string>
#include <vector>

namespace artery
{
namespace mcm
{

std::vector<std::string> splitString(const std::string& line, char delimiter);
std::vector<float> readCX(const std::string& filename, char delimiter, const std::string& column_name);
bool checkColumnExists(const std::string& filename, const std::string& columnName, char delimiter);

struct RouteTrajectoryColumns {
	std::string xColumn;
	std::string yColumn;
	bool usingFallback = false;
};

RouteTrajectoryColumns selectRouteTrajectoryColumns(const std::string& filename, char delimiter, const std::string& routeId);

} // namespace mcm
} // namespace artery

#endif /* TRAJECTORYCSV_H_ */
