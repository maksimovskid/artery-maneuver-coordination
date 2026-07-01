#include "artery/application/mcm/TrajectoryCsv.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace artery
{
namespace mcm
{

std::vector<std::string> splitString(const std::string& line, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(line);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<float> readCX(const std::string& filename, char delimiter, const std::string& column_name)
{
    std::vector<float> cx;
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        if (std::getline(file, line)) { // Read the header line
            std::vector<std::string> headers = splitString(line, delimiter);
            auto it = std::find(headers.begin(), headers.end(), column_name);
            if (it != headers.end()) { // Column name found
                size_t columnIndex = std::distance(headers.begin(), it);
                while (std::getline(file, line)) {
                    std::vector<std::string> row = splitString(line, delimiter);
                    if (row.size() > columnIndex) {
                        float x = std::stof(row[columnIndex]);
                        cx.push_back(x);
                    }
                }
            } else {
                std::cout << "Column name '" << column_name << "' not found in the CSV file." << std::endl;
            }
        }
        file.close();
    } else {
        std::cout << "Failed to open the CSV file: " << filename << std::endl;
    }
    return cx;
}

bool checkColumnExists(const std::string& filename, const std::string& columnName, char delimiter)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        // File couldn't be opened
        return false;
    }

    std::string line;
    if (std::getline(file, line)) {
        // Read the header line (assuming the first line contains headers)
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            if (token == columnName) {
                // Column found
                file.close();
                return true;
            }
        }
    }

    // Column not found
    file.close();
    return false;
}

RouteTrajectoryColumns selectRouteTrajectoryColumns(const std::string& filename, char delimiter, const std::string& routeId)
{
    RouteTrajectoryColumns columns;
    columns.xColumn = routeId + "_x";
    columns.yColumn = routeId + "_y";

    if (checkColumnExists(filename, columns.xColumn, delimiter) &&
            checkColumnExists(filename, columns.yColumn, delimiter)) {
        return columns;
    }

    columns.xColumn = "others_x";
    columns.yColumn = "others_y";
    columns.usingFallback = true;
    return columns;
}

} // namespace mcm
} // namespace artery
