#ifndef VEHICLECONTROLLER_H_AXBS5NQM
#define VEHICLECONTROLLER_H_AXBS5NQM

#include "artery/traci/Controller.h"
#include "artery/traci/VehicleType.h"

#include <vector>

namespace traci
{

class VehicleCache;

class VehicleController : public Controller
{
public:
    VehicleController(std::shared_ptr<traci::API>, const std::string& id);
    VehicleController(std::shared_ptr<traci::API>, std::shared_ptr<VehicleCache> cache);

    const std::string& getVehicleId() const;
    const VehicleType& getVehicleType() const;
    const std::string getVehicleClass() const;

    using Controller::getLength;

    Velocity getMaxSpeed() const;
    void setMaxSpeed(Velocity);
    void setSpeed(Velocity);
    void setSpeedFactor(double);

    void changeTarget(const std::string& edge);

    void changeLane(const std::string& vehicleID, int laneIndex, double duration) const;
    void changeSublane(const std::string& vehicleID, double latDist) const;
    void moveTo(const std::string& vehicleID, const std::string& laneID, double position, int reason = libsumo::MOVE_TELEPORT) const;
    void moveToXY(const std::string& vehicleID, const std::string& edgeID, int lane, double x, double y, double angle, int keepRoute) const;
    void slowDown(const std::string& vehicleID, double speed, double duration) const;
    void setLaneChangeMode(const std::string& vehicleID, int mode) const;
    void setSpeedMode(const std::string& vehicleID, int mode) const;

    artery::Position getPositionSumo() const;

    std::string getRouteID() const;
    int getRouteIndex() const;
    std::vector<std::string> getRoute() const;
    std::vector<std::string> getEdges(const std::string& routeID) const;
    double getLength(const std::string& laneID) const;

    void setAccel(const std::string& typeID, double accel) const;
    void setDecel(const std::string& typeID, double decel) const;
    void setTau(const std::string& typeID, double tau) const;
    void setType(const std::string& vehicleID, const std::string& typeID) const;

private:
    VehicleType m_type;
};

} // namespace traci

#endif /* VEHICLECONTROLLER_H_AXBS5NQM */
