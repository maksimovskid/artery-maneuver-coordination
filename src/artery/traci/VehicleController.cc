#include "artery/traci/VehicleController.h"
#include "artery/traci/Cast.h"

namespace si = boost::units::si;

namespace traci
{

VehicleController::VehicleController(std::shared_ptr<traci::API> api, const std::string& id) :
    VehicleController(api, std::make_shared<VehicleCache>(api, id))
{
}

VehicleController::VehicleController(std::shared_ptr<traci::API> api, std::shared_ptr<VehicleCache> cache) :
    Controller(api, cache),
    m_type(api->vehicletype, api->vehicle.getTypeID(cache->getId()))
{
}

const std::string& VehicleController::getVehicleId() const
{
    return getId();
}

const VehicleType& VehicleController::getVehicleType() const
{
    return m_type;
}

const std::string VehicleController::getVehicleClass() const
{
    return m_cache->get<libsumo::VAR_VEHICLECLASS>();
}

auto VehicleController::getMaxSpeed() const -> Velocity
{
    return m_cache->get<libsumo::VAR_MAXSPEED>() * si::meter_per_second;
}

void VehicleController::setMaxSpeed(Velocity v)
{
    m_traci->vehicle.setMaxSpeed(getId(), v / si::meter_per_second);
}

void VehicleController::setSpeed(Velocity v)
{
    m_traci->vehicle.setSpeed(getId(), v / si::meter_per_second);
}

void VehicleController::setSpeedFactor(double f)
{
    m_traci->vehicle.setSpeedFactor(getId(), f);
}

void VehicleController::changeTarget(const std::string& edge)
{
    m_traci->vehicle.changeTarget(getId(), edge);
}

void VehicleController::changeLane(const std::string&, int laneIndex, double duration) const
{
    m_traci->vehicle.changeLane(getId(), laneIndex, duration);
}

void VehicleController::changeSublane(const std::string&, double latDist) const
{
    m_traci->vehicle.changeSublane(getId(), latDist);
}

void VehicleController::moveTo(const std::string&, const std::string& laneID, double position, int reason) const
{
    m_traci->vehicle.moveTo(getId(), laneID, position, reason);
}

void VehicleController::moveToXY(const std::string&, const std::string& edgeID, int lane, double x, double y, double angle, int keepRoute) const
{
    m_traci->vehicle.moveToXY(getId(), edgeID, lane, x, y, angle, keepRoute);
}

void VehicleController::slowDown(const std::string&, double speed, double duration) const
{
    m_traci->vehicle.slowDown(getId(), speed, duration);
}

void VehicleController::setLaneChangeMode(const std::string&, int mode) const
{
    m_traci->vehicle.setLaneChangeMode(getId(), mode);
}

void VehicleController::setSpeedMode(const std::string&, int mode) const
{
    m_traci->vehicle.setSpeedMode(getId(), mode);
}

artery::Position VehicleController::getPositionSumo() const
{
    return traci::position_cast_sumo(m_cache->get<libsumo::VAR_POSITION>());
}

std::string VehicleController::getRouteID() const
{
    return m_traci->vehicle.getRouteID(getId());
}

int VehicleController::getRouteIndex() const
{
    return m_traci->vehicle.getRouteIndex(getId());
}

std::vector<std::string> VehicleController::getRoute() const
{
    return m_traci->vehicle.getRoute(getId());
}

std::vector<std::string> VehicleController::getEdges(const std::string& routeID) const
{
    return m_traci->route.getEdges(routeID);
}

double VehicleController::getLength(const std::string& laneID) const
{
    return m_traci->lane.getLength(laneID);
}

void VehicleController::setAccel(const std::string&, double accel) const
{
    // Old MCM call sites sometimes pass a vehicle ID here; mutate this vehicle's current type for compatibility.
    m_traci->vehicletype.setAccel(getTypeId(), accel);
}

void VehicleController::setDecel(const std::string&, double decel) const
{
    // Old MCM call sites sometimes pass a vehicle ID here; mutate this vehicle's current type for compatibility.
    m_traci->vehicletype.setDecel(getTypeId(), decel);
}

void VehicleController::setTau(const std::string&, double tau) const
{
    // Old MCM call sites sometimes pass a vehicle ID here; mutate this vehicle's current type for compatibility.
    m_traci->vehicletype.setTau(getTypeId(), tau);
}

void VehicleController::setType(const std::string&, const std::string& typeID) const
{
    m_traci->vehicle.setType(getId(), typeID);
}

} // namespace traci
