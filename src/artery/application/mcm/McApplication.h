#ifndef ARTERY_MCM_MCAPPLICATION_H_
#define ARTERY_MCM_MCAPPLICATION_H_

#include <omnetpp/simtime.h>

namespace traci
{
class VehicleController;
}

namespace artery
{
namespace mcm
{

class McApplication
{
public:
    McApplication() = default;

    void initialize(traci::VehicleController*);
    void tick(omnetpp::SimTime now);
    void clearCommand();
    bool hasActiveExecution() const;

    // TODO: add negotiation strategy and maneuver state handling.
    // TODO: add scenario-specific maneuver coordination behavior.
    // TODO: integrate TrajectoryPlanner helpers after the service-level MCM plumbing is stable.

private:
    enum class Role {
        None,
        RequestingVehicle,
        CooperatingVehicle
    };

    enum class ExecutionState {
        Idle,
        Pending,
        Executing,
        Complete,
        Cancelled
    };

    enum class CommandKind {
        None,
        DecelerateTo,
        RestoreNormalSpeed
    };

    void applyCommand();

    traci::VehicleController* mVehicleController = nullptr;
    Role mRole = Role::None;
    ExecutionState mExecutionState = ExecutionState::Idle;
    CommandKind mPendingCommand = CommandKind::None;

    double mTargetSpeed = 0.0;
    double mCommandDuration = 0.0;
    double mRestoreSpeed = 0.0;
    double mRestoreMaxSpeed = 0.0;
};

}  // namespace mcm

}  // namespace artery

#endif /* ARTERY_MCM_MCAPPLICATION_H_ */
