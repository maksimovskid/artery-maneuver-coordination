#ifndef ARTERY_MCM_MCAPPLICATION_H_
#define ARTERY_MCM_MCAPPLICATION_H_

#include <omnetpp/simtime.h>

#include <cstddef>
#include <cstdint>

namespace traci
{
class VehicleController;
}

namespace artery
{
namespace mcm
{

enum class McmOperationMode {
    Unknown,
    IntentSharing,
    ManeuverNegotiation,
    ManeuverExecution
};

struct ReceivedMcm {
    uint32_t stationId = 0;
    uint16_t generationDeltaTime = 0;
    long longitude = 0;
    long latitude = 0;
    long speedValue = 0;
    long headingValue = 0;
    std::size_t plannedTrajectoryPointCount = 0;
    McmOperationMode operationMode = McmOperationMode::Unknown;
    bool hasNegotiationContainer = false;
    bool hasExecutionContainer = false;
    long mcmCategory = -1;
    long priorityManeuver = -1;
    long cooperationTypeMcm = -1;
    long requestId = -1;
    long numberOfVehicles = -1;
    uint32_t negotiationVehicleId1 = 0;
    bool hasNegotiationVehicleId2 = false;
    uint32_t negotiationVehicleId2 = 0;
    std::size_t requestedTrajectoryPointCount = 0;
    std::size_t offeredTrajectoryPointCount = 0;
    bool hasAlternativeTrajectory = false;
    std::size_t alternativeTrajectoryPointCount = 0;
    omnetpp::SimTime receivedAt;
};

struct SentMcm {
    uint32_t stationId = 0;
    uint16_t generationDeltaTime = 0;
    long longitude = 0;
    long latitude = 0;
    long speedValue = 0;
    long headingValue = 0;
    std::size_t plannedTrajectoryPointCount = 0;
    McmOperationMode operationMode = McmOperationMode::Unknown;
    bool hasNegotiationContainer = false;
    bool hasExecutionContainer = false;
    long mcmCategory = -1;
    long priorityManeuver = -1;
    long cooperationTypeMcm = -1;
    long requestId = -1;
    long numberOfVehicles = -1;
    uint32_t negotiationVehicleId1 = 0;
    bool hasNegotiationVehicleId2 = false;
    uint32_t negotiationVehicleId2 = 0;
    std::size_t requestedTrajectoryPointCount = 0;
    std::size_t offeredTrajectoryPointCount = 0;
    bool hasAlternativeTrajectory = false;
    std::size_t alternativeTrajectoryPointCount = 0;
    omnetpp::SimTime sentAt;
};

class McApplication
{
public:
    McApplication() = default;

    void initialize(traci::VehicleController*);
    void tick(omnetpp::SimTime now);
    void handleReceivedMcm(const ReceivedMcm&);
    void handleSentMcm(const SentMcm&);
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

    unsigned mReceivedMcmCount = 0;
    bool mHasLastReceivedMcm = false;
    ReceivedMcm mLastReceivedMcm;
    unsigned mSentMcmCount = 0;
    bool mHasLastSentMcm = false;
    SentMcm mLastSentMcm;

    double mTargetSpeed = 0.0;
    double mCommandDuration = 0.0;
    double mRestoreSpeed = 0.0;
    double mRestoreMaxSpeed = 0.0;
};

}  // namespace mcm

}  // namespace artery

#endif /* ARTERY_MCM_MCAPPLICATION_H_ */
