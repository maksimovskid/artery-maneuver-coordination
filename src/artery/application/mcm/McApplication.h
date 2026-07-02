#ifndef ARTERY_MCM_MCAPPLICATION_H_
#define ARTERY_MCM_MCAPPLICATION_H_

#include "artery/application/mcm/TrajPointMCM.h"
#include "artery/application/mcm/TrajectoryPlanner.h"

#include <omnetpp/simtime.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace traci
{
class VehicleController;
}

namespace artery
{
class VehicleDataProvider;

namespace mcm
{

enum class operationMode {
    Unknown,
    IntentionSharingMode,
    ManeuverNegotiationMode,
    ManeuverExecutionMode
};

enum class mcmSubtype {
    Request,
    Accept,
    Reject,
    Offer,
    Confirm,
    Execute,
    Cancel,
    Abort,
    CascadingRequest,
    CascadingAccept,
    CascadingReject,
    CascadingExecute,
    Regular
};

enum class priorityMcmCategory {
    LowPriority,
    MediumPriority,
    HighPriority,
    EmergencyPriority,
    NoPriority
};

enum class coordinationManeuver {
    Merge,
    LaneChange,
    Overtake,
    NoCoordManeuver
};

enum class coordinationProgressRV {
    NoCoordination,
    CheckForCoordination,
    CoordinationRequired,
    RequestSent,
    SendConfirm,
    ConfirmSent,
    SendExecute,
    SendComplete,
    CompleteSent,
    NoCoordinationRequired,
    CancelCoordination,
    SecondRequest
};

enum class coordinationProgressCV {
    NoRequest,
    ReceivedRequest,
    SendOffer,
    OfferSent,
    SendAccept,
    AcceptSent,
    SendExecuteCV,
    SendCompleteCV,
    CompleteSentCV,
    CancelNegotiation,
    SendReject
};

enum class cooperatingVehicleType {
    NCV,
    RV,
    CV,
    EmergencyV
};

enum class controlManeuver {
    DoNothing,
    Decelerate,
    Accelerate,
    ChangeLane,
    LaneChangeExecution,
    EmergencyDeceleration
};

struct McmSnapshot {
    uint32_t stationId = 0;
    uint16_t generationDeltaTime = 0;
    long longitude = 0;
    long latitude = 0;
    long speedValue = 0;
    long headingValue = 0;
    bool hasLaneId = false;
    long laneId = -1;
    std::size_t plannedTrajectoryPointCount = 0;
    std::vector<mcm::TrajPointMCM> plannedTrajectory;
    mcm::operationMode operationMode = mcm::operationMode::Unknown;
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
    std::vector<mcm::TrajPointMCM> requestedTrajectory;
    std::size_t offeredTrajectoryPointCount = 0;
    bool hasAlternativeTrajectory = false;
    std::size_t alternativeTrajectoryPointCount = 0;
    long cooperationId = -1;
    uint32_t cooperationVehicleId1 = 0;
    bool hasCooperationVehicleId2 = false;
    uint32_t cooperationVehicleId2 = 0;
};

struct ReceivedMcm {
    McmSnapshot data;
    omnetpp::SimTime receivedAt;
};

struct SentMcm {
    McmSnapshot data;
    omnetpp::SimTime sentAt;
};

struct McEgoContext {
    omnetpp::SimTime now;
    uint32_t stationId = 0;
    std::string routeId;
    int laneIndex = -1;
    double x = 0.0;
    double y = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    TrajectoryPlanner::Trajectory plannedTrajectory;
    TrajectoryPlanner::Vec_f routeReferenceX;
    TrajectoryPlanner::Vec_f routeReferenceY;
    int routeReferenceIndex = -1;
};

struct PendingMcmCommand {
    enum class Kind {
        Negotiation,
        Execution
    };

    Kind kind = Kind::Negotiation;
    mcmSubtype subtype = mcmSubtype::Request;
    priorityMcmCategory priority = priorityMcmCategory::MediumPriority;
    long cooperationType = 0;
    uint8_t requestId = 0;
    uint8_t numberOfVehicles = 1;
    uint32_t targetVehicle1 = 0;
    bool hasTargetVehicle2 = false;
    uint32_t targetVehicle2 = 0;

    TrajectoryPlanner::Trajectory requestedTrajectory;

    // Used by Offer messages.
    // For Request/Accept/Reject this can stay empty and McService can use a valid placeholder.
    TrajectoryPlanner::Trajectory offeredTrajectory;
    bool hasOfferedTrajectory = false;
};

class McApplication
{
public:
    McApplication() = default;

    void initialize(traci::VehicleController*, const VehicleDataProvider*);
    void setNegotiationRetryInterval(omnetpp::SimTime interval);
    void updateEgoContext(const McEgoContext&);
    void tick(omnetpp::SimTime now);
    void prepareMcmGeneration(omnetpp::SimTime now);
    void handleReceivedMcm(const ReceivedMcm&);
    void handleSentMcm(const SentMcm&);
    std::optional<PendingMcmCommand> consumePendingCommand();
    void clearCommand();
    bool hasActiveExecution() const;

    // TODO: add negotiation strategy and maneuver state handling.
    // TODO: add scenario-specific maneuver coordination behavior.
    // TODO: integrate TrajectoryPlanner helpers after the service-level MCM plumbing is stable.

private:
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
    void evaluateMergingRequestTrigger(omnetpp::SimTime now);
    void evaluateRvRequestRetry(omnetpp::SimTime now);
    void evaluateEmergencyBrakingTrigger(omnetpp::SimTime now);
    void evaluateSafetyCriticalLaneChangeTrigger(omnetpp::SimTime now);
    void logScenarioVehicleLifetime(omnetpp::SimTime now);
    void applyRvExecutionControl();
    void applyCvDecelerationControl();
    void applyCvAccelerationControl();
    void applyCvLaneChangeControl();
    void restoreCvSpeedControl();
    void classifyCvMergingControlManeuver(const ReceivedMcm&);
    void evaluateRvExecutionProgress();
    void evaluateCvExecutionProgress();
    void evaluateCvRequestResponse(const ReceivedMcm&);
    void handleReceivedCancelAsCv(const ReceivedMcm&);
    void handleReceivedOfferAsRv(const ReceivedMcm&);
    void handleReceivedConfirmAsCv(const ReceivedMcm&);
    void handleReceivedAcceptAsRv(const ReceivedMcm&);
    void handleReceivedExecuteAsCv(const ReceivedMcm&);
    void handleReceivedEmergencyAsFollower(const ReceivedMcm&);
    void logNegotiationTrace(const char* action, const McmSnapshot& snapshot, omnetpp::SimTime time) const;
    bool hasReachedActiveNegotiatedTrajectoryEnd() const;
    void queueRepeatedExecute();
    void resetMergingGapDiagnostics();
    void sampleMergingGapDiagnostics(const char* phase);
    void logMergingGapSummary(omnetpp::SimTime completionTime) const;
    uint8_t makeRequestId(omnetpp::SimTime now) const;

    traci::VehicleController* mVehicleController = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    TrajectoryPlanner mTrajectoryPlanner;
    operationMode mOperationMode = operationMode::IntentionSharingMode;
    mcmSubtype mMcmSubtype = mcmSubtype::Regular;
    priorityMcmCategory mPriorityMcmCategory = priorityMcmCategory::NoPriority;
    coordinationManeuver mCoordinationManeuver = coordinationManeuver::NoCoordManeuver;
    coordinationProgressRV mCoordinationProgressRV = coordinationProgressRV::NoCoordination;
    coordinationProgressCV mCoordinationProgressCV = coordinationProgressCV::NoRequest;
    cooperatingVehicleType mCooperatingVehicleType = cooperatingVehicleType::NCV;
    controlManeuver mControlManeuver = controlManeuver::DoNothing;
    ExecutionState mExecutionState = ExecutionState::Idle;
    CommandKind mPendingCommand = CommandKind::None;

    unsigned mReceivedMcmCount = 0;
    bool mHasLastReceivedMcm = false;
    ReceivedMcm mLastReceivedMcm;
    std::vector<ReceivedMcm> mReceivedMcmCache;
    unsigned mSentMcmCount = 0;
    bool mHasLastSentMcm = false;
    SentMcm mLastSentMcm;
    bool mHasEgoContext = false;
    McEgoContext mEgoContext;
    std::optional<PendingMcmCommand> mPendingMcmCommand;
    bool mMergingRequestQueuedOrSent = false;
    omnetpp::SimTime mNegotiationRetryInterval = 0.1;

    // CV-side state for responding to a received Request.
    bool mCvResponseQueuedOrSent = false;
    uint32_t mCvRvStationId = 0;
    uint8_t mCvRequestId = 0;

    // RV-side state for the active merging Request.
    uint8_t mRvRequestId = 0;
    uint8_t mRvNumberOfVehicles = 1;
    uint32_t mRvTargetVehicle1 = 0;
    uint32_t mRvTargetVehicle2 = 0;
    bool mRvOfferReceived1 = false;
    bool mRvOfferReceived2 = false;
    bool mRvConfirmQueuedOrSent = false;
    omnetpp::SimTime mRvLastRequestQueuedAt = omnetpp::SimTime::ZERO;
    bool mHasRvLastRequestQueuedAt = false;

    // RV-side state for final Accept responses after Confirm.
    bool mRvAcceptReceived1 = false;
    bool mRvAcceptReceived2 = false;
    bool mRvExecuteQueuedOrSent = false;

    // Fixed trajectory agreed during negotiation.
    // During execution this is used only as the completion reference;
    // the live plannedTrajectory/intent may continue updating beyond this horizon.
    TrajectoryPlanner::Trajectory mActiveNegotiatedTrajectory;
    bool mHasActiveNegotiatedTrajectory = false;
    TrajectoryPlanner::Trajectory mCvSelectedTrajectory;
    bool mHasCvSelectedTrajectory = false;

    // RV-side repeated Execute signaling during maneuver execution.
    omnetpp::SimTime mLastExecuteQueuedAt = omnetpp::SimTime::ZERO;
    bool mHasLastExecuteQueuedAt = false;
    bool mRvMergingExecutionControlLogged = false;
    bool mCvDecelerationControlApplied = false;
    bool mCvDecelerationControlSkippedLogged = false;
    bool mCvAccelerationControlApplied = false;
    bool mCvLaneChangeControlLogged = false;

    bool mEmergencyBrakeApplied = false;
    bool mEmergencyMcmQueued = false;
    bool mScenarioVehicleFirstSeenLogged = false;
    bool mScenarioVehicleNearEmergencyLogged = false;
    bool mScenarioVehicleAfterEmergencyLogged = false;
    bool mEmergencyTriggerWaitingLogged = false;
    bool mEmergencyReceived = false;
    bool mLaneChangeRequestQueuedOrSent = false;
    bool mLaneChangeStateLogged = false;
    bool mLaneChangeThreeVehiclePath = false;
    uint32_t mEmergencyStationId = 0;
    double mEmergencyDistanceGap = 0.0;
    double mEmergencyTimeGap = 0.0;
    double mEmergencyReactionWindow = 0.0;

    // Diagnostic-only route_merging_1 RV/CV gap tracking.
    bool mMergingGapDiagActive = false;
    omnetpp::SimTime mMergingGapDiagExecutionStart = omnetpp::SimTime::ZERO;
    uint32_t mMergingGapDiagTargetCvStationId = 0;
    double mMergingGapDiagMinDistance = 0.0;
    double mMergingGapDiagMinTimeGap = 0.0;
    omnetpp::SimTime mMergingGapDiagMinDistanceAt = omnetpp::SimTime::ZERO;
    omnetpp::SimTime mMergingGapDiagMinTimeGapAt = omnetpp::SimTime::ZERO;
    bool mMergingGapDiagHasMinDistance = false;
    bool mMergingGapDiagHasMinTimeGap = false;
    std::string mMergingGapDiagMinRvLaneId;
    std::string mMergingGapDiagMinCvLaneId;
    int mMergingGapDiagMinRvLaneIndex = -1;
    int mMergingGapDiagMinCvLaneIndex = -1;
    double mMergingGapDiagMinRvLanePosition = 0.0;
    double mMergingGapDiagMinCvX = 0.0;
    double mMergingGapDiagMinCvY = 0.0;
    double mMergingGapDiagMinRvX = 0.0;
    double mMergingGapDiagMinRvY = 0.0;

    double mTargetSpeed = 0.0;
    double mCommandDuration = 0.0;
    double mRestoreSpeed = 0.0;
    double mRestoreMaxSpeed = 0.0;
};

}  // namespace mcm

}  // namespace artery

#endif /* ARTERY_MCM_MCAPPLICATION_H_ */
