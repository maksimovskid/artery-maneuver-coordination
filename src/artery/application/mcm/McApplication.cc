#include "artery/application/mcm/McApplication.h"

namespace artery
{
namespace mcm
{

void McApplication::initialize(traci::VehicleController* controller)
{
    mVehicleController = controller;
}

void McApplication::tick(omnetpp::SimTime)
{
    // TODO: later apply execution commands from parsed MCM Execute/Cancel events.
}

void McApplication::handleReceivedMcm(const ReceivedMcm& mcm)
{
    ++mReceivedMcmCount;
    mLastReceivedMcm = mcm;
    mHasLastReceivedMcm = true;
}

void McApplication::handleSentMcm(const SentMcm& mcm)
{
    ++mSentMcmCount;
    mLastSentMcm = mcm;
    mHasLastSentMcm = true;
}

void McApplication::clearCommand()
{
    mPendingCommand = CommandKind::None;
    mExecutionState = ExecutionState::Idle;
    mTargetSpeed = 0.0;
    mCommandDuration = 0.0;
    mRestoreSpeed = 0.0;
    mRestoreMaxSpeed = 0.0;
}

bool McApplication::hasActiveExecution() const
{
    return mExecutionState == ExecutionState::Pending || mExecutionState == ExecutionState::Executing;
}

void McApplication::applyCommand()
{
    // TODO: later use VehicleController hooks for DecelerateTo and RestoreNormalSpeed commands.
}

}  // namespace mcm
}  // namespace artery
