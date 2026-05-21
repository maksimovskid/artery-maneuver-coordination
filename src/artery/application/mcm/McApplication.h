#ifndef ARTERY_MCM_MCAPPLICATION_H_
#define ARTERY_MCM_MCAPPLICATION_H_

namespace artery
{
namespace mcm
{

class McApplication
{
public:
    McApplication() = default;

    // TODO: add minimal MCM message creation and send/receive state handling.
    // TODO: integrate VehicleController execution hooks without changing controller APIs first.
    // TODO: integrate TrajectoryPlanner helpers and negotiation state after the service skeleton is stable.
};

}  // namespace mcm

}  // namespace artery

#endif /* ARTERY_MCM_MCAPPLICATION_H_ */
