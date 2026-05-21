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

    // TODO: add negotiation strategy and maneuver state handling.
    // TODO: add scenario-specific maneuver coordination behavior.
    // TODO: integrate TrajectoryPlanner helpers after the service-level MCM plumbing is stable.
};

}  // namespace mcm

}  // namespace artery

#endif /* ARTERY_MCM_MCAPPLICATION_H_ */
