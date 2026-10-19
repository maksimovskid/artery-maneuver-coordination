#ifndef TRAJPOINTMCM_H_
#define TRAJPOINTMCM_H_
#include <omnetpp/simtime.h>
namespace artery
{
namespace mcm
{

    class TrajPointMCM
{
public:
    // Default values avoid undefined data in partially constructed fallback trajectories.
    double mX = 0.0;
    double mY = 0.0;
    double mHeading = 0.0;
    omnetpp::SimTime mTime; // absolute sample time in simulation seconds
    omnetpp::SimTime mReferenceTime; // planning origin, preserved in retained proposals
};

} // namespace mcm
} // namespace artery

#endif /* TRAJPOINTMCM_H_ */
