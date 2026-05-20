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
    double mX = 0.0;
    double mY = 0.0;
    double mHeading = 0.0;
    omnetpp::SimTime mTime;
};

} // namespace mcm
} // namespace artery

#endif /* TRAJPOINTMCM_H_ */
