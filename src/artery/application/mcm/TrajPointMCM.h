#ifndef TRAJPOINTMCM_H_
#define TRAJPOINTMCM_H_
#include <omnetpp/simtime.h>
namespace artery
{
using namespace omnetpp;
namespace mcm
{

    class TrajPointMCM
{
public:
    double mX;   
    double mY;
    double mHeading;
    //omnetpp::SimTime mTime;
    omnetpp::SimTime mTime;
};

} // namespace mcm
} // namespace artery

#endif /* TRAJPOINTMCM_H_ */
