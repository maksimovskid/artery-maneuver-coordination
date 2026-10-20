#ifndef ARTERY_CAM_MCM_AWARENESS_H
#define ARTERY_CAM_MCM_AWARENESS_H
#include "artery/application/LocalDynamicMapMCM.h"
namespace artery {
inline bool camObservesNegotiation(const LocalDynamicMapMCM::AwarenessEntries* entries,
    uint32_t ego, omnetpp::SimTime now)
{
    if (!entries) return false;
    for (const auto& item : *entries) {
        if (item.second.expiry() < now) continue;
        const auto* negotiation = item.second.mcm()->mcm.mcmParameters.maneuverNegotiationContainer;
        if (negotiation && negotiation->negotiationVehicleID1 != ego &&
            (!negotiation->negotiationVehicleID2 || *negotiation->negotiationVehicleID2 != ego)) return true;
    }
    return false;
}
}
#endif
