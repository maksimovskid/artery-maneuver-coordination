#ifndef ARTERY_CAM_MCM_AWARENESS_H
#define ARTERY_CAM_MCM_AWARENESS_H
#include "artery/application/LocalDynamicMapMCM.h"
namespace artery {
/**
 * Read-only eligibility for background CAM reduction: at least one unexpired
 * negotiation container must name participants other than ego. ID1 is required;
 * ID2 is checked only when present (absence must not exclude station zero).
 * Sender/RV is not an extra exclusion. Like the reference, this recognizes any
 * negotiation container, not just Requests. The current LDM keeps each sender's
 * latest packet with generation-time +2 s expiry; check expiry even before its
 * next cleanup, using the same expiry<now boundary. Missing map is safe/inactive.
 *
 * This reports awareness, not a negotiated agreement or a priority decision.
 * An existing policy hold still uses its own time-and-CBR recovery conditions.
 */
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
