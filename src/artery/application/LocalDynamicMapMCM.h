#ifndef ARTERY_LOCALDYNAMICMAPMCM_H_
#define ARTERY_LOCALDYNAMICMAPMCM_H_

#include "artery/application/McObject.h"

#include <omnetpp/simtime.h>

#include <cstdint>
#include <map>
#include <memory>

namespace artery
{

class Timer;

class LocalDynamicMapMCM
{
public:
    using StationID = uint32_t;
    using Mcm = vanetza::asn1::Mcm;

    class AwarenessEntry
    {
    public:
        AwarenessEntry(const McObject&, omnetpp::SimTime receivedAt, omnetpp::SimTime expiry);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime receivedAt() const { return mReceivedAt; }
        omnetpp::SimTime expiry() const { return mExpiry; }
        const Mcm& mcm() const { return mObject.asn1(); }
        std::shared_ptr<const Mcm> mcmPtr() const { return mObject.shared_ptr(); }
        const McObject& object() const { return mObject; }

    private:
        omnetpp::SimTime mReceivedAt;
        omnetpp::SimTime mExpiry;
        McObject mObject;
    };

    using AwarenessEntries = std::map<StationID, AwarenessEntry>;

    LocalDynamicMapMCM(const Timer&);

    void updateAwarenessMCM(const McObject&);
    void dropExpired();
    std::shared_ptr<const Mcm> getMcm(StationID) const;
    const AwarenessEntries* localMapMCM() const { return &mMcmMessages; }
    const AwarenessEntries& allEntries() const { return mMcmMessages; }
    std::size_t size() const { return mMcmMessages.size(); }

private:
    const Timer& mTimer;
    AwarenessEntries mMcmMessages;
};

}  // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAPMCM_H_ */
