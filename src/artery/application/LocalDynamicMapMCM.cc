#include "artery/application/LocalDynamicMapMCM.h"

#include "artery/application/Timer.h"

#include <omnetpp.h>

namespace artery
{

LocalDynamicMapMCM::LocalDynamicMapMCM(const Timer& timer) :
    mTimer(timer)
{
}

void LocalDynamicMapMCM::updateAwarenessMCM(const McObject& obj)
{
    const vanetza::asn1::Mcm& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 2000, omnetpp::SIMTIME_MS };
    const auto tai = mTimer.reconstructMilliseconds(msg->mcm.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;
    const auto now = omnetpp::simTime();

    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT;
        EV_WARN << "Expiry of received MCM is out of bounds for station "
            << msg->header.stationID << '\n';
        return;
    }

    AwarenessEntry entry(obj, now, expiry);
    auto found = mMcmMessages.find(msg->header.stationID);
    if (found != mMcmMessages.end()) {
        found->second = std::move(entry);
    } else {
        mMcmMessages.emplace(msg->header.stationID, std::move(entry));
    }

    EV_STATICCONTEXT;
    EV_DETAIL << "LocalDynamicMapMCM entry updated for station "
        << msg->header.stationID << "; entries=" << mMcmMessages.size() << '\n';
}

void LocalDynamicMapMCM::dropExpired()
{
    const auto now = omnetpp::simTime();
    std::size_t removed = 0;
    for (auto it = mMcmMessages.begin(); it != mMcmMessages.end();) {
        if (it->second.expiry() < now) {
            it = mMcmMessages.erase(it);
            ++removed;
        } else {
            ++it;
        }
    }

    if (removed > 0) {
        EV_STATICCONTEXT;
        EV_DETAIL << "LocalDynamicMapMCM expired entries removed: "
            << removed << "; entries=" << mMcmMessages.size() << '\n';
    }
}

std::shared_ptr<const LocalDynamicMapMCM::Mcm> LocalDynamicMapMCM::getMcm(StationID stationId) const
{
    auto found = mMcmMessages.find(stationId);
    if (found != mMcmMessages.end()) {
        return found->second.mcmPtr();
    }

    return nullptr;
}

LocalDynamicMapMCM::AwarenessEntry::AwarenessEntry(
    const McObject& obj,
    omnetpp::SimTime receivedAt,
    omnetpp::SimTime expiry) :
    mReceivedAt(receivedAt),
    mExpiry(expiry),
    mObject(obj)
{
}

}  // namespace artery
