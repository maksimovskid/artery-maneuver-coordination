#ifndef ARTERY_MCM_RETRANSMISSION_H
#define ARTERY_MCM_RETRANSMISSION_H
#include "artery/application/mcm/McApplication.h"
#include "artery/application/mcm/McmTrajectory.h"
#include <vanetza/asn1/mcm.hpp>
#include <map>
#include <memory>
#include <stdexcept>

namespace artery { namespace mcm {
// Request and Offer retransmission repeats the ENTIRE issued ASN.1 message.
// In particular neither its generationDeltaTime nor its intent is refreshed.
// Only two entries are retained; a fresh proposal replaces the role's entry.
class McmRetransmissionCache {
public:
    struct Packet {
        PendingMcmCommand command;
        std::shared_ptr<const vanetza::asn1::Mcm> message;
        omnetpp::SimTime generatedAt;
    };
    void remember(const PendingMcmCommand& command, std::shared_ptr<const vanetza::asn1::Mcm> message,
        omnetpp::SimTime generatedAt)
    {
        if (command.retransmission || (command.subtype != mcmSubtype::Request && command.subtype != mcmSubtype::Offer))
            throw std::logic_error("only a fresh Request/Offer can become an issued packet");
        mPackets.insert_or_assign(command.subtype, Packet {command, message, generatedAt});
    }
    const Packet& replay(const PendingMcmCommand& command) const
    {
        auto found = mPackets.find(command.subtype);
        if (!command.retransmission || found == mPackets.end())
            throw std::logic_error("missing issued MCM for retransmission");
        const auto& issued = found->second.command;
        if (command.kind != issued.kind || command.requestId != issued.requestId ||
            command.priority != issued.priority || command.cooperationType != issued.cooperationType ||
            command.numberOfVehicles != issued.numberOfVehicles || command.targetVehicle1 != issued.targetVehicle1 ||
            command.hasTargetVehicle2 != issued.hasTargetVehicle2 || command.targetVehicle2 != issued.targetVehicle2 ||
            command.hasOfferedTrajectory != issued.hasOfferedTrajectory ||
            !sameMcmProposal(command.requestedTrajectory, issued.requestedTrajectory) ||
            !sameMcmProposal(command.offeredTrajectory, issued.offeredTrajectory))
            throw std::logic_error("changed proposal cannot retransmit an issued MCM");
        return found->second;
    }
private:
    std::map<mcmSubtype, Packet> mPackets;
};
} }
#endif
