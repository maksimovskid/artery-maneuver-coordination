#include "artery/application/McService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/McObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/mcm/McApplication.h"

#include <omnetpp.h>
#include <vanetza/common/its_aid.hpp>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");

Define_Module(McService)

McService::McService() = default;

McService::~McService() = default;

void McService::initialize()
{
    ItsG5BaseService::initialize();

    mLastMcmTimestamp = simTime();
    mApplication.reset(new mcm::McApplication());
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::MDM);
}

void McService::trigger()
{
    Enter_Method("trigger");

    // TODO: create and send minimal MCM messages after the ASN.1 content builder is introduced.
    // TODO: integrate trajectory planning and negotiation logic through McApplication, not here.
    (void) mPrimaryChannel;
    (void) mLastMcmTimestamp;
    (void) scSignalMcmSent;
}

void McService::indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");

    Asn1PacketVisitor<Mcm> visitor;
    const Mcm* mcm = boost::apply_visitor(visitor, *packet);
    if (mcm && mcm->validate()) {
        McObject obj = visitor.shared_wrapper;
        emit(scSignalMcmReceived, &obj);
        // TODO: pass decoded MCMs to McApplication for send/receive state handling.
    }
}

}  // namespace artery
