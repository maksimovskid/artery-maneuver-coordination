#ifndef ARTERY_MCSERVICE_H_
#define ARTERY_MCSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"

#include <omnetpp/simtime.h>
#include <vanetza/asn1/mcm.hpp>
#include <vanetza/btp/data_interface.hpp>

#include <memory>

namespace artery
{

namespace mcm
{
class McApplication;
}

class VehicleDataProvider;

class McService : public ItsG5BaseService
{
public:
    McService();
    ~McService() override;
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;

private:
    vanetza::asn1::Mcm createMinimalIntentionSharingMessage(const VehicleDataProvider&, uint16_t generationDeltaTime) const;

    ChannelNumber mPrimaryChannel = channel::CCH;
    omnetpp::SimTime mLastMcmTimestamp;
    std::unique_ptr<mcm::McApplication> mApplication;
};

}  // namespace artery

#endif /* ARTERY_MCSERVICE_H_ */
