#include "artery/application/McService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/McObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McApplication.h"
#include "artery/utility/round.h"

#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp.h>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

#include <exception>
#include <string>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");

namespace
{

constexpr long scMcmMessageId = 20;

// Experimental MCM/MCS ITS-AID placeholder until an official or project-specific value is introduced.
constexpr vanetza::ItsAid scExperimentalMcmAid = 650;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

struct McmOperationMetadata {
    mcm::McmOperationMode operationMode = mcm::McmOperationMode::Unknown;
    bool hasNegotiationContainer = false;
    bool hasExecutionContainer = false;
    long mcmCategory = -1;
    long priorityManeuver = -1;
};

McmOperationMetadata getMcmOperationMetadata(const McmParameters_t& parameters)
{
    McmOperationMetadata metadata;
    metadata.hasNegotiationContainer = parameters.maneuverNegotiationContainer != nullptr;
    metadata.hasExecutionContainer = parameters.maneuverExecutionContainer != nullptr;

    if (parameters.maneuverExecutionContainer) {
        metadata.operationMode = mcm::McmOperationMode::ManeuverExecution;
        metadata.mcmCategory = parameters.maneuverExecutionContainer->mcmCategory;
        metadata.priorityManeuver = parameters.maneuverExecutionContainer->priorityManeuver;
    } else if (parameters.maneuverNegotiationContainer) {
        metadata.operationMode = mcm::McmOperationMode::ManeuverNegotiation;
        metadata.mcmCategory = parameters.maneuverNegotiationContainer->mcmCategory;
        metadata.priorityManeuver = parameters.maneuverNegotiationContainer->priorityManeuver;
    } else {
        metadata.operationMode = mcm::McmOperationMode::IntentSharing;
    }

    return metadata;
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
    static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
    static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

    SpeedValue_t speed = SpeedValue_unavailable;
    if (v >= upper) {
        speed = 16382;
    } else if (v >= lower) {
        speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
    }
    return speed;
}

}  // namespace

Define_Module(McService)

McService::McService() = default;

McService::~McService() = default;

void McService::initialize()
{
    ItsG5BaseService::initialize();

    mLastMcmTimestamp = simTime();
    mApplication.reset(new mcm::McApplication());
    // TODO: wire VehicleController later when execution logic needs it and facility initialization order is confirmed.
}

void McService::trigger()
{
    Enter_Method("trigger");
    if (!mVehicleDataProvider) {
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mTimer = &getFacilities().get_const<Timer>();
        mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(scExperimentalMcmAid);
    }
    sendMcm(simTime());
}

void McService::sendMcm(const SimTime& T_now)
{
    uint16_t generationDeltaTime = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto mcm = createMinimalIntentionSharingMessage(*mVehicleDataProvider, generationDeltaTime);

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::MCM;
    request.gn.its_aid = scExperimentalMcmAid;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    McObject obj(std::move(mcm));
    emit(scSignalMcmSent, &obj);
    const MCM_t& message = *obj.asn1();
    const BasicContainerMCM_t& basic = message.mcm.mcmParameters.basicContainerMCM;
    const IntentSharingContainer_t& intent = message.mcm.mcmParameters.intentionSharingContainer;
    const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);
    // Copy only value data across the service/application boundary; do not retain ASN.1 pointers.
    const mcm::SentMcm snapshot {
        static_cast<uint32_t>(message.header.stationID),
        static_cast<uint16_t>(message.mcm.generationDeltaTime),
        basic.referencePosition.longitude,
        basic.referencePosition.latitude,
        intent.speed.speedValue,
        intent.heading.headingValue,
        static_cast<std::size_t>(intent.plannedTrajectory.list.count),
        metadata.operationMode,
        metadata.hasNegotiationContainer,
        metadata.hasExecutionContainer,
        metadata.mcmCategory,
        metadata.priorityManeuver,
        T_now
    };
    mApplication->handleSentMcm(snapshot);
    EV_DETAIL << "Sending minimal MCM for station " << obj.asn1()->header.stationID << " at " << T_now << '\n';
    mLastMcmTimestamp = T_now;

    using McmByteBuffer = convertible::byte_buffer_impl<asn1::Mcm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new McmByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}

vanetza::asn1::Mcm McService::createMinimalIntentionSharingMessage(const VehicleDataProvider& vdp, uint16_t generationDeltaTime) const
{
    vanetza::asn1::Mcm message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = scMcmMessageId;
    header.stationID = vdp.station_id();

    ManeuverCoordinationMessage_t& mcm = (*message).mcm;
    mcm.generationDeltaTime = generationDeltaTime * GenerationDeltaTime_oneMilliSec;

    BasicContainerMCM_t& basic = mcm.mcmParameters.basicContainerMCM;
    basic.stationType = StationType_passengerCar;
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

    IntentSharingContainer_t& intent = mcm.mcmParameters.intentionSharingContainer;
    intent.heading.headingValue = round(vdp.heading(), decidegree);
    intent.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    intent.speed.speedValue = buildSpeedValue(vdp.speed());
    intent.speed.speedConfidence = SpeedConfidence_unavailable;
    intent.driveDirection = vdp.speed().value() >= 0.0 ? DriveDirection_forward : DriveDirection_backward;
    intent.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    intent.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    intent.vehicleWidth = VehicleWidth_unavailable;
    intent.vehicleAutomationLevel = VehicleAutomationLevel_notAvailable;

    // TODO: replace this placeholder with TrajectoryPlanner output when planned trajectories are integrated.
    TrajectoryPointMCM_t* point = vanetza::asn1::allocate<TrajectoryPointMCM_t>();
    point->deltaLongitudinalPosition = 0;
    point->deltaLateralPosition = 0;
    point->deltaHeading = 0;
    point->deltaTime = 0;
    ASN_SEQUENCE_ADD(&intent.plannedTrajectory, point);

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid minimal MCM: %s", error.c_str());
    }

    return message;
}

void McService::indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");

    try {
        Asn1PacketVisitor<Mcm> visitor;
        const Mcm* decodedMcm = boost::apply_visitor(visitor, *packet);
        if (!decodedMcm) {
            EV_WARN << "McService receive: decoded MCM is null, dropping packet\n";
            return;
        }

        std::string error;
        if (!decodedMcm->validate(error)) {
            EV_WARN << "McService receive: invalid MCM, dropping packet: " << error << '\n';
            return;
        }

        EV_DETAIL << "McService receive: decoded and validated MCM, emitting McmReceived\n";
        McObject obj = visitor.shared_wrapper;
        emit(scSignalMcmReceived, &obj);

        const MCM_t& message = *obj.asn1();
        const BasicContainerMCM_t& basic = message.mcm.mcmParameters.basicContainerMCM;
        const IntentSharingContainer_t& intent = message.mcm.mcmParameters.intentionSharingContainer;
        const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);
        // Copy only value data across the service/application boundary; do not retain ASN.1 pointers.
        const mcm::ReceivedMcm snapshot {
            static_cast<uint32_t>(message.header.stationID),
            static_cast<uint16_t>(message.mcm.generationDeltaTime),
            basic.referencePosition.longitude,
            basic.referencePosition.latitude,
            intent.speed.speedValue,
            intent.heading.headingValue,
            static_cast<std::size_t>(intent.plannedTrajectory.list.count),
            metadata.operationMode,
            metadata.hasNegotiationContainer,
            metadata.hasExecutionContainer,
            metadata.mcmCategory,
            metadata.priorityManeuver,
            simTime()
        };
        mApplication->handleReceivedMcm(snapshot);
    } catch (const std::exception& e) {
        EV_WARN << "McService receive: exception while decoding or validating MCM: " << e.what() << '\n';
    } catch (...) {
        EV_WARN << "McService receive: unknown exception while decoding or validating MCM\n";
    }
}

}  // namespace artery
