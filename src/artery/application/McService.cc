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
#include <vanetza/asn1/support/per_support.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

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

const asn_per_constraints_t scVehicleAutomationLevelConstraints = {
    { asn_per_constraint_t::APC_CONSTRAINED, 3, 3, 0, 5 },
    { asn_per_constraint_t::APC_UNCONSTRAINED, -1, -1, 0, 0 },
    nullptr, nullptr
};
const asn_INTEGER_enum_map_t scVehicleAutomationLevelValueMap[] = {
    { VehicleAutomationLevel_notAvailable, 12, "notAvailable" },
    { VehicleAutomationLevel_saeLevel1, 9, "saeLevel1" },
    { VehicleAutomationLevel_saeLevel2, 9, "saeLevel2" },
    { VehicleAutomationLevel_saeLevel3, 9, "saeLevel3" },
    { VehicleAutomationLevel_saeLevel4, 9, "saeLevel4" },
    { VehicleAutomationLevel_saeLevel5, 9, "saeLevel5" }
};
const unsigned int scVehicleAutomationLevelNameMap[] = {
    0, 1, 2, 3, 4, 5
};
const asn_INTEGER_specifics_t scVehicleAutomationLevelSpecifics = {
    scVehicleAutomationLevelValueMap,
    scVehicleAutomationLevelNameMap,
    6,
    0,
    1,
    0,
    0
};
const asn_per_constraints_t scTrajectoryOffsetPointConstraints = {
    { asn_per_constraint_t::APC_CONSTRAINED, 22, 22, -2000000, 2000000 },
    { asn_per_constraint_t::APC_UNCONSTRAINED, -1, -1, 0, 0 },
    nullptr, nullptr
};
const asn_per_constraints_t scTrajectoryOffsetHeadingConstraints = {
    { asn_per_constraint_t::APC_CONSTRAINED, 12, 12, -1800, 1800 },
    { asn_per_constraint_t::APC_UNCONSTRAINED, -1, -1, 0, 0 },
    nullptr, nullptr
};
const asn_per_constraints_t scTrajectoryOffsetTimeConstraints = {
    { asn_per_constraint_t::APC_CONSTRAINED, 15, 15, 0, 30000 },
    { asn_per_constraint_t::APC_UNCONSTRAINED, -1, -1, 0, 0 },
    nullptr, nullptr
};
const asn_per_constraints_t scTrajectoryMcmConstraints = {
    { asn_per_constraint_t::APC_UNCONSTRAINED, -1, -1, 0, 0 },
    { asn_per_constraint_t::APC_CONSTRAINED, 5, 5, 1, 20 },
    nullptr, nullptr
};

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

void configureExperimentalMcmPerConstraints()
{
    // MCMextra descriptors miss generated PER metadata for these mandatory minimal MCM fields.
    asn_DEF_VehicleAutomationLevel.encoding_constraints.per_constraints = &scVehicleAutomationLevelConstraints;
    asn_DEF_VehicleAutomationLevel.specifics = &scVehicleAutomationLevelSpecifics;
    asn_DEF_TrajectoryOffsetPoint.encoding_constraints.per_constraints = &scTrajectoryOffsetPointConstraints;
    asn_DEF_TrajectoryOffsetHeading.encoding_constraints.per_constraints = &scTrajectoryOffsetHeadingConstraints;
    asn_DEF_TrajectoryOffsetTime.encoding_constraints.per_constraints = &scTrajectoryOffsetTimeConstraints;
    asn_DEF_TrajectoryMCM.encoding_constraints.per_constraints = &scTrajectoryMcmConstraints;
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

    configureExperimentalMcmPerConstraints();
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
    EV_INFO << "Sending minimal MCM for station " << obj.asn1()->header.stationID << " at " << T_now << '\n';
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

    // TODO: re-enable validation once MCM trajectory offset descriptors expose safe constraint hooks.

    return message;
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
