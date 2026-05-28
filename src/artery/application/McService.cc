#include "artery/application/McService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/McObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McApplication.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/round.h"

#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <omnetpp.h>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

#include <string>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");

namespace
{

constexpr long scMcmMessageId = 20;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

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

    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mLastMcmTimestamp = simTime();
    mApplication.reset(new mcm::McApplication());
    mApplication->initialize(&getFacilities().get_mutable<traci::VehicleController>());
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::MDM);
}

void McService::trigger()
{
    Enter_Method("trigger");

    uint16_t generationDeltaTime = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto mcm = createMinimalIntentionSharingMessage(*mVehicleDataProvider, generationDeltaTime);
    McObject obj(std::move(mcm));

    EV_INFO << "Created valid minimal MCM for station " << obj.asn1()->header.stationID << " at " << simTime() << '\n';
    mLastMcmTimestamp = simTime();

    // TODO: integrate trajectory planning and negotiation logic through McApplication, not here.
    // TODO: emit McmSent only when this dry-run path is replaced by real BTP transmission.
    (void) scSignalMcmSent;
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

    Asn1PacketVisitor<Mcm> visitor;
    const Mcm* mcm = boost::apply_visitor(visitor, *packet);
    if (mcm && mcm->validate()) {
        McObject obj = visitor.shared_wrapper;
        emit(scSignalMcmReceived, &obj);
        // TODO: pass decoded MCMs to McApplication for send/receive state handling.
    }
}

}  // namespace artery
