#include "artery/application/McService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/McObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/NetworkInterfaceTable.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McApplication.h"
#include "artery/utility/round.h"

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp.h>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

#include <algorithm>
#include <chrono>
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
    mcm::operationMode operationMode = mcm::operationMode::Unknown;
    bool hasNegotiationContainer = false;
    bool hasExecutionContainer = false;
    long mcmCategory = -1;
    long priorityManeuver = -1;
    long cooperationTypeMcm = -1;
    long requestId = -1;
    long numberOfVehicles = -1;
    uint32_t negotiationVehicleId1 = 0;
    bool hasNegotiationVehicleId2 = false;
    uint32_t negotiationVehicleId2 = 0;
    std::size_t requestedTrajectoryPointCount = 0;
    std::size_t offeredTrajectoryPointCount = 0;
    bool hasAlternativeTrajectory = false;
    std::size_t alternativeTrajectoryPointCount = 0;
    long cooperationId = -1;
    uint32_t cooperationVehicleId1 = 0;
    bool hasCooperationVehicleId2 = false;
    uint32_t cooperationVehicleId2 = 0;
};

McmOperationMetadata getMcmOperationMetadata(const McmParameters_t& parameters)
{
    McmOperationMetadata metadata;
    metadata.hasNegotiationContainer = parameters.maneuverNegotiationContainer != nullptr;
    metadata.hasExecutionContainer = parameters.maneuverExecutionContainer != nullptr;

    if (parameters.maneuverExecutionContainer) {
        const ManeuverExecutionContainer_t& execution = *parameters.maneuverExecutionContainer;
        metadata.operationMode = mcm::operationMode::ManeuverExecutionMode;
        metadata.mcmCategory = execution.mcmCategory;
        metadata.priorityManeuver = execution.priorityManeuver;
        metadata.cooperationId = execution.cooperationID;
        metadata.cooperationVehicleId1 = static_cast<uint32_t>(execution.cooperationVehicleID1);
        metadata.hasCooperationVehicleId2 = execution.cooperationVehicleID2 != nullptr;
        if (execution.cooperationVehicleID2) {
            metadata.cooperationVehicleId2 = static_cast<uint32_t>(*execution.cooperationVehicleID2);
        }
    } else if (parameters.maneuverNegotiationContainer) {
        const ManeuverNegotiationContainer_t& negotiation = *parameters.maneuverNegotiationContainer;
        metadata.operationMode = mcm::operationMode::ManeuverNegotiationMode;
        metadata.mcmCategory = negotiation.mcmCategory;
        metadata.priorityManeuver = negotiation.priorityManeuver;
        metadata.cooperationTypeMcm = negotiation.cooperationTypeMCM;
        metadata.requestId = negotiation.requestID;
        metadata.numberOfVehicles = negotiation.numberOfVehicles;
        metadata.negotiationVehicleId1 = static_cast<uint32_t>(negotiation.negotiationVehicleID1);
        metadata.hasNegotiationVehicleId2 = negotiation.negotiationVehicleID2 != nullptr;
        if (negotiation.negotiationVehicleID2) {
            metadata.negotiationVehicleId2 = static_cast<uint32_t>(*negotiation.negotiationVehicleID2);
        }
        metadata.requestedTrajectoryPointCount = static_cast<std::size_t>(negotiation.requestedTrajectory.list.count);
        metadata.offeredTrajectoryPointCount = static_cast<std::size_t>(negotiation.offeredTrajectory.list.count);
        metadata.hasAlternativeTrajectory = negotiation.alternativeTrajectory != nullptr;
        if (negotiation.alternativeTrajectory) {
            metadata.alternativeTrajectoryPointCount = static_cast<std::size_t>(negotiation.alternativeTrajectory->list.count);
        }
    } else {
        metadata.operationMode = mcm::operationMode::IntentionSharingMode;
    }

    return metadata;
}

mcm::McmSnapshot extractMcmSnapshot(const MCM_t& message)
{
    const BasicContainerMCM_t& basic = message.mcm.mcmParameters.basicContainerMCM;
    const IntentSharingContainer_t& intent = message.mcm.mcmParameters.intentionSharingContainer;
    const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);

    mcm::McmSnapshot snapshot;
    snapshot.stationId = static_cast<uint32_t>(message.header.stationID);
    snapshot.generationDeltaTime = static_cast<uint16_t>(message.mcm.generationDeltaTime);
    snapshot.longitude = basic.referencePosition.longitude;
    snapshot.latitude = basic.referencePosition.latitude;
    snapshot.speedValue = intent.speed.speedValue;
    snapshot.headingValue = intent.heading.headingValue;
    snapshot.plannedTrajectoryPointCount = static_cast<std::size_t>(intent.plannedTrajectory.list.count);
    snapshot.operationMode = metadata.operationMode;
    snapshot.hasNegotiationContainer = metadata.hasNegotiationContainer;
    snapshot.hasExecutionContainer = metadata.hasExecutionContainer;
    snapshot.mcmCategory = metadata.mcmCategory;
    snapshot.priorityManeuver = metadata.priorityManeuver;
    snapshot.cooperationTypeMcm = metadata.cooperationTypeMcm;
    snapshot.requestId = metadata.requestId;
    snapshot.numberOfVehicles = metadata.numberOfVehicles;
    snapshot.negotiationVehicleId1 = metadata.negotiationVehicleId1;
    snapshot.hasNegotiationVehicleId2 = metadata.hasNegotiationVehicleId2;
    snapshot.negotiationVehicleId2 = metadata.negotiationVehicleId2;
    snapshot.requestedTrajectoryPointCount = metadata.requestedTrajectoryPointCount;
    snapshot.offeredTrajectoryPointCount = metadata.offeredTrajectoryPointCount;
    snapshot.hasAlternativeTrajectory = metadata.hasAlternativeTrajectory;
    snapshot.alternativeTrajectoryPointCount = metadata.alternativeTrajectoryPointCount;
    snapshot.cooperationId = metadata.cooperationId;
    snapshot.cooperationVehicleId1 = metadata.cooperationVehicleId1;
    snapshot.hasCooperationVehicleId2 = metadata.hasCooperationVehicleId2;
    snapshot.cooperationVehicleId2 = metadata.cooperationVehicleId2;
    return snapshot;
}

mcm::SentMcm makeSentMcm(const MCM_t& message, omnetpp::SimTime sentAt)
{
    return { extractMcmSnapshot(message), sentAt };
}

mcm::ReceivedMcm makeReceivedMcm(const MCM_t& message, omnetpp::SimTime receivedAt)
{
    return { extractMcmSnapshot(message), receivedAt };
}

void appendZeroTrajectoryPoint(TrajectoryMCM_t& trajectory)
{
    TrajectoryPointMCM_t* point = vanetza::asn1::allocate<TrajectoryPointMCM_t>();
    point->deltaLongitudinalPosition = 0;
    point->deltaLateralPosition = 0;
    point->deltaHeading = 0;
    point->deltaTime = 0;
    ASN_SEQUENCE_ADD(&trajectory, point);
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

McService::McService() :
    mGenMcmMin { 100, SIMTIME_MS },
    mGenMcmMax { 1000, SIMTIME_MS },
    mGenMcm(mGenMcmMax)
{
}

McService::~McService() = default;

void McService::initialize()
{
    ItsG5BaseService::initialize();

    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(scExperimentalMcmAid);

    mLastMcmTimestamp = simTime();
    mGenMcmMin = par("minInterval");
    mGenMcmMax = par("maxInterval");
    mGenMcm = mGenMcmMax;
    mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
    mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
    mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;
    mDccRestriction = par("withDccRestriction");
    mFixedRate = par("fixedRate");
    mFixedRateInterval = par("fixedRateInterval");
    mSendNegotiationTestMcm = par("sendNegotiationTestMcm").boolValue();
    mApplication.reset(new mcm::McApplication());
    mApplication->initialize(nullptr);
    // TODO: wire VehicleController later when execution logic needs it and facility initialization order is confirmed.
}

void McService::trigger()
{
    Enter_Method("trigger");
    mApplication->tick(simTime());
    checkTriggeringConditions(simTime());
}

void McService::checkTriggeringConditions(const SimTime& T_now)
{
    SimTime& T_GenMcm = mGenMcm;
    const SimTime& T_GenMcmMin = mGenMcmMin;
    const SimTime& T_GenMcmMax = mGenMcmMax;
    const SimTime T_GenMcmDcc = mDccRestriction ? genMcmDcc() : T_GenMcmMin;
    const SimTime T_elapsed = T_now - mLastMcmTimestamp;
    const SimTime fixedInterval = mFixedRateInterval > SIMTIME_ZERO ? mFixedRateInterval : T_GenMcmMin;

    if (T_elapsed >= T_GenMcmDcc) {
        if (mFixedRate) {
            if (T_elapsed >= fixedInterval) {
                sendMcm(T_now);
            }
        } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
            sendMcm(T_now);
            T_GenMcm = std::min(T_elapsed, T_GenMcmMax);
            mGenMcmLowDynamicsCounter = 0;
        } else if (T_elapsed >= T_GenMcm) {
            sendMcm(T_now);
            if (++mGenMcmLowDynamicsCounter >= mGenMcmLowDynamicsLimit) {
                T_GenMcm = T_GenMcmMax;
            }
        }
    }
}

bool McService::checkHeadingDelta() const
{
    return !vanetza::facilities::similar_heading(mLastMcmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool McService::checkPositionDelta() const
{
    return distance(mLastMcmPosition, mVehicleDataProvider->position()) > mPositionDelta;
}

bool McService::checkSpeedDelta() const
{
    return abs(mLastMcmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void McService::sendMcm(const SimTime& T_now)
{
    uint16_t generationDeltaTime = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto mcm = mSendNegotiationTestMcm ?
        createMinimalNegotiationTestMessage(*mVehicleDataProvider, generationDeltaTime) :
        createMinimalIntentionSharingMessage(*mVehicleDataProvider, generationDeltaTime);
    if (mSendNegotiationTestMcm) {
        EV_DETAIL << "Sending minimal negotiation test MCM for station "
            << (*mcm).header.stationID << " at " << T_now << '\n';
    }

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
    mApplication->handleSentMcm(makeSentMcm(message, T_now));
    EV_DETAIL << "Sending minimal MCM for station " << obj.asn1()->header.stationID << " at " << T_now << '\n';
    mLastMcmPosition = mVehicleDataProvider->position();
    mLastMcmSpeed = mVehicleDataProvider->speed();
    mLastMcmHeading = mVehicleDataProvider->heading();
    mLastMcmTimestamp = T_now;

    using McmByteBuffer = convertible::byte_buffer_impl<asn1::Mcm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new McmByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}

void McService::fillHeader(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = scMcmMessageId;
    header.stationID = vdp.station_id();
}

void McService::fillGenerationDeltaTime(vanetza::asn1::Mcm& message, uint16_t generationDeltaTime) const
{
    ManeuverCoordinationMessage_t& mcm = (*message).mcm;
    mcm.generationDeltaTime = generationDeltaTime * GenerationDeltaTime_oneMilliSec;
}

void McService::fillBasicContainer(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    BasicContainerMCM_t& basic = (*message).mcm.mcmParameters.basicContainerMCM;

    basic.stationType = StationType_passengerCar;
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
}

void McService::fillIntentionSharingContainer(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    IntentSharingContainer_t& intent = (*message).mcm.mcmParameters.intentionSharingContainer;

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
    appendZeroTrajectoryPoint(intent.plannedTrajectory);
}

void McService::addManeuverNegotiationContainer(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    ManeuverNegotiationContainer_t*& negotiation = (*message).mcm.mcmParameters.maneuverNegotiationContainer;

    negotiation = vanetza::asn1::allocate<ManeuverNegotiationContainer_t>();
    negotiation->mcmCategory = McmCategory_request;
    negotiation->priorityManeuver = PriorityManeuver_low;
    negotiation->cooperationTypeMCM = CooperationTypeMCM_agreementSeeking;
    negotiation->requestID = 0;
    negotiation->numberOfVehicles = 1;
    negotiation->negotiationVehicleID1 = vdp.station_id();

    appendZeroTrajectoryPoint(negotiation->requestedTrajectory);
    appendZeroTrajectoryPoint(negotiation->offeredTrajectory);
}

void McService::addManeuverExecutionContainer(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    ManeuverExecutionContainer_t*& execution = (*message).mcm.mcmParameters.maneuverExecutionContainer;

    execution = vanetza::asn1::allocate<ManeuverExecutionContainer_t>();
    execution->mcmCategory = McmCategory_execute;
    execution->priorityManeuver = PriorityManeuver_low;
    execution->cooperationID = 0;
    execution->cooperationVehicleID1 = vdp.station_id();
}

vanetza::asn1::Mcm McService::createMinimalIntentionSharingMessage(const VehicleDataProvider& vdp, uint16_t generationDeltaTime) const
{
    vanetza::asn1::Mcm message;

    fillHeader(message, vdp);
    fillGenerationDeltaTime(message, generationDeltaTime);
    fillBasicContainer(message, vdp);
    fillIntentionSharingContainer(message, vdp);

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid minimal MCM: %s", error.c_str());
    }

    return message;
}

vanetza::asn1::Mcm McService::createMinimalNegotiationTestMessage(const VehicleDataProvider& vdp, uint16_t generationDeltaTime) const
{
    vanetza::asn1::Mcm message = createMinimalIntentionSharingMessage(vdp, generationDeltaTime);

    addManeuverNegotiationContainer(message, vdp);

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid minimal negotiation test MCM: %s", error.c_str());
    }

    return message;
}

vanetza::asn1::Mcm McService::createMinimalExecutionTestMessage(const VehicleDataProvider& vdp, uint16_t generationDeltaTime) const
{
    vanetza::asn1::Mcm message = createMinimalIntentionSharingMessage(vdp, generationDeltaTime);

    addManeuverExecutionContainer(message, vdp);

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid minimal execution test MCM: %s", error.c_str());
    }

    return message;
}

SimTime McService::genMcmDcc()
{
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for MC's primary channel %i", mPrimaryChannel);
    }

    static const vanetza::dcc::TransmissionLite mc_tx(vanetza::dcc::Profile::DP2, 0);
    vanetza::Clock::duration interval = trc->interval(mc_tx);
    SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
    return std::min(mGenMcmMax, std::max(mGenMcmMin, dcc));
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
        mApplication->handleReceivedMcm(makeReceivedMcm(message, simTime()));
    } catch (const std::exception& e) {
        EV_WARN << "McService receive: exception while decoding or validating MCM: " << e.what() << '\n';
    } catch (...) {
        EV_WARN << "McService receive: unknown exception while decoding or validating MCM\n";
    }
}

}  // namespace artery
