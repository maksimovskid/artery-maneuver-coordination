#include "artery/application/McService.h"

#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/LocalDynamicMapMCM.h"
#include "artery/application/McObject.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/NetworkInterfaceTable.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/mcm/McApplication.h"
#include "artery/application/mcm/TrajectoryCsv.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/nic/RadioDriverBase.h"
#include "artery/utility/round.h"
#include "artery/traci/VehicleController.h"

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
#include <cmath>
#include <string>
#include <vector>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");
static const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");
static const simsignal_t scSignalMcmReceivedCounter = cComponent::registerSignal("McmReceivedCounter");
static const simsignal_t scSignalMcmSentCounter = cComponent::registerSignal("McmSentCounter");
static const simsignal_t scSignalMcmIntentionReceivedCounter = cComponent::registerSignal("McmIntentionReceivedCounter");
static const simsignal_t scSignalMcmIntentionSentCounter = cComponent::registerSignal("McmIntentionSentCounter");
static const simsignal_t scSignalMcmNegotiationReceivedCounter = cComponent::registerSignal("McmNegotiationReceivedCounter");
static const simsignal_t scSignalMcmNegotiationSentCounter = cComponent::registerSignal("McmNegotiationSentCounter");
static const simsignal_t scSignalMcmExecutionReceivedCounter = cComponent::registerSignal("McmExecutionReceivedCounter");
static const simsignal_t scSignalMcmExecutionSentCounter = cComponent::registerSignal("McmExecutionSentCounter");
static const simsignal_t scSignalMcmExecutionEmergencyReceivedCounter = cComponent::registerSignal("McmExecutionEmergencyReceivedCounter");
static const simsignal_t scSignalMcmExecutionEmergencySentCounter = cComponent::registerSignal("McmExecutionEmergencySentCounter");

static const simsignal_t scSignalMessageSizeMcmReceived = cComponent::registerSignal("msgsizeReceived");
static const simsignal_t scSignalEteDelayMcm = cComponent::registerSignal("EteDelayMcm");
static const simsignal_t scSignalEteDelayMcmNegotiation = cComponent::registerSignal("EteDelayMcmNegotiation");
static const simsignal_t scSignalEteDelayMcmExecution = cComponent::registerSignal("EteDelayMcmExecution");
static const simsignal_t scSignalEteDelayMcmEmergency = cComponent::registerSignal("EteDelayMcmEmergency");
static const simsignal_t scSignalCoopVehicleAgeOfInformation = cComponent::registerSignal("CoopVehicleAgeOfInformation");
static const simsignal_t scSignalDccTimeWaitNextMcm = cComponent::registerSignal("dccTimeWaitNextMcm");
static const simsignal_t scSignalCoopCbr = cComponent::registerSignal("coopCBR");

static const simsignal_t scSignalNegotiationStartedCounter = cComponent::registerSignal("NegotiationStartedCounter");
static const simsignal_t scSignalNegotiationCompletedCounter = cComponent::registerSignal("NegotiationCompletedCounter");
static const simsignal_t scSignalNegotiationTime = cComponent::registerSignal("negotiationTime");
static const simsignal_t scSignalExecutionStartedCounter = cComponent::registerSignal("ExecutionStartedCounter");
static const simsignal_t scSignalExecutionCompletedCounter = cComponent::registerSignal("ExecutionCompletedCounter");
static const simsignal_t scSignalCounterNegotiationRejected = cComponent::registerSignal("CounterNegotiationRejected");
static const simsignal_t scSignalCounterNegotiationTwoVehicles = cComponent::registerSignal("CounterNegotiationTwoVehicles");
static const simsignal_t scSignalCounterNegotiationThreeVehicles = cComponent::registerSignal("CounterNegotiationThreeVehicles");
static const simsignal_t scSignalTrajectoryCost = cComponent::registerSignal("TrajectoryCost");
static const simsignal_t scSignalTrajectoryCostRV = cComponent::registerSignal("TrajectoryCostRV");
static const simsignal_t scSignalSecondRequestStartedCounter = cComponent::registerSignal("SecondRequestStartedCounter");
static const simsignal_t scSignalSecondRequestCompletedCounter = cComponent::registerSignal("SecondRequestCompletedCounter");
static const simsignal_t scSignalSecondRequestRejectedCounter = cComponent::registerSignal("SecondRequestRejectedCounter");
static const simsignal_t scSignalCounterCoordPossiblePriorityLow = cComponent::registerSignal("CounterCoordPossiblePriorityLow");
static const simsignal_t scSignalCounterCoordPossiblePriorityMedium = cComponent::registerSignal("CounterCoordPossiblePriorityMedium");
static const simsignal_t scSignalCounterCoordPossiblePriorityHigh = cComponent::registerSignal("CounterCoordPossiblePriorityHigh");
static const simsignal_t scSignalCounterTrajectoryType0 = cComponent::registerSignal("CounterTrajectoryType0");
static const simsignal_t scSignalCounterTrajectoryType1 = cComponent::registerSignal("CounterTrajectoryType1");
static const simsignal_t scSignalCounterTrajectoryType2 = cComponent::registerSignal("CounterTrajectoryType2");
static const simsignal_t scSignalCounterTrajectoryType4 = cComponent::registerSignal("CounterTrajectoryType4");
static const simsignal_t scSignalCounterTrajectoryType5 = cComponent::registerSignal("CounterTrajectoryType5");
static const simsignal_t scSignalCounterTrajectoryType6 = cComponent::registerSignal("CounterTrajectoryType6");
static const simsignal_t scSignalCurrentMcsOperatingMode = cComponent::registerSignal("currentMCSoperatingMode");

namespace
{

constexpr long scMcmMessageId = 20;
constexpr int scMaxTrajectoryMcmPoints = 20;

// Experimental MCM/MCS ITS-AID placeholder until an official or project-specific value is introduced.
constexpr vanetza::ItsAid scExperimentalMcmAid = 650;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

long mcmSubtypeToAsnCategory(mcm::mcmSubtype subtype)
{
    switch (subtype) {
        case mcm::mcmSubtype::Request:
            return McmCategory_request;
        case mcm::mcmSubtype::Accept:
            return McmCategory_accept;
        case mcm::mcmSubtype::Reject:
            return McmCategory_reject;
        case mcm::mcmSubtype::Offer:
            return McmCategory_offer;
        case mcm::mcmSubtype::Confirm:
            return McmCategory_confirm;
        case mcm::mcmSubtype::Execute:
            return McmCategory_execute;
        case mcm::mcmSubtype::Cancel:
            return McmCategory_cancel;
        case mcm::mcmSubtype::Abort:
            return McmCategory_abort;
        case mcm::mcmSubtype::CascadingRequest:
            return McmCategory_cascadingRequest;
        case mcm::mcmSubtype::CascadingAccept:
            return McmCategory_cascadingAccept;
        case mcm::mcmSubtype::CascadingReject:
            return McmCategory_cascadingReject;
        case mcm::mcmSubtype::CascadingExecute:
            return McmCategory_cascadingExecute;
        case mcm::mcmSubtype::Regular:
            return McmCategory_request;
    }

    return McmCategory_request;
}

long priorityToAsnPriority(mcm::priorityMcmCategory priority)
{
    switch (priority) {
        case mcm::priorityMcmCategory::LowPriority:
            return PriorityManeuver_low;
        case mcm::priorityMcmCategory::MediumPriority:
            return PriorityManeuver_medium;
        case mcm::priorityMcmCategory::HighPriority:
            return PriorityManeuver_high;
        case mcm::priorityMcmCategory::EmergencyPriority:
            return PriorityManeuver_emergency;
        case mcm::priorityMcmCategory::NoPriority:
            return PriorityManeuver_low;
    }

    return PriorityManeuver_low;
}

McService::IntentTriggeringCondition parseIntentTriggeringCondition(const std::string& value)
{
    if (value == "SameAsCAMreduced2Hz") {
        return McService::IntentTriggeringCondition::SameAsCamReduced2Hz;
    }
    if (value == "SameAsCAMreduced1Hz") {
        return McService::IntentTriggeringCondition::SameAsCamReduced1Hz;
    }
    if (value == "PeriodicFixed") {
        return McService::IntentTriggeringCondition::PeriodicFixed;
    }
    if (value == "PeriodicFixed0.5Hz") {
        return McService::IntentTriggeringCondition::PeriodicFixedHalfHz;
    }
    if (value == "PeriodicFixed1Hz") {
        return McService::IntentTriggeringCondition::PeriodicFixed1Hz;
    }
    if (value == "PeriodicFixed2Hz") {
        return McService::IntentTriggeringCondition::PeriodicFixed2Hz;
    }
    if (value == "PeriodicFixed3Hz") {
        return McService::IntentTriggeringCondition::PeriodicFixed3Hz;
    }
    if (value == "PeriodicFixed5Hz") {
        return McService::IntentTriggeringCondition::PeriodicFixed5Hz;
    }
    return McService::IntentTriggeringCondition::SameAsCam;
}

McService::CoordinationTriggeringCondition parseCoordinationTriggeringCondition(const std::string& value)
{
    if (value == "PeriodicFixed") {
        return McService::CoordinationTriggeringCondition::PeriodicFixed;
    }
    if (value == "PeriodicFixedNoDCC") {
        return McService::CoordinationTriggeringCondition::PeriodicFixedNoDcc;
    }
    return McService::CoordinationTriggeringCondition::SameAsCam;
}

McService::McmTimeSource parseMcmTimeSource(const std::string& value)
{
    if (value == "CurrentTime") {
        return McService::McmTimeSource::CurrentTime;
    }
    return McService::McmTimeSource::DataProvider;
}

const char* intentTriggeringConditionName(McService::IntentTriggeringCondition condition)
{
    switch (condition) {
        case McService::IntentTriggeringCondition::SameAsCam: return "SameAsCAM";
        case McService::IntentTriggeringCondition::SameAsCamReduced2Hz: return "SameAsCAMreduced2Hz";
        case McService::IntentTriggeringCondition::SameAsCamReduced1Hz: return "SameAsCAMreduced1Hz";
        case McService::IntentTriggeringCondition::PeriodicFixed: return "PeriodicFixed";
        case McService::IntentTriggeringCondition::PeriodicFixedHalfHz: return "PeriodicFixed0.5Hz";
        case McService::IntentTriggeringCondition::PeriodicFixed1Hz: return "PeriodicFixed1Hz";
        case McService::IntentTriggeringCondition::PeriodicFixed2Hz: return "PeriodicFixed2Hz";
        case McService::IntentTriggeringCondition::PeriodicFixed3Hz: return "PeriodicFixed3Hz";
        case McService::IntentTriggeringCondition::PeriodicFixed5Hz: return "PeriodicFixed5Hz";
    }
    return "SameAsCAM";
}

const char* coordinationTriggeringConditionName(McService::CoordinationTriggeringCondition condition)
{
    switch (condition) {
        case McService::CoordinationTriggeringCondition::SameAsCam: return "SameAsCAM";
        case McService::CoordinationTriggeringCondition::PeriodicFixed: return "PeriodicFixed";
        case McService::CoordinationTriggeringCondition::PeriodicFixedNoDcc: return "PeriodicFixedNoDCC";
    }
    return "SameAsCAM";
}

const char* mcmTimeSourceName(McService::McmTimeSource source)
{
    switch (source) {
        case McService::McmTimeSource::DataProvider: return "DataProvider";
        case McService::McmTimeSource::CurrentTime: return "CurrentTime";
    }
    return "DataProvider";
}

const char* frequencyReduceStateName(McService::McmFrequencyReduceState state)
{
    switch (state) {
        case McService::McmFrequencyReduceState::None: return "None";
        case McService::McmFrequencyReduceState::Intent: return "Intent";
        case McService::McmFrequencyReduceState::Negotiation: return "Negotiation";
    }
    return "None";
}

const char* mcmSubtypeName(mcm::mcmSubtype subtype)
{
    switch (subtype) {
        case mcm::mcmSubtype::Request:
            return "Request";
        case mcm::mcmSubtype::Accept:
            return "Accept";
        case mcm::mcmSubtype::Reject:
            return "Reject";
        case mcm::mcmSubtype::Offer:
            return "Offer";
        case mcm::mcmSubtype::Confirm:
            return "Confirm";
        case mcm::mcmSubtype::Execute:
            return "Execute";
        case mcm::mcmSubtype::Cancel:
            return "Cancel";
        case mcm::mcmSubtype::Abort:
            return "Abort";
        default:
            return "Negotiation";
    }
}

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

enum class McmMeasurementCategory {
    Intent,
    Negotiation,
    Execution,
    EmergencyExecution
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

McmMeasurementCategory classifyMcmForMeasurement(const MCM_t& message)
{
    const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);
    if (metadata.hasExecutionContainer) {
        if (metadata.priorityManeuver == PriorityManeuver_emergency) {
            return McmMeasurementCategory::EmergencyExecution;
        }
        return McmMeasurementCategory::Execution;
    }
    if (metadata.hasNegotiationContainer) {
        return McmMeasurementCategory::Negotiation;
    }
    return McmMeasurementCategory::Intent;
}

bool isCooperativeAoiMessageForStation(const McmOperationMetadata& metadata, uint32_t egoStationId)
{
    if (metadata.hasNegotiationContainer) {
        return metadata.negotiationVehicleId1 == egoStationId ||
            (metadata.hasNegotiationVehicleId2 && metadata.negotiationVehicleId2 == egoStationId);
    }

    if (metadata.hasExecutionContainer) {
        return metadata.cooperationVehicleId1 == egoStationId ||
            (metadata.hasCooperationVehicleId2 && metadata.cooperationVehicleId2 == egoStationId);
    }

    return false;
}

double operatingModeMeasurementValue(mcm::operationMode mode)
{
    switch (mode) {
        case mcm::operationMode::IntentionSharingMode: return 1.0;
        case mcm::operationMode::ManeuverNegotiationMode: return 2.0;
        case mcm::operationMode::ManeuverExecutionMode: return 3.0;
        case mcm::operationMode::Unknown:
        default: return 0.0;
    }
}

long messageRequestKey(const McmOperationMetadata& metadata)
{
    if (metadata.hasExecutionContainer) {
        return metadata.cooperationId;
    }
    if (metadata.hasNegotiationContainer) {
        return metadata.requestId;
    }
    return -1;
}

mcm::TrajectoryPlanner::Trajectory extractTrajectory(const TrajectoryMCM_t& trajectory)
{
    mcm::TrajectoryPlanner::Trajectory points;
    points.reserve(static_cast<std::size_t>(trajectory.list.count));

    for (int i = 0; i < trajectory.list.count; ++i) {
        const TrajectoryPointMCM_t* point = trajectory.list.array[i];
        if (!point) {
            continue;
        }

        mcm::TrajPointMCM parsed;
        parsed.mX = point->deltaLongitudinalPosition;
        parsed.mY = point->deltaLateralPosition;
        parsed.mHeading = point->deltaHeading;
        parsed.mTime = omnetpp::SimTime { static_cast<double>(point->deltaTime) / 100.0 };
        points.push_back(parsed);
    }

    return points;
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
    snapshot.hasLaneId = intent.laneID != nullptr;
    if (intent.laneID) {
        snapshot.laneId = *intent.laneID;
    }
    snapshot.plannedTrajectoryPointCount = static_cast<std::size_t>(intent.plannedTrajectory.list.count);
    snapshot.plannedTrajectory = extractTrajectory(intent.plannedTrajectory);
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
    if (message.mcm.mcmParameters.maneuverNegotiationContainer) {
        snapshot.requestedTrajectory = extractTrajectory(
            message.mcm.mcmParameters.maneuverNegotiationContainer->requestedTrajectory);
    }
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
    auto snapshot = extractMcmSnapshot(message);
    EV_DETAIL << "McService received MCM snapshot for station " << snapshot.stationId
        << " with plannedTrajectory points=" << snapshot.plannedTrajectoryPointCount << '\n';
    return { snapshot, receivedAt };
}

void appendZeroTrajectoryPoint(TrajectoryMCM_t& trajectory)
{
    TrajectoryPointMCM_t* point = vanetza::asn1::allocate<TrajectoryPointMCM_t>();
    point->deltaLongitudinalPosition = 0;
    point->deltaLateralPosition = 0;
    point->deltaHeading = 0;
    point->deltaTime = 0;
    if (ASN_SEQUENCE_ADD(&trajectory, point) != 0) {
        throw cRuntimeError("Failed to append MCM trajectory point");
    }
}

long normalizeTrajectoryHeading(long heading)
{
    while (heading > 1800) {
        heading -= 3600;
    }
    while (heading < -1800) {
        heading += 3600;
    }
    return heading;
}

void appendTrajectoryPoint(TrajectoryMCM_t& trajectory, const mcm::TrajPointMCM& point, long heading)
{
    TrajectoryPointMCM_t* trajectoryPoint = vanetza::asn1::allocate<TrajectoryPointMCM_t>();
    trajectoryPoint->deltaLongitudinalPosition = static_cast<long>(std::round(point.mX));
    trajectoryPoint->deltaLateralPosition = static_cast<long>(std::round(point.mY));
    trajectoryPoint->deltaHeading = normalizeTrajectoryHeading(heading);
    trajectoryPoint->deltaTime = static_cast<long>(std::round(point.mTime.dbl() * 100.0));
    if (ASN_SEQUENCE_ADD(&trajectory, trajectoryPoint) != 0) {
        throw cRuntimeError("Failed to append MCM trajectory point");
    }
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

void McService::loadCommunicationConfig()
{
    mCommunicationConfig.intentTrigger =
        parseIntentTriggeringCondition(par("triggeringCondition").stdstringValue());
    mCommunicationConfig.coordinationTrigger =
        parseCoordinationTriggeringCondition(par("triggeringConditionCoordination").stdstringValue());
    mCommunicationConfig.withDccRestriction = par("withDccRestriction").boolValue();
    mCommunicationConfig.minInterval = par("minInterval");
    mCommunicationConfig.maxInterval = par("maxInterval");
    mCommunicationConfig.fixedRate = par("fixedRate").boolValue();
    mCommunicationConfig.fixedRateInterval = par("fixedRateInterval");
    mCommunicationConfig.fixedInterval = par("fixedInterval");
    mCommunicationConfig.maxIntervalNegMcm = par("maxIntervalNegMcm");
    mCommunicationConfig.negotiationRetryInterval = par("negotiationRetryInterval");
    mCommunicationConfig.negotiationLimitMerging = par("negotiationLimitMerging");
    mCommunicationConfig.negotiationLimitLaneChange = par("negotiationLimitLaneChange");
    mCommunicationConfig.dccProfiles = par("dccProfiles").boolValue();
    mCommunicationConfig.dccOnlyMcm = par("dccOnlyMcm").boolValue();
    mCommunicationConfig.timeSource = parseMcmTimeSource(par("timeGenMcm").stdstringValue());
    mCommunicationConfig.newGenMcmRules = par("newGenMcmRules").boolValue();
    mCommunicationConfig.newGenMcmRulesIntent = par("newGenMcmRulesIntent").boolValue();
    mCommunicationConfig.newGenMcmRulesIntent1HzMco = par("newGenMcmRulesIntent1Hz_MCO").boolValue();
    mCommunicationConfig.freqReduceCbrMin = par("freqReduceCBRmin").doubleValue();
    mCommunicationConfig.freqReduceCbrMedium = par("freqReduceCBRmedium").doubleValue();
    mCommunicationConfig.freqReduceCbrMax = par("freqReduceCBRmax").doubleValue();
    mCommunicationConfig.freqReduceCbrMco = par("freqReduceCBRmco").doubleValue();
}

void McService::logCommunicationConfig() const
{
    EV_INFO << "[MCM-QOS-CONFIG]"
        << " triggeringCondition=" << intentTriggeringConditionName(mCommunicationConfig.intentTrigger)
        << " triggeringConditionCoordination=" << coordinationTriggeringConditionName(mCommunicationConfig.coordinationTrigger)
        << " withDccRestriction=" << mCommunicationConfig.withDccRestriction
        << " minInterval=" << mCommunicationConfig.minInterval
        << " maxInterval=" << mCommunicationConfig.maxInterval
        << " fixedRate=" << mCommunicationConfig.fixedRate
        << " fixedRateInterval=" << mCommunicationConfig.fixedRateInterval
        << " fixedInterval=" << mCommunicationConfig.fixedInterval
        << " maxIntervalNegMcm=" << mCommunicationConfig.maxIntervalNegMcm
        << " dccProfiles=" << mCommunicationConfig.dccProfiles
        << " dccOnlyMcm=" << mCommunicationConfig.dccOnlyMcm
        << " timeGenMcm=" << mcmTimeSourceName(mCommunicationConfig.timeSource)
        << " newGenMcmRules=" << mCommunicationConfig.newGenMcmRules
        << " newGenMcmRulesIntent=" << mCommunicationConfig.newGenMcmRulesIntent
        << " newGenMcmRulesIntent1Hz_MCO=" << mCommunicationConfig.newGenMcmRulesIntent1HzMco
        << " freqReduceCBRmin=" << mCommunicationConfig.freqReduceCbrMin
        << " freqReduceCBRmedium=" << mCommunicationConfig.freqReduceCbrMedium
        << " freqReduceCBRmax=" << mCommunicationConfig.freqReduceCbrMax
        << " freqReduceCBRmco=" << mCommunicationConfig.freqReduceCbrMco
        << " note=configuration-hooks-active-behavior-staged\n";
}

void McService::emitCurrentOperatingModeIfChanged()
{
    if (!mApplication) {
        return;
    }

    const mcm::operationMode mode = mApplication->currentOperationMode();
    if (mHasLastEmittedOperatingMode && mode == mLastEmittedOperatingMode) {
        return;
    }

    mHasLastEmittedOperatingMode = true;
    mLastEmittedOperatingMode = mode;
    emit(scSignalCurrentMcsOperatingMode, operatingModeMeasurementValue(mode));
}

void McService::emitSentMeasurements(
    const MCM_t& message,
    mcm::operationMode modeBeforeHandleSent)
{
    emit(scSignalMcmSentCounter, 1L);
    emit(scSignalCoopCbr, mLocalCbr);

    const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);
    switch (classifyMcmForMeasurement(message)) {
        case McmMeasurementCategory::Intent:
            emit(scSignalMcmIntentionSentCounter, 1L);
            break;
        case McmMeasurementCategory::Negotiation:
            emit(scSignalMcmNegotiationSentCounter, 1L);
            break;
        case McmMeasurementCategory::Execution:
            emit(scSignalMcmExecutionSentCounter, 1L);
            break;
        case McmMeasurementCategory::EmergencyExecution:
            emit(scSignalMcmExecutionSentCounter, 1L);
            emit(scSignalMcmExecutionEmergencySentCounter, 1L);
            break;
    }

    const long key = messageRequestKey(metadata);
    if (metadata.hasNegotiationContainer &&
            metadata.mcmCategory == McmCategory_request &&
            key >= 0 &&
            mMeasuredNegotiationStartedRequestIds.insert(key).second) {
        emit(scSignalNegotiationStartedCounter, 1L);
        mNegotiationStartedAtByRequestId.emplace(key, simTime());
        // Current MCM commands expose numberOfVehicles, not a separate conflict
        // count. Treat one/two-CV negotiations conservatively as the
        // "two vehicles" bucket and reserve "three vehicles" for larger sets.
        if (metadata.numberOfVehicles > 2) {
            emit(scSignalCounterNegotiationThreeVehicles, 1L);
        } else {
            emit(scSignalCounterNegotiationTwoVehicles, 1L);
        }
    }

    if (metadata.hasNegotiationContainer &&
            metadata.mcmCategory == McmCategory_reject &&
            key >= 0 &&
            mMeasuredRejectedRequestIds.insert(key).second) {
        emit(scSignalCounterNegotiationRejected, 1L);
    }

    if (metadata.hasExecutionContainer &&
            metadata.priorityManeuver != PriorityManeuver_emergency &&
            key >= 0 &&
            mMeasuredExecutionStartedRequestIds.insert(key).second) {
        emit(scSignalExecutionStartedCounter, 1L);
    }

    if (metadata.hasNegotiationContainer &&
            metadata.mcmCategory == McmCategory_cancel &&
            modeBeforeHandleSent == mcm::operationMode::ManeuverExecutionMode &&
            key >= 0 &&
            mMeasuredExecutionCompletedRequestIds.insert(key).second) {
        emit(scSignalExecutionCompletedCounter, 1L);
    }
}

void McService::emitPlannerMeasurements(const std::vector<mcm::PlannerMeasurement>& measurements)
{
    for (const auto& measurement : measurements) {
        switch (measurement.metric) {
            case mcm::PlannerMeasurementMetric::TrajectoryCost:
                emit(scSignalTrajectoryCost, measurement.value);
                break;
            case mcm::PlannerMeasurementMetric::TrajectoryCostRV:
                emit(scSignalTrajectoryCostRV, measurement.value);
                break;
            case mcm::PlannerMeasurementMetric::SecondRequestStartedCounter:
                emit(scSignalSecondRequestStartedCounter, 1L);
                break;
            case mcm::PlannerMeasurementMetric::SecondRequestCompletedCounter:
                emit(scSignalSecondRequestCompletedCounter, 1L);
                break;
            case mcm::PlannerMeasurementMetric::SecondRequestRejectedCounter:
                emit(scSignalSecondRequestRejectedCounter, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterCoordPossiblePriorityLow:
                emit(scSignalCounterCoordPossiblePriorityLow, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterCoordPossiblePriorityMedium:
                emit(scSignalCounterCoordPossiblePriorityMedium, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterCoordPossiblePriorityHigh:
                emit(scSignalCounterCoordPossiblePriorityHigh, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType0:
                emit(scSignalCounterTrajectoryType0, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType1:
                emit(scSignalCounterTrajectoryType1, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType2:
                emit(scSignalCounterTrajectoryType2, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType4:
                emit(scSignalCounterTrajectoryType4, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType5:
                emit(scSignalCounterTrajectoryType5, 1L);
                break;
            case mcm::PlannerMeasurementMetric::CounterTrajectoryType6:
                emit(scSignalCounterTrajectoryType6, 1L);
                break;
        }
    }
}

void McService::emitReceivedMeasurements(
    const MCM_t& message,
    const vanetza::asn1::Mcm& decoded,
    const SimTime& now)
{
    emit(scSignalMcmReceivedCounter, 1L);
    emit(scSignalMessageSizeMcmReceived, static_cast<unsigned long>(decoded.size()));

    const McmOperationMetadata metadata = getMcmOperationMetadata(message.mcm.mcmParameters);
    switch (classifyMcmForMeasurement(message)) {
        case McmMeasurementCategory::Intent:
            emit(scSignalMcmIntentionReceivedCounter, 1L);
            break;
        case McmMeasurementCategory::Negotiation:
            emit(scSignalMcmNegotiationReceivedCounter, 1L);
            break;
        case McmMeasurementCategory::Execution:
            emit(scSignalMcmExecutionReceivedCounter, 1L);
            break;
        case McmMeasurementCategory::EmergencyExecution:
            emit(scSignalMcmExecutionReceivedCounter, 1L);
            emit(scSignalMcmExecutionEmergencyReceivedCounter, 1L);
            break;
    }

    const SimTime generatedAt = mTimer->getTimeFor(
        mTimer->reconstructMilliseconds(message.mcm.generationDeltaTime));
    const SimTime eteDelay = now - generatedAt;
    if (eteDelay < SIMTIME_ZERO) {
        // TODO: Revisit generationDeltaTime reconstruction for cross-second
        // wraparound edge cases before emitting negative E2E delay samples.
        return;
    }

    emit(scSignalEteDelayMcm, eteDelay);
    if (metadata.hasNegotiationContainer) {
        emit(scSignalEteDelayMcmNegotiation, eteDelay);
    }
    if (metadata.hasExecutionContainer) {
        emit(scSignalEteDelayMcmExecution, eteDelay);
        if (metadata.priorityManeuver == PriorityManeuver_emergency) {
            emit(scSignalEteDelayMcmEmergency, eteDelay);
        }
    }
    if (isCooperativeAoiMessageForStation(metadata, mVehicleDataProvider->station_id())) {
        emit(scSignalCoopVehicleAgeOfInformation, eteDelay);
    }
}

void McService::initialize()
{
    ItsG5BaseService::initialize();

    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(scExperimentalMcmAid);
    mVehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
    if (auto* host = findHost()) {
        cModule* radioDriver = host->getSubmodule("radioDriver");
        if (!radioDriver) {
            radioDriver = host->getSubmodule("radioDriver", 0);
        }
        if (radioDriver) {
            radioDriver->subscribe(RadioDriverBase::ChannelLoadSignal, this);
            EV_INFO << "[MCM-QOS-CONFIG] subscribed channel-load signal from "
                << radioDriver->getFullPath() << '\n';
        } else {
            EV_WARN << "[MCM-QOS-CONFIG] radioDriver module not found; local CBR will remain at default 0\n";
        }

        if (auto* environmentModelModule = host->getSubmodule("environmentModel")) {
            mLocalEnvironmentModel = static_cast<const LocalEnvironmentModel*>(environmentModelModule);
        }
    }

    loadCommunicationConfig();
    logCommunicationConfig();

    mLastMcmTimestamp = simTime();
    mGenMcmMin = mCommunicationConfig.minInterval;
    mGenMcmMax = mCommunicationConfig.maxInterval;
    mGenMcm = mGenMcmMax;
    mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
    mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
    mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;
    mDccRestriction = mCommunicationConfig.withDccRestriction;
    mFixedRate = mCommunicationConfig.fixedRate;
    mFixedRateInterval = mCommunicationConfig.fixedRateInterval;
    mNegotiationRetryInterval = mCommunicationConfig.negotiationRetryInterval;
    mNegotiationLimitMerging = mCommunicationConfig.negotiationLimitMerging;
    mNegotiationLimitLaneChange = mCommunicationConfig.negotiationLimitLaneChange;
    mEffectiveIntentTrigger = mCommunicationConfig.intentTrigger;
    mSendNegotiationTestMcm = par("sendNegotiationTestMcm").boolValue();
    mForceFirstCvRejectForSecondRequestSmoke = par("forceFirstCvRejectForSecondRequestSmoke").boolValue();
    const long forceRejectStationId = par("forceFirstCvRejectStationId").intValue();
    mForceFirstCvRejectStationId = forceRejectStationId > 0 ?
        static_cast<uint32_t>(forceRejectStationId) : 0;
    mUsePrerecordedIntentTrajectory = par("usePrerecordedIntentTrajectory").boolValue();
    mPrerecordedTrajectoryCsv = par("prerecordedTrajectoryCsv").stdstringValue();
    mPrerecordedTrajectorySteps = par("prerecordedTrajectorySteps").intValue();
    mPrerecordedTrajectoryDt = par("prerecordedTrajectoryDt").doubleValue();
    mPrerecordedTrajectorySteps = std::max(1, std::min(mPrerecordedTrajectorySteps, scMaxTrajectoryMcmPoints));
    mTrajectoryPlanner.initialize(mVehicleController, mVehicleDataProvider, mLocalEnvironmentModel);

    if (mUsePrerecordedIntentTrajectory) {
        if (!mVehicleController) {
            EV_WARN << "McService prerecorded plannedTrajectory enabled but VehicleController is unavailable; zero placeholder fallback will be used\n";
        } else if (mPrerecordedTrajectoryCsv.empty()) {
            EV_WARN << "McService prerecorded plannedTrajectory enabled but prerecordedTrajectoryCsv is empty; zero placeholder fallback will be used\n";
        } else {
            const std::string routeId = mVehicleController->getRouteID();
            const auto columns = mcm::selectRouteTrajectoryColumns(mPrerecordedTrajectoryCsv, ',', routeId);
            EV_INFO << "McService prerecorded plannedTrajectory route ID: " << routeId << '\n';
            EV_INFO << "McService loading prerecorded trajectory CSV: " << mPrerecordedTrajectoryCsv << '\n';
            if (columns.usingFallback) {
                EV_WARN << "McService route columns " << routeId << "_x/" << routeId
                    << "_y missing; using dummy/filler columns others_x/others_y for Intent Sharing only\n";
                mPrerecordedTrajectoryFallbackUsed = true;
            }
            EV_INFO << "McService selected prerecorded trajectory columns: "
                << columns.xColumn << ", " << columns.yColumn << '\n';

            mPrerecordedCx = mcm::readCX(mPrerecordedTrajectoryCsv, ',', columns.xColumn);
            mPrerecordedCy = mcm::readCX(mPrerecordedTrajectoryCsv, ',', columns.yColumn);
            EV_INFO << "McService loaded prerecorded trajectory CSV with "
                << std::min(mPrerecordedCx.size(), mPrerecordedCy.size())
                << " usable coordinate rows for station " << mVehicleDataProvider->station_id() << '\n';
        }
    }
    mApplication.reset(new mcm::McApplication());
    mApplication->initialize(mVehicleController, mVehicleDataProvider, mLocalEnvironmentModel);
    mApplication->setNegotiationRetryInterval(mNegotiationRetryInterval);
    mApplication->setNegotiationLimits(
        mNegotiationLimitMerging,
        mNegotiationLimitLaneChange);
    mApplication->setSecondRequestSmokeReject(
        mForceFirstCvRejectForSecondRequestSmoke,
        mForceFirstCvRejectStationId);
}

void McService::receiveSignal(cComponent*, simsignal_t signal, double value, cObject*)
{
    if (signal != RadioDriverBase::ChannelLoadSignal) {
        return;
    }

    ASSERT(value >= 0.0 && value <= 1.0);
    mLocalCbr = std::max(0.0, std::min(1.0, value));
}

void McService::trigger()
{
    Enter_Method("trigger");
    updateApplicationEgoContext(simTime());
    mApplication->tick(simTime());
    emitPlannerMeasurements(mApplication->consumePlannerMeasurements());
    emitCurrentOperatingModeIfChanged();
    checkTriggeringConditions(simTime());
}

void McService::checkTriggeringConditions(const SimTime& T_now)
{
    const SimTime dccInterval = mDccRestriction ? genMcmDcc() : mGenMcmMin;
    if (shouldGenerateCoordinationMcm(T_now, dccInterval)) {
        sendMcm(T_now);
        return;
    }

    if (shouldGenerateIntentMcm(T_now, dccInterval)) {
        sendMcm(T_now);
    }
}

bool McService::shouldGenerateIntentMcm(const SimTime& now, SimTime dccInterval)
{
    updateAdaptiveIntentFrequency(now);

    const SimTime elapsed = now - mLastMcmTimestamp;
    const SimTime fixedInterval = mFixedRateInterval > SIMTIME_ZERO ? mFixedRateInterval : mGenMcmMin;
    const IntentTriggeringCondition trigger = mEffectiveIntentTrigger;

    if (elapsed < dccInterval) {
        return false;
    }

    if (trigger == IntentTriggeringCondition::PeriodicFixed ||
            trigger == IntentTriggeringCondition::PeriodicFixedHalfHz ||
            trigger == IntentTriggeringCondition::PeriodicFixed1Hz ||
            trigger == IntentTriggeringCondition::PeriodicFixed2Hz ||
            trigger == IntentTriggeringCondition::PeriodicFixed3Hz ||
            trigger == IntentTriggeringCondition::PeriodicFixed5Hz) {
        return elapsed >= intervalForIntentTriggeringCondition(trigger);
    }

    if (trigger == IntentTriggeringCondition::SameAsCam && mFixedRate) {
        return elapsed >= fixedInterval;
    }

    if (!mHasLastMcmKinematics) {
        return true;
    }

    const SimTime reducedInterval = intervalForIntentTriggeringCondition(trigger);
    if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
        if ((trigger == IntentTriggeringCondition::SameAsCamReduced1Hz ||
                trigger == IntentTriggeringCondition::SameAsCamReduced2Hz) &&
                elapsed < reducedInterval) {
            return false;
        }
        mGenMcm = std::min(elapsed, mGenMcmMax);
        mGenMcmLowDynamicsCounter = 0;
        return true;
    }

    if (elapsed >= mGenMcm) {
        if (++mGenMcmLowDynamicsCounter >= mGenMcmLowDynamicsLimit) {
            mGenMcm = mGenMcmMax;
        }
        return true;
    }

    return false;
}

bool McService::adaptiveIntentRulesEnabled() const
{
    return mCommunicationConfig.newGenMcmRules &&
        mCommunicationConfig.newGenMcmRulesIntent &&
        !mCommunicationConfig.newGenMcmRulesIntent1HzMco;
}

void McService::updateAdaptiveIntentFrequency(const SimTime& now)
{
    // The MCO 1 Hz rule is intentionally enabled by its own flag. It takes
    // precedence over the general newGenMcmRules/newGenMcmRulesIntent rule so
    // both rules cannot update adaptive state in the same tick.
    if (mCommunicationConfig.newGenMcmRulesIntent1HzMco) {
        // TODO: Exclude Important Intent Sharing once an explicit
        // important-intent state is available.
        const double threshold = mCommunicationConfig.freqReduceCbrMco;

        if (mFrequencyReduceState == McmFrequencyReduceState::None &&
                mEffectiveIntentTrigger == IntentTriggeringCondition::SameAsCam &&
                mLocalCbr > threshold) {
            mEffectiveIntentTrigger = IntentTriggeringCondition::PeriodicFixed1Hz;
            mFrequencyReduceState = McmFrequencyReduceState::Intent;
            mFrequencyReducedIntentAt = now;
            mFrequencyReduceIntentRecoveryDelay = SimTime {
                mCommunicationConfig.dccOnlyMcm ? uniform(0.1, 1.0) : uniform(1.0, 2.0)
            };
            EV_INFO << "[MCM-QOS-ADAPT] MCO intent reduction: SameAsCAM -> PeriodicFixed1Hz"
                << " localCbr=" << mLocalCbr
                << " threshold=" << threshold
                << " recoveryDelay=" << mFrequencyReduceIntentRecoveryDelay
                << '\n';
            return;
        }

        if (mFrequencyReduceState == McmFrequencyReduceState::Intent &&
                mEffectiveIntentTrigger == IntentTriggeringCondition::PeriodicFixed1Hz &&
                mLocalCbr < threshold &&
                now - mFrequencyReducedIntentAt >= mFrequencyReduceIntentRecoveryDelay) {
            mEffectiveIntentTrigger = IntentTriggeringCondition::SameAsCam;
            mFrequencyReduceState = McmFrequencyReduceState::None;
            mFrequencyReducedIntentAt = SIMTIME_ZERO;
            mFrequencyReduceIntentRecoveryDelay = SIMTIME_ZERO;
            EV_INFO << "[MCM-QOS-ADAPT] MCO intent recovery: PeriodicFixed1Hz -> SameAsCAM"
                << " localCbr=" << mLocalCbr
                << " threshold=" << threshold
                << '\n';
        }

        return;
    }

    if (!adaptiveIntentRulesEnabled()) {
        return;
    }

    // TODO: Exempt Important Intent Sharing once McService exposes an explicit
    // important-intent state. Until then, this rule is limited to normal Intent
    // MCM generation and does not touch negotiation/execution containers.
    const double enterThreshold = mCommunicationConfig.dccOnlyMcm ?
        mCommunicationConfig.freqReduceCbrMax :
        mCommunicationConfig.freqReduceCbrMin;
    const double recoverThreshold = enterThreshold;

    if (mFrequencyReduceState == McmFrequencyReduceState::None &&
            mEffectiveIntentTrigger == IntentTriggeringCondition::SameAsCam &&
            mLocalCbr > enterThreshold) {
        mEffectiveIntentTrigger = IntentTriggeringCondition::PeriodicFixed1Hz;
        mFrequencyReduceState = McmFrequencyReduceState::Intent;
        mFrequencyReducedIntentAt = now;
        mFrequencyReduceIntentRecoveryDelay = SimTime {
            mCommunicationConfig.dccOnlyMcm ? uniform(0.5, 1.5) : uniform(1.0, 2.0)
        };
        EV_INFO << "[MCM-QOS-ADAPT] reducing Intent MCM generation to "
            << intentTriggeringConditionName(mEffectiveIntentTrigger)
            << " localCbr=" << mLocalCbr
            << " threshold=" << enterThreshold
            << " recoveryDelay=" << mFrequencyReduceIntentRecoveryDelay
            << " state=" << frequencyReduceStateName(mFrequencyReduceState)
            << '\n';
        return;
    }

    if (mFrequencyReduceState == McmFrequencyReduceState::Intent &&
            mEffectiveIntentTrigger == IntentTriggeringCondition::PeriodicFixed1Hz &&
            mLocalCbr < recoverThreshold &&
            now - mFrequencyReducedIntentAt > mFrequencyReduceIntentRecoveryDelay) {
        mEffectiveIntentTrigger = mCommunicationConfig.intentTrigger;
        mFrequencyReduceState = McmFrequencyReduceState::None;
        mFrequencyReducedIntentAt = SIMTIME_ZERO;
        mFrequencyReduceIntentRecoveryDelay = SIMTIME_ZERO;
        EV_INFO << "[MCM-QOS-ADAPT] restoring Intent MCM generation to "
            << intentTriggeringConditionName(mEffectiveIntentTrigger)
            << " localCbr=" << mLocalCbr
            << " threshold=" << recoverThreshold
            << " state=" << frequencyReduceStateName(mFrequencyReduceState)
            << '\n';
    }
}

bool McService::shouldGenerateCoordinationMcm(const SimTime& now, SimTime dccInterval)
{
    if (!hasPendingCoordinationMcm()) {
        return false;
    }

    const SimTime elapsed = now - mLastMcmTimestamp;
    const auto trigger = mCommunicationConfig.coordinationTrigger;
    const SimTime dccGate = trigger == CoordinationTriggeringCondition::PeriodicFixedNoDcc ?
        mGenMcmMin :
        dccInterval;

    if (elapsed < dccGate) {
        return false;
    }

    if (trigger == CoordinationTriggeringCondition::PeriodicFixed ||
            trigger == CoordinationTriggeringCondition::PeriodicFixedNoDcc) {
        return elapsed >= intervalForCoordinationTriggeringCondition(trigger);
    }

    if (mFixedRate) {
        const SimTime fixedInterval = mFixedRateInterval > SIMTIME_ZERO ? mFixedRateInterval : mGenMcmMin;
        return elapsed >= fixedInterval;
    }

    if (!mHasLastMcmKinematics) {
        return true;
    }

    if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
        mGenMcm = std::min(elapsed, mGenMcmMax);
        mGenMcmLowDynamicsCounter = 0;
        return true;
    }

    if (elapsed >= mGenMcm) {
        if (++mGenMcmLowDynamicsCounter >= mGenMcmLowDynamicsLimit) {
            mGenMcm = mGenMcmMax;
        }
        return true;
    }

    return false;
}

bool McService::hasPendingCoordinationMcm() const
{
    return mApplication && mApplication->hasPendingCoordinationCommand();
}

SimTime McService::intervalForIntentTriggeringCondition(IntentTriggeringCondition condition) const
{
    switch (condition) {
        case IntentTriggeringCondition::PeriodicFixedHalfHz:
            return SimTime { 2.0 };
        case IntentTriggeringCondition::SameAsCamReduced1Hz:
        case IntentTriggeringCondition::PeriodicFixed1Hz:
            return SimTime { 1.0 };
        case IntentTriggeringCondition::SameAsCamReduced2Hz:
        case IntentTriggeringCondition::PeriodicFixed2Hz:
            return SimTime { 0.5 };
        case IntentTriggeringCondition::PeriodicFixed3Hz:
            return SimTime { 0.33 };
        case IntentTriggeringCondition::PeriodicFixed5Hz:
            return SimTime { 0.2 };
        case IntentTriggeringCondition::SameAsCam:
        case IntentTriggeringCondition::PeriodicFixed:
        default:
            return mGenMcmMin;
    }
}

SimTime McService::intervalForCoordinationTriggeringCondition(CoordinationTriggeringCondition condition) const
{
    switch (condition) {
        case CoordinationTriggeringCondition::PeriodicFixed:
        case CoordinationTriggeringCondition::PeriodicFixedNoDcc:
            return mCommunicationConfig.maxIntervalNegMcm > SIMTIME_ZERO ?
                mCommunicationConfig.maxIntervalNegMcm :
                mGenMcmMin;
        case CoordinationTriggeringCondition::SameAsCam:
        default:
            return mGenMcmMin;
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

vanetza::dcc::Profile McService::selectMcmDccProfile(
    mcm::operationMode mode,
    mcm::priorityMcmCategory priority) const
{
    using vanetza::dcc::Profile;

    // Keep current behavior unless differentiated DCC profile mapping
    // is explicitly enabled. The current implementation has used DP2 for all
    // MCMs, so dccProfiles=false must continue returning DP2.
    if (!mCommunicationConfig.dccProfiles) {
        return Profile::DP2;
    }

    // MCM negotiation uses a higher DCC priority than execution
    // because Request/Offer/Confirm/Accept messages unblock cooperative
    // maneuver decisions, while execution messages can tolerate slightly lower
    // priority except for emergency execution.
    if (mCommunicationConfig.dccOnlyMcm) {
        if (mode == mcm::operationMode::ManeuverNegotiationMode) {
            if (priority == mcm::priorityMcmCategory::HighPriority) {
                return Profile::DP0;
            }
            if (priority == mcm::priorityMcmCategory::MediumPriority) {
                return Profile::DP1;
            }
            if (priority == mcm::priorityMcmCategory::LowPriority) {
                return Profile::DP2;
            }
        }

        if (mode == mcm::operationMode::ManeuverExecutionMode) {
            if (priority == mcm::priorityMcmCategory::EmergencyPriority) {
                return Profile::DP0;
            }
            if (priority == mcm::priorityMcmCategory::HighPriority) {
                return Profile::DP1;
            }
            if (priority == mcm::priorityMcmCategory::MediumPriority) {
                return Profile::DP2;
            }
            if (priority == mcm::priorityMcmCategory::LowPriority) {
                return Profile::DP3;
            }
        }

        return Profile::DP3;
    }

    if (mode == mcm::operationMode::ManeuverNegotiationMode) {
        if (priority == mcm::priorityMcmCategory::HighPriority) {
            return Profile::DP0;
        }
        if (priority == mcm::priorityMcmCategory::MediumPriority ||
                priority == mcm::priorityMcmCategory::LowPriority) {
            return Profile::DP1;
        }
    }

    if (mode == mcm::operationMode::ManeuverExecutionMode) {
        if (priority == mcm::priorityMcmCategory::EmergencyPriority) {
            return Profile::DP0;
        }
        if (priority == mcm::priorityMcmCategory::HighPriority) {
            return Profile::DP1;
        }
        if (priority == mcm::priorityMcmCategory::MediumPriority ||
                priority == mcm::priorityMcmCategory::LowPriority) {
            return Profile::DP2;
        }
    }

    return Profile::DP2;
}

void McService::sendMcm(const SimTime& T_now)
{
    if (mApplication) {
        mApplication->prepareMcmGeneration(T_now);
    }

    auto command = mApplication->consumePendingCommand();
    mcm::operationMode generatedMode = mcm::operationMode::IntentionSharingMode;
    mcm::priorityMcmCategory generatedPriority = mcm::priorityMcmCategory::NoPriority;

    uint16_t generationDeltaTime = 0;
    if (command || mCommunicationConfig.timeSource == McmTimeSource::CurrentTime) {
        generationDeltaTime = countTaiMilliseconds(mTimer->getCurrentTime());
    } else {
        generationDeltaTime = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    }

    auto mcmMessage = createMinimalIntentionSharingMessage(*mVehicleDataProvider, generationDeltaTime);
    if (command && command->kind == mcm::PendingMcmCommand::Kind::Negotiation) {
        generatedMode = mcm::operationMode::ManeuverNegotiationMode;
        generatedPriority = command->priority;
        addManeuverNegotiationContainer(mcmMessage, *mVehicleDataProvider, *command);
        std::string error;
        if (!mcmMessage.validate(error)) {
            throw cRuntimeError("Invalid MCM %s command: %s", mcmSubtypeName(command->subtype), error.c_str());
        }
        EV_INFO << "McService validated route_merging_1 " << mcmSubtypeName(command->subtype)
            << " MCM: requestId="
            << static_cast<int>(command->requestId)
            << " numberOfVehicles=" << static_cast<int>(command->numberOfVehicles)
            << " target1=" << command->targetVehicle1
            << " target2=" << command->targetVehicle2
            << " requestedTrajectoryPoints=" << command->requestedTrajectory.size() << '\n';
    } else if (command && command->kind == mcm::PendingMcmCommand::Kind::Execution) {
        generatedMode = mcm::operationMode::ManeuverExecutionMode;
        generatedPriority = command->priority;
        addManeuverExecutionContainer(mcmMessage, *mVehicleDataProvider, *command);
        // The generated MCMextra CooperationID descriptor has no constraint
        // callback, so asn_check_constraints() dereferences null for execution
        // containers. Keep validation for minimal/negotiation MCMs, but skip it
        // here to preserve the execution-container emergency workaround.
        EV_INFO << "McService serialized " << mcmSubtypeName(command->subtype)
            << " execution MCM: cooperationId="
            << static_cast<int>(command->requestId)
            << " priority=" << static_cast<int>(command->priority)
            << " target1=" << command->targetVehicle1
            << " target2=" << command->targetVehicle2
            << '\n';
    } else if (mSendNegotiationTestMcm) {
        addManeuverNegotiationContainer(mcmMessage, *mVehicleDataProvider);
        std::string error;
        if (!mcmMessage.validate(error)) {
            throw cRuntimeError("Invalid minimal negotiation test MCM: %s", error.c_str());
        }
        EV_DETAIL << "Sending minimal negotiation test MCM for station "
            << (*mcmMessage).header.stationID << " at " << T_now << '\n';
    }

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::MCM;
    request.gn.its_aid = scExperimentalMcmAid;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    const auto dccProfile = selectMcmDccProfile(generatedMode, generatedPriority);
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dccProfile));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    McObject obj(std::move(mcmMessage));
    emit(scSignalMcmSent, &obj);
    const MCM_t& message = *obj.asn1();
    const mcm::operationMode modeBeforeHandleSent = mApplication->currentOperationMode();
    emitSentMeasurements(message, modeBeforeHandleSent);
    mApplication->handleSentMcm(makeSentMcm(message, T_now));
    emitCurrentOperatingModeIfChanged();
    EV_DETAIL << "Sending minimal MCM for station " << obj.asn1()->header.stationID << " at " << T_now << '\n';
    mLastMcmPosition = mVehicleDataProvider->position();
    mLastMcmSpeed = mVehicleDataProvider->speed();
    mLastMcmHeading = mVehicleDataProvider->heading();
    mHasLastMcmKinematics = true;
    mLastMcmTimestamp = T_now;

    using McmByteBuffer = convertible::byte_buffer_impl<asn1::Mcm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new McmByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}

void McService::updateApplicationEgoContext(const SimTime& now)
{
    if (!mApplication || !mVehicleDataProvider) {
        return;
    }

    mcm::McEgoContext context;
    context.now = now;
    context.stationId = mVehicleDataProvider->station_id();
    context.speed = mVehicleDataProvider->speed() / boost::units::si::meter_per_second;
    context.heading = mVehicleDataProvider->heading() / boost::units::si::radians;

    if (mVehicleController) {
        context.routeId = mVehicleController->getRouteID();
        const auto position = mVehicleController->getPositionSumo();
        context.x = position.x / boost::units::si::meter;
        context.y = position.y / boost::units::si::meter;
        try {
            context.laneIndex = mVehicleController->getTraCI()->vehicle.getLaneIndex(mVehicleController->getVehicleId());
        } catch (const std::exception& e) {
            EV_WARN << "McService ego context lane lookup failed: " << e.what() << '\n';
        }
    }

    if (mUsePrerecordedIntentTrajectory && mVehicleController &&
            !mPrerecordedCx.empty() && !mPrerecordedCy.empty()) {
        context.routeReferenceX = mPrerecordedCx;
        context.routeReferenceY = mPrerecordedCy;
        context.routeReferenceIndex = mPrerecordedTrajectoryIndex;

        context.plannedTrajectory = mTrajectoryPlanner.calculateRefTrajectory(
            mPrerecordedTrajectorySteps,
            mPrerecordedTrajectoryDt,
            mPrerecordedCx,
            mPrerecordedCy,
            mPrerecordedTrajectoryIndex,
            true,
            0.0F,
            0.0F,
            0.0F);
    }

    mApplication->updateEgoContext(context);
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

    if (!mUsePrerecordedIntentTrajectory) {
        appendZeroTrajectoryPoint(intent.plannedTrajectory);
    } else if (!appendPrerecordedIntentTrajectory(message)) {
        EV_WARN << "McService using zero placeholder plannedTrajectory for station "
            << vdp.station_id() << '\n';
        appendZeroTrajectoryPoint(intent.plannedTrajectory);
    }

    if (mVehicleController) {
        try {
            const std::string vehicleId = mVehicleController->getVehicleId();
            const long laneIndex = mVehicleController->getTraCI()->vehicle.getLaneIndex(vehicleId);
            intent.roadID = vanetza::asn1::allocate<LaneID_t>();
            intent.laneID = vanetza::asn1::allocate<LaneID_t>();
            intent.laneIDEnd = vanetza::asn1::allocate<LaneID_t>();
            *intent.roadID = laneIndex;
            *intent.laneID = laneIndex;
            *intent.laneIDEnd = laneIndex;
        } catch (const std::exception& e) {
            EV_WARN << "McService laneID optional fields unavailable for station "
                << vdp.station_id() << ": " << e.what() << '\n';
        }
    }
}

bool McService::appendPrerecordedIntentTrajectory(vanetza::asn1::Mcm& message) const
{
    if (!mUsePrerecordedIntentTrajectory) {
        return false;
    }

    if (!mVehicleController || mPrerecordedTrajectoryCsv.empty() ||
            mPrerecordedCx.empty() || mPrerecordedCy.empty()) {
        EV_WARN << "McService prerecorded plannedTrajectory generation unavailable; falling back to zero placeholder\n";
        return false;
    }

    const auto trajectory = mTrajectoryPlanner.calculateRefTrajectory(
        mPrerecordedTrajectorySteps,
        mPrerecordedTrajectoryDt,
        mPrerecordedCx,
        mPrerecordedCy,
        mPrerecordedTrajectoryIndex,
        true,
        0.0F,
        0.0F,
        0.0F);

    if (trajectory.empty()) {
        EV_WARN << "McService prerecorded plannedTrajectory generation returned no points; falling back to zero placeholder\n";
        return false;
    }

    IntentSharingContainer_t& intent = (*message).mcm.mcmParameters.intentionSharingContainer;
    const long heading = round(mVehicleDataProvider->heading(), decidegree);
    for (const auto& point : trajectory) {
        appendTrajectoryPoint(intent.plannedTrajectory, point, heading);
    }

    artery::State state;
    state.x = mVehicleController->getPositionSumo().x / boost::units::si::meter;
    state.y = mVehicleController->getPositionSumo().y / boost::units::si::meter;
    state.yaw = mVehicleDataProvider->heading() / boost::units::si::radians;
    state.v = mVehicleDataProvider->speed() / boost::units::si::meter_per_second;
    mPrerecordedTrajectoryIndex = mTrajectoryPlanner.calc_nearest_index(
        state,
        mPrerecordedCx,
        mPrerecordedCy,
        mPrerecordedTrajectoryIndex);

    EV_INFO << "McService generated prerecorded plannedTrajectory with "
        << trajectory.size() << " points for station " << (*message).header.stationID;
    if (mPrerecordedTrajectoryFallbackUsed) {
        EV_INFO << " using dummy/filler others_x/others_y";
    }
    EV_INFO << '\n';

    return true;
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

void McService::addManeuverNegotiationContainer(
    vanetza::asn1::Mcm& message,
    const VehicleDataProvider&,
    const mcm::PendingMcmCommand& command) const
{
    ManeuverNegotiationContainer_t*& negotiation = (*message).mcm.mcmParameters.maneuverNegotiationContainer;

    negotiation = vanetza::asn1::allocate<ManeuverNegotiationContainer_t>();
    negotiation->mcmCategory = mcmSubtypeToAsnCategory(command.subtype);
    negotiation->priorityManeuver = priorityToAsnPriority(command.priority);
    negotiation->cooperationTypeMCM = command.cooperationType;
    negotiation->requestID = command.requestId;
    negotiation->numberOfVehicles = command.numberOfVehicles;
    negotiation->negotiationVehicleID1 = command.targetVehicle1;
    if (command.hasTargetVehicle2 && command.numberOfVehicles > 1) {
        negotiation->negotiationVehicleID2 = vanetza::asn1::allocate<StationID_t>();
        *negotiation->negotiationVehicleID2 = command.targetVehicle2;
    }

    const long heading = round(mVehicleDataProvider->heading(), decidegree);
    for (const auto& point : command.requestedTrajectory) {
        appendTrajectoryPoint(negotiation->requestedTrajectory, point, heading);
    }

    if (command.requestedTrajectory.empty()) {
        appendZeroTrajectoryPoint(negotiation->requestedTrajectory);
    }

    if (command.hasOfferedTrajectory && !command.offeredTrajectory.empty()) {
        for (const auto& point : command.offeredTrajectory) {
            appendTrajectoryPoint(negotiation->offeredTrajectory, point, heading);
        }
    } else {
        appendZeroTrajectoryPoint(negotiation->offeredTrajectory);
    }

    EV_INFO << "McService serialized route_merging_1 " << mcmSubtypeName(command.subtype)
        << " container: requestId="
        << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " target1=" << command.targetVehicle1
        << " target2=" << command.targetVehicle2
        << " requestedTrajectoryPoints=" << command.requestedTrajectory.size()
        << " offeredTrajectoryPoints="
        << (command.hasOfferedTrajectory ? command.offeredTrajectory.size() : 0)
        << '\n';
}

void McService::addManeuverExecutionContainer(vanetza::asn1::Mcm& message, const VehicleDataProvider& vdp) const
{
    ManeuverExecutionContainer_t*& execution = (*message).mcm.mcmParameters.maneuverExecutionContainer;

    execution = vanetza::asn1::allocate<ManeuverExecutionContainer_t>();
    execution->mcmCategory = McmCategory_execute;
    execution->priorityManeuver = PriorityManeuver_low;
    execution->cooperationID = 0;
    execution->cooperationVehicleID1 = vdp.station_id();
    execution->cooperationVehicleID2 = nullptr;
}

void McService::addManeuverExecutionContainer(
    vanetza::asn1::Mcm& message,
    const VehicleDataProvider&,
    const mcm::PendingMcmCommand& command) const
{
    ManeuverExecutionContainer_t*& execution = (*message).mcm.mcmParameters.maneuverExecutionContainer;

    execution = vanetza::asn1::allocate<ManeuverExecutionContainer_t>();
    execution->mcmCategory = mcmSubtypeToAsnCategory(command.subtype);
    execution->priorityManeuver = priorityToAsnPriority(command.priority);
    execution->cooperationID = command.requestId;
    execution->cooperationVehicleID1 = command.targetVehicle1;
    execution->cooperationVehicleID2 = nullptr;

    if (command.hasTargetVehicle2 && command.numberOfVehicles > 1) {
        execution->cooperationVehicleID2 = vanetza::asn1::allocate<StationID_t>();
        *execution->cooperationVehicleID2 = command.targetVehicle2;
    }

    EV_INFO << "McService serialized execution " << mcmSubtypeName(command.subtype)
        << " container: requestId=" << static_cast<int>(command.requestId)
        << " numberOfVehicles=" << static_cast<int>(command.numberOfVehicles)
        << " cooperationVehicleID1=" << command.targetVehicle1
        << " cooperationVehicleID2=" << command.targetVehicle2
        << '\n';
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

    const auto profile = selectMcmDccProfile(
        mcm::operationMode::IntentionSharingMode,
        mcm::priorityMcmCategory::NoPriority);
    const vanetza::dcc::TransmissionLite mc_tx(profile, 0);
    vanetza::Clock::duration interval = trc->interval(mc_tx);
    SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
    emit(scSignalDccTimeWaitNextMcm, dcc);
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

        const MCM_t& message = **decodedMcm;
        const bool hasExecutionContainer = message.mcm.mcmParameters.maneuverExecutionContainer != nullptr;
        if (!hasExecutionContainer) {
            std::string error;
            if (!decodedMcm->validate(error)) {
                EV_WARN << "McService receive: invalid MCM, dropping packet: " << error << '\n';
                return;
            }
            EV_DETAIL << "McService receive: decoded and validated MCM, emitting McmReceived\n";
        } else {
            // The generated MCMextra CooperationID descriptor has no constraint
            // callback. Skip validation for execution-container MCMs for the
            // same reason as the send path, while preserving validation for
            // minimal and negotiation MCMs.
            EV_DETAIL << "McService receive: decoded execution-container MCM, emitting McmReceived\n";
        }

        McObject obj = visitor.shared_wrapper;
        emit(scSignalMcmReceived, &obj);
        emitReceivedMeasurements(message, *decodedMcm, simTime());
        if (!mLocalDynamicMapMCM) {
            mLocalDynamicMapMCM = getFacilities().get_mutable_ptr<LocalDynamicMapMCM>();
        }
        if (mLocalDynamicMapMCM) {
            mLocalDynamicMapMCM->updateAwarenessMCM(obj);
        } else {
            EV_WARN << "McService receive: LocalDynamicMapMCM unavailable; validated MCM awareness not stored\n";
        }

        mApplication->handleReceivedMcm(makeReceivedMcm(message, simTime()));
        emitPlannerMeasurements(mApplication->consumePlannerMeasurements());
        if (auto completedRequestId = mApplication->consumeCompletedRvNegotiationRequestId()) {
            const long completedKey = static_cast<long>(*completedRequestId);
            if (mMeasuredNegotiationCompletedRequestIds.insert(completedKey).second) {
                emit(scSignalNegotiationCompletedCounter, 1L);
                const auto startedAt = mNegotiationStartedAtByRequestId.find(completedKey);
                if (startedAt != mNegotiationStartedAtByRequestId.end()) {
                    // RV-side successful negotiation duration: first Request
                    // sent until McApplication observes either the final
                    // required Accept or, for a missing Accept, Execute from
                    // the missing expected CV as acceptance evidence.
                    // RV Execute send time and maneuver execution are excluded.
                    emit(scSignalNegotiationTime, simTime() - startedAt->second);
                    mNegotiationStartedAtByRequestId.erase(startedAt);
                }
            }
        }
        emitCurrentOperatingModeIfChanged();
    } catch (const std::exception& e) {
        EV_WARN << "McService receive: exception while decoding or validating MCM: " << e.what() << '\n';
    } catch (...) {
        EV_WARN << "McService receive: unknown exception while decoding or validating MCM\n";
    }
}

}  // namespace artery
