#ifndef ARTERY_MCSERVICE_H_
#define ARTERY_MCSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"

#include <omnetpp/simtime.h>
#include <vanetza/asn1/mcm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace traci { class VehicleController; }

namespace artery
{

namespace mcm
{
class McApplication;
enum class operationMode;
enum class priorityMcmCategory;
struct PendingMcmCommand;
}

class VehicleDataProvider;
class NetworkInterfaceTable;
class Timer;
class LocalDynamicMapMCM;
class LocalEnvironmentModel;

class McService : public ItsG5BaseService
{
public:
    McService();
    ~McService() override;

    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void receiveSignal(
        omnetpp::cComponent*,
        omnetpp::simsignal_t,
        double,
        omnetpp::cObject*) override;
    void trigger() override;

    // Typed representation of old MCM generation/QoS NED strings. These are
    // public type names only; McService still owns all configuration behavior.
    enum class IntentTriggeringCondition {
        SameAsCam,
        SameAsCamReduced2Hz,
        SameAsCamReduced1Hz,
        PeriodicFixed,
        PeriodicFixedHalfHz,
        PeriodicFixed1Hz,
        PeriodicFixed2Hz,
        PeriodicFixed3Hz,
        PeriodicFixed5Hz
    };

    enum class CoordinationTriggeringCondition {
        SameAsCam,
        PeriodicFixed,
        PeriodicFixedNoDcc
    };

    enum class McmTimeSource {
        DataProvider,
        CurrentTime
    };

    enum class McmFrequencyReduceState {
        None,
        Intent,
        Negotiation
    };

    struct McmCommunicationConfig {
        IntentTriggeringCondition intentTrigger = IntentTriggeringCondition::SameAsCam;
        CoordinationTriggeringCondition coordinationTrigger = CoordinationTriggeringCondition::SameAsCam;
        bool withDccRestriction = false;
        omnetpp::SimTime minInterval;
        omnetpp::SimTime maxInterval;
        bool fixedRate = true;
        omnetpp::SimTime fixedRateInterval;
        omnetpp::SimTime fixedInterval;
        omnetpp::SimTime maxIntervalNegMcm;
        omnetpp::SimTime negotiationRetryInterval;
        omnetpp::SimTime negotiationLimitMerging;
        omnetpp::SimTime negotiationLimitLaneChange;
        bool dccProfiles = false;
        bool dccOnlyMcm = false;
        McmTimeSource timeSource = McmTimeSource::DataProvider;
        bool newGenMcmRules = false;
        bool newGenMcmRulesIntent = false;
        bool newGenMcmRulesIntent1HzMco = false;
        double freqReduceCbrMin = 0.5;
        double freqReduceCbrMedium = 0.6;
        double freqReduceCbrMax = 0.65;
        double freqReduceCbrMco = 0.5;
    };

private:
    // CA-service-style MCM generation rules
    void checkTriggeringConditions(const omnetpp::SimTime&);
    bool shouldGenerateIntentMcm(const omnetpp::SimTime& now, omnetpp::SimTime dccInterval);
    bool shouldGenerateCoordinationMcm(const omnetpp::SimTime& now, omnetpp::SimTime dccInterval) const;
    void updateAdaptiveIntentFrequency(const omnetpp::SimTime& now);
    bool adaptiveIntentRulesEnabled() const;
    omnetpp::SimTime intervalForIntentTriggeringCondition(IntentTriggeringCondition) const;
    omnetpp::SimTime intervalForCoordinationTriggeringCondition(CoordinationTriggeringCondition) const;
    bool checkHeadingDelta() const;
    bool checkPositionDelta() const;
    bool checkSpeedDelta() const;
    omnetpp::SimTime genMcmDcc();
    vanetza::dcc::Profile selectMcmDccProfile(
        mcm::operationMode,
        mcm::priorityMcmCategory) const;
    void loadCommunicationConfig();
    void logCommunicationConfig() const;

    // MCM transmission
    void sendMcm(const omnetpp::SimTime&);
    void updateApplicationEgoContext(const omnetpp::SimTime&);

    // MCM message builders
    vanetza::asn1::Mcm createMinimalIntentionSharingMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    vanetza::asn1::Mcm createMinimalNegotiationTestMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    vanetza::asn1::Mcm createMinimalExecutionTestMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    // MCM container helpers
    void fillHeader(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    void fillGenerationDeltaTime(
        vanetza::asn1::Mcm&,
        uint16_t generationDeltaTime) const;

    void fillBasicContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    void fillIntentionSharingContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;
    bool appendPrerecordedIntentTrajectory(vanetza::asn1::Mcm&) const;

    void addManeuverNegotiationContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    void addManeuverNegotiationContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&,
        const mcm::PendingMcmCommand&) const;

    void addManeuverExecutionContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    void addManeuverExecutionContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&,
        const mcm::PendingMcmCommand&) const;

    // Facilities-layer state
    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    LocalDynamicMapMCM* mLocalDynamicMapMCM = nullptr;
    const LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;

    // CA-service-style generation state
    omnetpp::SimTime mGenMcmMin;
    omnetpp::SimTime mGenMcmMax;
    omnetpp::SimTime mGenMcm;
    unsigned mGenMcmLowDynamicsCounter = 0;
    unsigned mGenMcmLowDynamicsLimit = 3;

    Position mLastMcmPosition;
    vanetza::units::Velocity mLastMcmSpeed;
    vanetza::units::Angle mLastMcmHeading;
    bool mHasLastMcmKinematics = false;
    omnetpp::SimTime mLastMcmTimestamp;

    vanetza::units::Angle mHeadingDelta;
    vanetza::units::Length mPositionDelta;
    vanetza::units::Velocity mSpeedDelta;

    bool mDccRestriction = false;
    bool mFixedRate = false;
    // Local channel busy ratio is staged input for future adaptive MCM
    // generation rules. It is observed and logged/configured now, but it does
    // not alter triggering conditions or message timing in this step.
    double mLocalCbr = 0.0;
    omnetpp::SimTime mFixedRateInterval;
    omnetpp::SimTime mNegotiationRetryInterval;
    omnetpp::SimTime mNegotiationLimitMerging;
    omnetpp::SimTime mNegotiationLimitLaneChange;

    // Temporary disabled-by-default test mode
    bool mSendNegotiationTestMcm = false;
    McmCommunicationConfig mCommunicationConfig;
    IntentTriggeringCondition mEffectiveIntentTrigger = IntentTriggeringCondition::SameAsCam;
    McmFrequencyReduceState mFrequencyReduceState = McmFrequencyReduceState::None;
    omnetpp::SimTime mFrequencyReducedIntentAt = omnetpp::SimTime::ZERO;
    omnetpp::SimTime mFrequencyReduceIntentRecoveryDelay = omnetpp::SimTime::ZERO;

    bool mUsePrerecordedIntentTrajectory = false;
    std::string mPrerecordedTrajectoryCsv;
    int mPrerecordedTrajectorySteps = 20;
    double mPrerecordedTrajectoryDt = 0.25;
    mutable int mPrerecordedTrajectoryIndex = 0;
    mutable bool mPrerecordedTrajectoryFallbackUsed = false;
    mutable mcm::TrajectoryPlanner::Vec_f mPrerecordedCx;
    mutable mcm::TrajectoryPlanner::Vec_f mPrerecordedCy;
    mutable mcm::TrajectoryPlanner mTrajectoryPlanner;
    traci::VehicleController* mVehicleController = nullptr;

    // Application layer handoff
    std::unique_ptr<mcm::McApplication> mApplication;
};

}  // namespace artery

#endif /* ARTERY_MCSERVICE_H_ */
