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
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace traci { class VehicleController; }

namespace artery
{

namespace mcm
{
class McApplication;
class McmRetransmissionCache;
enum class operationMode;
enum class priorityMcmCategory;
struct PendingMcmCommand;
struct PlannerMeasurement;
}

class VehicleDataProvider;
class NetworkInterfaceTable;
class Timer;
class LocalDynamicMapMCM;
class LocalEnvironmentModel;

class McService : public ItsG5BaseService
{
public:
    /**
     * Bridges Artery facilities, the MCM application state machine and the
     * Vanetza ITS-G5 stack. Negotiation, planning and execution decisions live
     * in McApplication and its planner modules; this class handles scheduling,
     * wire conversion, stack submission/reception and service measurements.
     */
    McService();
    ~McService() override;

    /** Resolve facilities, subscribe to channel load, validate configuration,
     * initialize McApplication and load an optional prerecorded route path. */
    void initialize() override;

    /** Decode an ITS-stack indication, validate/extract an MCM, reconstruct its
     * generation time, emit receive measurements and pass semantic data to the
     * local map and McApplication. Invoked by the BTP service callback. */
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;

    /** Consume the radio driver's channel-load signal for adaptive intent
     * admission. Values are expected as a channel-busy ratio in [0, 1]. */
    void receiveSignal(
        omnetpp::cComponent*,
        omnetpp::simsignal_t,
        double,
        omnetpp::cObject*) override;

    /** Middleware/service update callback (cadence comes from Middleware's
     * updateInterval). Refreshes ego state, advances the application, drains
     * measurements and evaluates generation policy; it does not itself imply
     * an MCM is generated. */
    void trigger() override;

    /** Accepted intent generation policies. Unknown strings fall back to
     * SameAsCam; the parser/name helper live in McService.cc. */
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

    /** Coordination generation policies. PeriodicFixedNoDcc bypasses only
     * this service's facilities DCC interval gate. */
    enum class CoordinationTriggeringCondition {
        SameAsCam,
        PeriodicFixed,
        PeriodicFixedNoDcc
    };

    /** Legacy timestamp configuration values. Current live plans and headers
     * are aligned to the millisecond generation timestamp regardless. */
    enum class McmTimeSource {
        DataProvider,
        CurrentTime
    };

    /** State of adaptive intent-rate reduction; Negotiation is currently
     * reserved and only None/Intent are used. */
    enum class McmFrequencyReduceState {
        None,
        Intent,
        Negotiation
    };

    /** Parsed communication settings. Intervals are simulation seconds and
     * CBR thresholds are unitless ratios in [0,1]. Some values are retained as
     * compatibility or staged hooks; see the corresponding NED parameters. */
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
    // Generation policy: coordination has precedence over intent when both
    // are eligible. These methods admit messages; they do not plan trajectories
    // or guarantee MAC transmission.
    /** Evaluate coordination first, then intent, and submit at most one MCM.
     * @param now Current middleware simulation time. */
    void checkTriggeringConditions(const omnetpp::SimTime&);
    /** Apply timing, selected intent policy, dynamics/adaptive state and the
     * facilities DCC interval gate. Updates no sent timestamp unless sendMcm runs. */
    bool shouldGenerateIntentMcm(const omnetpp::SimTime& now, omnetpp::SimTime dccInterval);
    /** Apply coordination policy to a pending command; a pending command takes
     * precedence over ordinary intent when eligible. */
    bool shouldGenerateCoordinationMcm(const omnetpp::SimTime& now, omnetpp::SimTime dccInterval);
    /** True when McApplication has an unsent coordination command. */
    bool hasPendingCoordinationMcm() const;
    /** Update the effective intent policy from the latest CBR and configured
     * adaptive flags; randomized delays govern recovery to the base policy. */
    void updateAdaptiveIntentFrequency(const omnetpp::SimTime& now);
    /** Whether the general adaptive intent rule is enabled (the separate MCO
     * 1 Hz rule has precedence). */
    bool adaptiveIntentRulesEnabled() const;
    /** Return the simulation-time interval associated with an intent policy. */
    omnetpp::SimTime intervalForIntentTriggeringCondition(IntentTriggeringCondition) const;
    /** Return the coordination interval; fixed coordination uses maxIntervalNegMcm. */
    omnetpp::SimTime intervalForCoordinationTriggeringCondition(CoordinationTriggeringCondition) const;
    /** Compare current provider heading with the last submitted MCM heading.
     * @return True when angular change exceeds the configured degree threshold. */
    bool checkHeadingDelta() const;
    /** Compare current provider position with the last submitted position.
     * @return True when displacement exceeds the configured metre threshold. */
    bool checkPositionDelta() const;
    /** Compare current provider speed with the last submitted speed.
     * @return True when speed change exceeds the configured m/s threshold. */
    bool checkSpeedDelta() const;
    /** Query the channel's transmit-rate throttle for the selected MCM profile;
     * emits the wait in seconds and clamps it to configured min/max intervals.
     * @return Facilities-generation interval in simulation seconds. */
    omnetpp::SimTime genMcmDcc();
    /** Map application mode and maneuver priority to the configured Vanetza
     * DCC profile. This selects a profile; it does not implement DCC. */
    vanetza::dcc::Profile selectMcmDccProfile(
        mcm::operationMode,
        mcm::priorityMcmCategory) const;
    /** Parse communication policy strings and copy generation/QoS settings. */
    void loadCommunicationConfig();
    /** Log parsed settings and staged/active adaptive policy inputs. */
    void logCommunicationConfig() const;
    /** Emit operating mode only when it differs from the last emitted value. */
    void emitCurrentOperatingModeIfChanged();
    /** Emit service-submission counters and deduplicated negotiation/execution
     * event counters for the serialized MCM. */
    void emitSentMeasurements(
        const MCM_t&,
        mcm::operationMode modeBeforeHandleSent);
    /** Emit receive counters, encoded byte size and generation-to-reception
     * delay measurements for a validated decoded packet. */
    void emitReceivedMeasurements(
        const MCM_t&,
        const vanetza::asn1::Mcm&,
        const omnetpp::SimTime&);
    /** Translate planner events/costs into registered OMNeT++ signals. */
    void emitPlannerMeasurements(const std::vector<mcm::PlannerMeasurement>&);

    // Outgoing service path. sendMcm prepares live state at the generation
    // opportunity, creates/retains a packet and submits it down the ITS stack;
    // submission is not confirmation of over-the-air transmission.
    /** Build or replay an MCM and submit it after facilities admission.
     * @param now Current generation opportunity; not the later MAC transmit time. */
    void sendMcm(const omnetpp::SimTime&);
    /** Snapshot the latest provider/controller state and refresh the application's
     * live trajectory/reference path.
     * @param now Current simulation evaluation/generation time. */
    void updateApplicationEgoContext(const omnetpp::SimTime&);

    // Builders create mandatory containers. The test builders add placeholder
    // negotiation/execution metadata; real commands use the PendingMcmCommand
    // overloads below.
    /** Create a validated intention MCM with a placeholder trajectory unless
     * prerecorded intent is enabled. generationDeltaTime is the live reference. */
    vanetza::asn1::Mcm createMinimalIntentionSharingMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    /** Build a placeholder negotiation message. Retained as a test helper; the
     * active smoke hook attaches its placeholder container in sendMcm.
     * @param generationDeltaTime Header's 16-bit millisecond value.
     * @return Validated minimal message when the descriptor supports it. */
    vanetza::asn1::Mcm createMinimalNegotiationTestMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    /** Build a placeholder execution message; not selected by the current
     * send path and not a real accepted agreement.
     * @param generationDeltaTime Header's 16-bit millisecond value. */
    vanetza::asn1::Mcm createMinimalExecutionTestMessage(
        const VehicleDataProvider&,
        uint16_t generationDeltaTime) const;

    // ASN.1 container conversion. Coordinates use the runtime's SUMO frame;
    // trajectory times are encoded as absolute simulation centiseconds.
    /** Fill the ITS PDU version, MCM message ID and provider station ID. */
    void fillHeader(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    /** Store the 16-bit millisecond generation timestamp. Live trajectory
     * reference times use this same representable timestamp.
     * @param generationDeltaTime Low 16 bits of the Timer millisecond epoch. */
    void fillGenerationDeltaTime(
        vanetza::asn1::Mcm&,
        uint16_t generationDeltaTime) const;

    /** Fill static station type and current geographic position; unavailable
     * altitude/confidence values follow the existing schema conventions. */
    void fillBasicContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    /** Fill current heading/speed and optional lane metadata, then append a
     * live prerecorded prediction or the existing placeholder point. */
    void fillIntentionSharingContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;
    /** Append the already refreshed application prediction from the selected
     * CSV route columns.
     * @return False when configuration/state is unavailable. */
    bool appendPrerecordedIntentTrajectory(vanetza::asn1::Mcm&) const;

    /** Add a zero-ID test Request and placeholder trajectories. */
    void addManeuverNegotiationContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    /** Serialize a real application command. Request/Offer proposals are
     * immutable after issue; Confirm/Accept copies retain agreement snapshots. */
    void addManeuverNegotiationContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&,
        const mcm::PendingMcmCommand&) const;

    /** Add a zero-ID execution test container. */
    void addManeuverExecutionContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    /** Serialize execution identity/participants from the accepted command;
     * the live remaining trajectory is refreshed separately from current state. */
    void addManeuverExecutionContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&,
        const mcm::PendingMcmCommand&) const;

    // Facilities and service dependencies. Facility pointers are borrowed;
    // McApplication and the retransmission cache below are owned by this module.
    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    LocalDynamicMapMCM* mLocalDynamicMapMCM = nullptr;
    const LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;

    // Message-generation state: last submitted MCM kinematics/time, base and
    // clamped interval, dynamics thresholds, and low-dynamics backoff. Service
    // triggers/planning ticks do not update these as if a packet were sent.
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
    // Latest clamped channel-busy ratio received from the radio driver. It
    // drives the currently enabled adaptive intent rules below.
    double mLocalCbr = 0.0;
    // Cached generation, retry and overall negotiation limits copied from NED.
    omnetpp::SimTime mFixedRateInterval;
    omnetpp::SimTime mNegotiationRetryInterval;
    omnetpp::SimTime mNegotiationLimitMerging;
    omnetpp::SimTime mNegotiationLimitLaneChange;

    // Smoke-test controls are inactive unless explicitly enabled in NED.
    bool mSendNegotiationTestMcm = false;
    bool mForceFirstCvRejectForSecondRequestSmoke = false;
    uint32_t mForceFirstCvRejectStationId = 0;
    // Parsed policy. timeSource, fixedInterval and the medium CBR threshold
    // remain compatibility/logging values in the current path.
    McmCommunicationConfig mCommunicationConfig;
    // Effective intent policy can temporarily differ from the configured base
    // policy while adaptive reduction/recovery is active.
    IntentTriggeringCondition mEffectiveIntentTrigger = IntentTriggeringCondition::SameAsCam;
    McmFrequencyReduceState mFrequencyReduceState = McmFrequencyReduceState::None;
    omnetpp::SimTime mFrequencyReducedIntentAt = omnetpp::SimTime::ZERO;
    omnetpp::SimTime mFrequencyReduceIntentRecoveryDelay = omnetpp::SimTime::ZERO;
    // Measurement deduplication and event-time state, keyed by request ID.
    bool mHasLastEmittedOperatingMode = false;
    mcm::operationMode mLastEmittedOperatingMode;
    std::set<long> mMeasuredNegotiationStartedRequestIds;
    std::set<long> mMeasuredNegotiationCompletedRequestIds;
    std::set<long> mMeasuredExecutionStartedRequestIds;
    std::set<long> mMeasuredExecutionCompletedRequestIds;
    std::set<long> mMeasuredRejectedRequestIds;
    std::map<long, omnetpp::SimTime> mNegotiationStartedAtByRequestId;

    // Optional CSV reference path. Index and fallback flags are mutable because
    // const message builders advance/record selection during service operation.
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

    // Application state machine and complete issued Request/Offer packets.
    std::unique_ptr<mcm::McApplication> mApplication;
    std::unique_ptr<mcm::McmRetransmissionCache> mRetransmissionCache;
};

}  // namespace artery

#endif /* ARTERY_MCSERVICE_H_ */
