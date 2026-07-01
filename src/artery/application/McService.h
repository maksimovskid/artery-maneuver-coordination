#ifndef ARTERY_MCSERVICE_H_
#define ARTERY_MCSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/mcm/TrajectoryPlanner.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"

#include <omnetpp/simtime.h>
#include <vanetza/asn1/mcm.hpp>
#include <vanetza/btp/data_interface.hpp>
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
}

class VehicleDataProvider;
class NetworkInterfaceTable;
class Timer;
class LocalDynamicMapMCM;

class McService : public ItsG5BaseService
{
public:
    McService();
    ~McService() override;

    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;

private:
    // CA-service-style MCM generation rules
    void checkTriggeringConditions(const omnetpp::SimTime&);
    bool checkHeadingDelta() const;
    bool checkPositionDelta() const;
    bool checkSpeedDelta() const;
    omnetpp::SimTime genMcmDcc();

    // MCM transmission
    void sendMcm(const omnetpp::SimTime&);

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

    void addManeuverExecutionContainer(
        vanetza::asn1::Mcm&,
        const VehicleDataProvider&) const;

    // Facilities-layer state
    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    LocalDynamicMapMCM* mLocalDynamicMapMCM = nullptr;

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
    omnetpp::SimTime mFixedRateInterval;

    // Temporary disabled-by-default test mode
    bool mSendNegotiationTestMcm = false;

    bool mUsePrerecordedIntentTrajectory = false;
    std::string mPrerecordedTrajectoryCsv;
    int mPrerecordedTrajectorySteps = 20;
    double mPrerecordedTrajectoryDt = 0.25;
    mutable int mPrerecordedTrajectoryIndex = 0;
    mutable bool mPrerecordedTrajectoryFallbackUsed = false;
    mutable mcm::TrajectoryPlanner::Vec_f mPrerecordedCx;
    mutable mcm::TrajectoryPlanner::Vec_f mPrerecordedCy;
    mutable mcm::TrajectoryPlanner mTrajectoryPlanner;
    const traci::VehicleController* mVehicleController = nullptr;

    // Application layer handoff
    std::unique_ptr<mcm::McApplication> mApplication;
};

}  // namespace artery

#endif /* ARTERY_MCSERVICE_H_ */
