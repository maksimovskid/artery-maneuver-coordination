/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CASERVICE_H_
#define ARTERY_CASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/CamGenerationPolicy.h"
#include "artery/application/mcm/McmOperatingMode.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;

/**
 * CAM facilities/application service: dynamics/fixed-rate triggering, optional
 * facilities-DCC generation restriction, adaptive policy, packet construction
 * and submission. MCM mode/awareness are read-only inputs to CAM adaptation.
 *
 * This service does not negotiate maneuvers, plan trajectories or select MCM
 * priorities. Access DCC (LIMERIC/reactive), EDCA and MAC scheduling live below
 * it; a generated/submitted CAM is not necessarily transmitted on air then.
 */
class CaService : public ItsG5BaseService
{
	public:
		CaService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;
		void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, double, omnetpp::cObject*) override;

	private:
		void updateAdaptivePolicy();
		// baseline is configured once; temporary reasons only change the effective
		// period (Dynamics -> 1 Hz -> Dynamics; Fixed2Hz -> 1 Hz -> Fixed2Hz).
		CamGenerationPolicy mPolicy;
		mcm::operationMode mManeuverMode = mcm::operationMode::Unknown;
		double mLocalCbr = 0.0;
		bool mLogAdaptiveGeneration = false;
		unsigned mLastReductionReasons = 0;
		omnetpp::cModule* mCbrDriver = nullptr;
		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		// Immediate facilities generation/submission, not a MAC transmission callback.
		void sendCam(const omnetpp::SimTime&);
		// Existing DP2 throttle interval, converted to ms and clamped to min/max.
		omnetpp::SimTime genCamDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		LocalDynamicMap* mLocalDynamicMap = nullptr;

		omnetpp::SimTime mGenCamMin;
		omnetpp::SimTime mGenCamMax;
		omnetpp::SimTime mGenCam;
		unsigned mGenCamLowDynamicsCounter;
		unsigned mGenCamLowDynamicsLimit;
		Position mLastCamPosition;
		vanetza::units::Velocity mLastCamSpeed;
		vanetza::units::Angle mLastCamHeading;
		omnetpp::SimTime mLastCamTimestamp;
		omnetpp::SimTime mLastLowCamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
};

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider&, uint16_t genDeltaTime);
void addLowFrequencyContainer(vanetza::asn1::Cam&, unsigned pathHistoryLength = 0);

} // namespace artery

#endif /* ARTERY_CASERVICE_H_ */
