#include "artery/application/Timer.h"
#include "artery/application/mcm/McmReception.h"
#include "artery/application/mcm/McmTiming.h"
#include "artery/application/mcm/McmTrajectory.h"
#include "artery/application/mcm/McmRetransmission.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp.h>
#include <omnetpp/cnullenvir.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace artery;
using namespace artery::mcm;
using omnetpp::SimTime;
namespace {
void require(bool value, const char* what) { if (!value) throw std::runtime_error(what); }
void near(double a, double b, const char* what) { require(std::abs(a-b) < 1e-8, what); }
template<class F> void invalid(F call, const char* what) {
    try { call(); } catch (const std::invalid_argument&) { return; }
    throw std::runtime_error(what);
}
McmTrajectory trajectory(double origin = 10, double y = 100) {
    McmTrajectory t(3);
    for (std::size_t i=0;i<t.size();++i) {
        t[i].mX = 5;
        t[i].mY = y - 27 * 0.25 * i;
        t[i].mReferenceTime = SimTime(origin);
        t[i].mTime = SimTime(origin + 0.25 * (i+1));
    }
    return t;
}
uint16_t stamp(const Timer& timer, double time) {
    return static_cast<uint16_t>(countTaiMilliseconds(timer.getTimeFor(SimTime(time))));
}
vanetza::asn1::Mcm packet(const Timer& timer, double generation) {
    vanetza::asn1::Mcm packet;
    auto& p = (*packet).mcm;
    p.generationDeltaTime = stamp(timer, generation);
    auto& intent = p.mcmParameters.intentionSharingContainer;
    intent.vehicleLength.vehicleLengthValue = 1; intent.vehicleWidth = 1;
    intent.heading.headingConfidence = 1; intent.speed.speedConfidence = 1;
    for (int i=0; i<3; ++i) {
        auto point = vanetza::asn1::allocate<TrajectoryPointMCM_t>();
        point->deltaLongitudinalPosition = 5; point->deltaLateralPosition = 100 - i*7;
        point->deltaTime = std::lround((generation + 0.25*(i+1))*100);
        ASN_SEQUENCE_ADD(&intent.plannedTrajectory, point);
    }
    return packet;
}
void negotiation(vanetza::asn1::Mcm& message, const PendingMcmCommand& command) {
    auto& n=message->mcm.mcmParameters.maneuverNegotiationContainer;
    n=vanetza::asn1::allocate<ManeuverNegotiationContainer_t>();
    n->mcmCategory=static_cast<long>(command.subtype);
    n->cooperationTypeMCM=command.cooperationType;
    n->priorityManeuver=1; n->requestID=command.requestId; n->numberOfVehicles=command.numberOfVehicles;
    n->negotiationVehicleID1=command.targetVehicle1;
    if (command.hasTargetVehicle2) {
        n->negotiationVehicleID2=vanetza::asn1::allocate<StationID_t>();
        *n->negotiationVehicleID2=command.targetVehicle2;
    }
    const auto append = [](auto& target, const auto& points) {
        if (points.empty()) ASN_SEQUENCE_ADD(&target, vanetza::asn1::allocate<TrajectoryPointMCM_t>());
        for (const auto& sample : points) {
            auto point=vanetza::asn1::allocate<TrajectoryPointMCM_t>();
            point->deltaLongitudinalPosition=std::lround(sample.mX);
            point->deltaLateralPosition=std::lround(sample.mY);
            point->deltaHeading=std::lround(sample.mHeading*1800/M_PI);
            point->deltaTime=std::lround(sample.mTime.dbl()*100);
            ASN_SEQUENCE_ADD(&target,point);
        }
    };
    append(n->requestedTrajectory,command.requestedTrajectory);
    append(n->offeredTrajectory,command.offeredTrajectory);
}
// Runtime-shaped packets: only the OPTIONAL fields populated by McService.
void runtimeRoundTrip(omnetpp::cSimulation& sim, Timer& timer) {
    sim.setSimTime(SimTime(10.1));
    VehicleDataProvider vdp(101); VehicleKinematics k;
    k.speed=27.0 * boost::units::si::meter_per_second; vdp.update(k);
    TrajectoryPlanner planner; planner.mVehicleDataProvider=&vdp;
    for (bool multi : {false,true})
    for (auto subtype : {mcmSubtype::Request,mcmSubtype::Offer,mcmSubtype::Confirm,mcmSubtype::Accept,mcmSubtype::Execute,mcmSubtype::Regular}) {
        const bool cv=subtype==mcmSubtype::Offer || subtype==mcmSubtype::Accept;
        const auto sender=cv ? 309u : 101u;
        const auto target=cv ? 101u : 309u;
        const bool second=multi && !cv;
        auto raw=packet(timer,10);
        raw->header.stationID=sender; raw->header.protocolVersion=2; raw->header.messageID=20;
        auto& parameters=raw->mcm.mcmParameters;
        auto& basic=parameters.basicContainerMCM;
        basic.stationType=5; basic.referencePosition.longitude=134050000;
        basic.referencePosition.latitude=525200000;
        auto& intent=parameters.intentionSharingContainer;
        intent.heading.headingValue=1800; intent.speed.speedValue=2700;
        intent.vehicleLength.vehicleLengthValue=1023; intent.vehicleWidth=62;
        intent.roadID=vanetza::asn1::allocate<LaneID_t>(); *intent.roadID=1;
        intent.laneID=vanetza::asn1::allocate<LaneID_t>(); *intent.laneID=1;
        intent.laneIDEnd=vanetza::asn1::allocate<LaneID_t>(); *intent.laneIDEnd=1;
        // Replace the short helper trajectory with the runtime's 20-point horizon.
        ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_TrajectoryMCM,&intent.plannedTrajectory);
        intent.plannedTrajectory={};
        McmTrajectory points(20);
        for (int i=0;i<20;++i) {
            auto point=vanetza::asn1::allocate<TrajectoryPointMCM_t>();
            point->deltaLongitudinalPosition=5; point->deltaLateralPosition=1000-7*i;
            point->deltaHeading=-900; point->deltaTime=1025+25*i;
            ASN_SEQUENCE_ADD(&intent.plannedTrajectory,point);
            points[i].mX=5; points[i].mY=1000-7*i; points[i].mHeading=-M_PI/2;
            points[i].mTime=SimTime(10.25+0.25*i); points[i].mReferenceTime=SimTime(10);
        }
        if (subtype==mcmSubtype::Execute) {
            auto& e=parameters.maneuverExecutionContainer;
            e=vanetza::asn1::allocate<ManeuverExecutionContainer_t>();
            e->mcmCategory=static_cast<long>(subtype); e->cooperationID=77;
            e->cooperationVehicleID1=309;
            if(multi) { e->cooperationVehicleID2=vanetza::asn1::allocate<StationID_t>(); *e->cooperationVehicleID2=589; }
        } else if(subtype!=mcmSubtype::Regular) {
            PendingMcmCommand c; c.subtype=subtype; c.requestId=77; c.numberOfVehicles=multi ? 2 : 1;
            c.targetVehicle1=target; c.hasTargetVehicle2=second; c.targetVehicle2=589;
            c.cooperationType=0; c.requestedTrajectory=points;
            if(subtype!=mcmSubtype::Request) c.offeredTrajectory=points;
            negotiation(raw,c);
        }
        vanetza::asn1::Mcm decoded;
        const auto label=std::string("runtime packet byte decode subtype=")+
            std::to_string(static_cast<int>(subtype))+" participants="+(multi ? "2" : "1");
        require(decoded.decode(raw.encode()),label.c_str());
        const auto r=decodeReceivedMcm(*decoded,sim.getSimTime(),timer);
        require(r.data.stationId==sender && r.generatedAt==SimTime(10),"runtime sender and generation");
        require(r.data.hasLaneId && r.data.laneId==1,"runtime CV-selection laneID");
        require(r.data.speedValue==2700 && r.data.headingValue==1800 &&
            r.data.longitude==134050000 && r.data.latitude==525200000,"runtime basic/dynamic state");
        require(decoded->mcm.mcmParameters.intentionSharingContainer.vehicleWidth==62 &&
            decoded->mcm.mcmParameters.intentionSharingContainer.vehicleLength.vehicleLengthValue==1023,
            "serialized vehicle geometry representation");
        const auto verify=[&](const McmTrajectory& actual,bool live) {
            require(actual.size()==20,"runtime trajectory point count");
            for (int i=0;i<20;++i) {
                near(actual[i].mX,points[i].mX,"runtime trajectory X");
                near(actual[i].mY,points[i].mY,"runtime trajectory Y");
                near(actual[i].mTime.dbl(),points[i].mTime.dbl(),"runtime point centiseconds");
                near(actual[i].mHeading,-M_PI/2,"runtime point heading");
            }
            // Agreement copies are checked above, never treated as live predictions.
            if(!live) return;
            auto ego=actual;
            for(auto& point:ego) point.mX=10000;
            ego[0].mX=actual[0].mX; ego[0].mY=actual[0].mY-2.8;
            require(planner.check_traj_conflict(ego,actual,0.01,r.plannedAge(SimTime(10.1))),
                "decoded trajectory reaches real compensated conflict path");
            require(!planner.check_traj_conflict(ego,actual,0.01,SimTime(0)),
                "decoded conflict depends on full network age");
            auto faster=actual;
            for(std::size_t i=0;i<faster.size();++i) { faster[i].mX=actual[i].mX; faster[i].mY=actual[0].mY-2.8-10*i; }
            faster[0].mX=actual[0].mX;
            require(planner.check_traj_collision(faster,actual,0.01,r.plannedAge(SimTime(10.1))),
                "decoded trajectory reaches real compensated TTC path");
            require(!planner.check_traj_collision(faster,actual,0.01,SimTime(0)),
                "decoded TTC depends on full network age");
            near(actual[0].mY-compensateTrajectoryPoint(actual,0,r.plannedAge(SimTime(10.15)),
                CompensationDirection::NegativeY).mY,4.2,"decoded cached compensation");
        };
        verify(r.data.plannedTrajectory,true);
        if(subtype==mcmSubtype::Execute) {
            require(r.data.cooperationId==77 && r.data.cooperationVehicleId1==309 &&
                r.data.hasCooperationVehicleId2==multi && (!multi || r.data.cooperationVehicleId2==589),"runtime execution identity");
        } else if(subtype==mcmSubtype::Regular) {
            require(!r.data.hasNegotiationContainer && !r.data.hasExecutionContainer,
                "intention-only packet remains intention sharing");
        } else {
            require(r.data.requestId==77 && r.data.numberOfVehicles==(multi ? 2 : 1) && r.data.negotiationVehicleId1==target &&
                r.data.hasNegotiationVehicleId2==second && (!second || r.data.negotiationVehicleId2==589),"runtime negotiation identity");
            verify(r.data.requestedTrajectory,subtype==mcmSubtype::Request);
            if(subtype!=mcmSubtype::Request) verify(r.data.offeredTrajectory,false);
            else require(r.data.offeredTrajectory.size()==1 && r.data.offeredTrajectory[0].mX==0,
                "Request retains runtime offered placeholder");
            require(r.data.mcmCategory==static_cast<long>(subtype) && r.data.priorityManeuver==1 &&
                r.data.cooperationTypeMcm==0,"runtime negotiation classification");
        }
    }
}
void ageAndTimestamps(omnetpp::cSimulation& sim, Timer& timer) {
    sim.setSimTime(SimTime(10.1));
    auto p = packet(timer,10.0);
    PendingMcmCommand request; request.requestId=7; request.numberOfVehicles=2;
    request.hasTargetVehicle2=true; request.targetVehicle1=2; request.targetVehicle2=3;
    request.requestedTrajectory=trajectory(10);
    negotiation(p,request);
    auto r = decodeReceivedMcm(*p,sim.getSimTime(),timer);
    near(r.plannedAge(SimTime(10.1)).dbl(),0.1,"immediate age includes generation-to-reception delay");
    near(r.plannedAge(SimTime(10.15)).dbl(),0.15,"cached age includes elapsed residence");
    near(r.requestedAge(SimTime(10.1)).dbl(),0.1,"single header-derived information age");
    near(r.data.plannedTrajectory.front().mTime.dbl(),10.25,"original absolute-centisecond point encoding");
    vanetza::asn1::Mcm decoded;
    require(decoded.decode(p.encode()),"ORIGINAL schema ASN.1 encode/decode round trip");
    auto wire = decodeReceivedMcm(*decoded,sim.getSimTime(),timer);
    require(wire.generatedAt==SimTime(10),"generationDeltaTime survives original-schema wire round trip");
    require(wire.data.requestedTrajectory.size()==3 && wire.data.negotiationVehicleId2==3,
        "original negotiation trajectory and optional participant survive encoding");
    sim.setSimTime(SimTime(10.100000207459));
    auto fractional = decodeReceivedMcm(*decoded,sim.getSimTime(),timer);
    require(fractional.generatedAt==wire.generatedAt,"fractional reception cannot alter generation epoch");
    near(fractional.plannedAge(sim.getSimTime()).dbl(),0.100000207459,"fractional evaluation retained in age");
    sim.setSimTime(SimTime(0.1));
    auto beforeZero=packet(timer,-0.1);
    near(decodeReceivedMcm(*beforeZero,sim.getSimTime(),timer).generatedAt.dbl(),-0.1,
        "valid generation before simulation zero is not clamped");
    invalid([]{timestampOffsetMilliseconds(-1,10000);},"negative malformed timestamp");
    invalid([]{timestampOffsetMilliseconds(65536,10000);},"oversize malformed timestamp");
    invalid([]{timestampOffsetMilliseconds(10101,10100);},"future timestamp");
    invalid([]{timestampOffsetMilliseconds(32768,0);},"half-period ambiguous timestamp");
    require(timestampOffsetMilliseconds(65530,65540)==-10,"wrap boundary");
    require(timestampOffsetMilliseconds(10,65556)==-10,"after wrap");
    near(timestampOffsetMilliseconds(10000,10100),-100,"normal timestamp milliseconds");
    const uint64_t base=countTaiMilliseconds(timer.getTimeFor(SimTime::ZERO));
    const double boundary=(65536-(base&65535))/1000.0;
    sim.setSimTime(SimTime(boundary+0.1));
    auto wrap=packet(timer,boundary-0.1);
    near(decodeReceivedMcm(*wrap,sim.getSimTime(),timer).plannedAge(sim.getSimTime()).dbl(),0.2,
        "real Timer wrap reconstruction");
    auto future=packet(timer,boundary+0.101);
    invalid([&]{decodeReceivedMcm(*future,sim.getSimTime(),timer);},"future generation rejected");
    p->mcm.generationDeltaTime=65536;
    invalid([&]{decodeReceivedMcm(*p,sim.getSimTime(),timer);},"raw timestamp checked before narrowing");
}
void retransmissions(omnetpp::cSimulation& sim, Timer& timer) {
    McmRetransmissionCache cache;
    PendingMcmCommand request; request.subtype=mcmSubtype::Request; request.requestId=7;
    request.requestedTrajectory=trajectory(10);
    auto original=std::make_shared<vanetza::asn1::Mcm>(packet(timer,10));
    negotiation(*original,request);
    cache.remember(request,original,SimTime(10));
    auto retry=request; retry.retransmission=true;
    const auto& retained=cache.replay(retry);
    require(retained.message->encode()==original->encode(),"Request retry repeats entire issued packet unchanged");
    sim.setSimTime(SimTime(10.6));
    near(decodeReceivedMcm(**retained.message,sim.getSimTime(),timer).plannedAge(sim.getSimTime()).dbl(),0.6,
        "Request retry retains original generation age, including retry waiting");
    auto changed=retry; changed.requestedTrajectory[0].mY+=5;
    bool rejected=false; try {cache.replay(changed);} catch(const std::logic_error&){rejected=true;}
    require(rejected,"changed proposal cannot reuse issued packet or timestamp");
    PendingMcmCommand offer=request; offer.subtype=mcmSubtype::Offer;
    offer.hasOfferedTrajectory=true; offer.offeredTrajectory=trajectory(10.1,50);
    auto issuedOffer=std::make_shared<vanetza::asn1::Mcm>(packet(timer,10.1));
    negotiation(*issuedOffer,offer);
    cache.remember(offer,issuedOffer,SimTime(10.1)); offer.retransmission=true;
    require(cache.replay(offer).message->encode()==issuedOffer->encode(),"Offer retry repeats original complete MCM");
    // Confirm/Accept are new packets with fresh intent, but their requested/
    // offered copies are immutable agreement snapshots, not new predictions.
    auto confirm=packet(timer,10.5);
    auto confirmCommand=request; confirmCommand.subtype=mcmSubtype::Confirm;
    negotiation(confirm,confirmCommand); sim.setSimTime(SimTime(10.6));
    auto receivedConfirm=decodeReceivedMcm(*confirm,sim.getSimTime(),timer);
    auto receivedRequest=decodeReceivedMcm(**original,sim.getSimTime(),timer);
    require(sameMcmProposal(receivedConfirm.data.requestedTrajectory,receivedRequest.data.requestedTrajectory),
        "fresh Confirm preserves original Request snapshot");
    bool snapshotRejected=false;
    try {receivedConfirm.requestedAge(sim.getSimTime());} catch(const std::logic_error&){snapshotRejected=true;}
    require(snapshotRejected,"Confirm agreement is not interpreted as fresh requested prediction");
    auto accept=packet(timer,10.6);
    auto acceptCommand=offer; acceptCommand.subtype=mcmSubtype::Accept; acceptCommand.retransmission=false;
    negotiation(accept,acceptCommand); sim.setSimTime(SimTime(10.7));
    auto receivedAccept=decodeReceivedMcm(*accept,sim.getSimTime(),timer);
    auto receivedOffer=decodeReceivedMcm(**issuedOffer,sim.getSimTime(),timer);
    require(sameMcmProposal(receivedAccept.data.offeredTrajectory,receivedOffer.data.offeredTrajectory),
        "fresh Accept preserves immutable CV Offer snapshot");
    near(receivedAccept.plannedAge(sim.getSimTime()).dbl(),0.1,"Accept's live intent uses its own generation time");
    requireGenerationReference(trajectory(10.5),generationReferenceTime(SimTime(10.5009)));
    rejected=false;
    try {requireGenerationReference(trajectory(10),SimTime(10.5));} catch(const std::logic_error&){rejected=true;}
    require(rejected,"fresh generation refuses stale live content");
    // DCC defers first generation: rebuild from current state BEFORE stamping.
    auto refreshed=trajectory(10.5,80); requireGenerationReference(refreshed,SimTime(10.5));
    PendingMcmCommand replacement=request; replacement.requestId=8; replacement.requestedTrajectory=refreshed;
    auto freshPacket=std::make_shared<vanetza::asn1::Mcm>(packet(timer,10.5));
    negotiation(*freshPacket,replacement); cache.remember(replacement,freshPacket,SimTime(10.5));
    rejected=false; try {cache.replay(retry);} catch(const std::logic_error&){rejected=true;}
    require(rejected,"new proposal invalidates old cached retransmission");
}
void compensation() {
    auto t = trajectory();
    VehicleDataProvider vdp(1);
    VehicleKinematics kinematics;
    kinematics.speed = 27.0 * boost::units::si::meter_per_second;
    vdp.update(kinematics);
    TrajectoryPlanner planner; planner.mVehicleDataProvider=&vdp;
    auto indexEgo=t; indexEgo[2].mY += 10;
    TrajectoryPlanner::Vec_f routeX {5,5,5};
    TrajectoryPlanner::Vec_f routeY {100,93.25,96.5};
    require(planner.findIndexTrajsConflictFree(indexEgo,t,0.05,routeX,routeY,0,SimTime(0))==2,
        "uncompensated lane-change index fixture");
    require(planner.findIndexTrajsConflictFree(indexEgo,t,0.05,routeX,routeY,0,SimTime(0.1))==0,
        "lane-change index selection consumes full information age once");
    auto conflictEgo=t;
    for (auto& point : conflictEgo) point.mX=1000;
    conflictEgo[0].mX=t[0].mX; conflictEgo[0].mY=t[0].mY-2.7;
    require(planner.check_traj_conflict(conflictEgo,t,0.01,SimTime(0.1)),"conflict consumer corrects first point");
    require(!planner.check_traj_conflict(conflictEgo,t,0.01,SimTime(0.05)),"conflict consumer threshold");
    auto endpointEgo=t;
    require(!planner.check_traj_conflict_merging(endpointEgo,t,0.01,SimTime(0.1),true),
        "safe compensated endpoint does not retain stale conflict evidence");
    require(!planner.check_traj_conflict({},t,1,SimTime(1)),"empty consumer trajectory");
    auto single=t; single.resize(1);
    require(!planner.check_traj_conflict(single,single,1,SimTime(1)),"single consumer point has insufficient trajectory");
    for (auto index : {0u,1u,2u}) {
        for (auto age : {0.0,0.049,0.05})
            near(compensateTrajectoryPoint(t,index,SimTime(age),CompensationDirection::NegativeY).mY,
                t[index].mY,"strict 50 ms threshold");
        near(t[index].mY-compensateTrajectoryPoint(t,index,SimTime(0.1),CompensationDirection::NegativeY).mY,
            2.7,"100 ms at 27 m/s including first point");
        near(t[index].mY-compensateTrajectoryPoint(t,index,SimTime(0.15),CompensationDirection::NegativeY).mY,
            4.05,"150 ms at 27 m/s");
        near(t[index].mY-compensateTrajectoryPoint(t,index,SimTime(0.051),CompensationDirection::NegativeY).mY,
            27*0.051,"above threshold");
    }
    require(trajectorySegmentSpeed({},0)==0,"empty speed");
    require(compensateTrajectoryPoint({},0,SimTime(0.1),CompensationDirection::NegativeY).mY==0,"empty point");
    auto one = t; one.resize(1);
    near(compensateTrajectoryPoint(one,0,SimTime(1),CompensationDirection::NegativeY).mY,one[0].mY,"one point unchanged");
    auto zero = t; zero[1].mTime=zero[0].mTime;
    near(compensateTrajectoryPoint(zero,0,SimTime(1),CompensationDirection::NegativeY).mY,zero[0].mY,"zero dt unchanged");
    zero[1].mTime = zero[0].mTime - SimTime(0.1);
    require(trajectorySegmentSpeed(zero,0)==0,"negative dt unavailable");
    auto x = t;
    for (std::size_t i=0;i<x.size();++i) { x[i].mX=27*0.25*i; x[i].mY=5; }
    near(compensateTrajectoryPoint(x,0,SimTime(0.1),CompensationDirection::PositiveX).mX,2.7,"positive X supported");
    near(compensateTrajectoryPoint(x,0,SimTime(0.1),CompensationDirection::NegativeY).mY,5,"unsupported direction unchanged");
    auto curve=t; curve[1].mX+=2;
    auto corrected=compensateTrajectoryPoint(curve,1,SimTime(0.1),CompensationDirection::NegativeY);
    near(corrected.mX,curve[1].mX,"curved/lateral: no arbitrary lateral correction");
    near(curve[1].mY-corrected.mY,trajectorySegmentSpeed(curve,1)*0.1,"curve uses documented scalar segment speed");
}
}

namespace artery { namespace mcm {
struct McApplicationTimingTestAccess {
static void run() {
    // Regression: the planner's explicit found flag gates merging responses,
    // even when fallback content and a nonempty failed-result trajectory exist.
    McApplication merging;
    McEgoContext mergeEgo; mergeEgo.plannedTrajectory=trajectory();
    merging.updateEgoContext(mergeEgo);
    merging.mCvSelectedTrajectory=trajectory(); merging.mHasCvSelectedTrajectory=true;
    ReceivedMcm mergeRequest; mergeRequest.data.requestedTrajectory=trajectory();
    mergeRequest.data.numberOfVehicles=2;
    TrajectoryPlanner::CvTrajectoryDecisionResult candidate;
    candidate.found=false; candidate.trajectory=trajectory();
    PendingMcmCommand mergeResponse;
    merging.buildCvMergingResponse(mergeResponse,mergeRequest,candidate.found);
    require(mergeResponse.subtype==mcmSubtype::Reject,"infeasible merging must Reject, never fallback Offer");
    require(!mergeResponse.hasOfferedTrajectory && mergeResponse.offeredTrajectory.empty(),"failed merging response has no fallback offer");
    require(merging.mCoordinationProgressCV==coordinationProgressCV::SendReject,"merging uses existing Reject state");
    candidate.found=true; candidate.trajectory=trajectory(10,50);
    merging.mCvSelectedTrajectory=candidate.trajectory; merging.mHasCvSelectedTrajectory=true;
    merging.buildCvMergingResponse(mergeResponse,mergeRequest,candidate.found);
    require(mergeResponse.subtype==mcmSubtype::Offer,"feasible merging still Offers");
    require(sameMcmProposal(mergeResponse.offeredTrajectory,candidate.trajectory),"Offer uses feasible planner candidate");
    mergeRequest.data.numberOfVehicles=1;
    merging.buildCvMergingResponse(mergeResponse,mergeRequest,false);
    require(mergeResponse.subtype==mcmSubtype::Reject,"infeasible one-CV merging also Rejects");
    McApplication rv;
    McEgoContext ego; ego.now=SimTime(10); ego.sampledAt=SimTime(10); ego.stationId=1;
    ego.plannedTrajectory=trajectory(); rv.updateEgoContext(ego);
    rv.mMergingRequestQueuedOrSent=true;
    rv.mCooperatingVehicleType=cooperatingVehicleType::RV;
    rv.mRvRequestId=7; rv.mRvNumberOfVehicles=2; rv.mRvTargetVehicle1=2; rv.mRvTargetVehicle2=3;
    PendingMcmCommand request; request.requestId=7; request.requestedTrajectory=ego.plannedTrajectory;
    rv.mPendingMcmCommand=request;
    for (int i=1;i<=5;++i) {
        ego.now=SimTime(10+0.1*i); ego.sampledAt=ego.now;
        ego.plannedTrajectory=trajectory(ego.now.dbl(),100-2.7*i);
        rv.updateEgoContext(ego); rv.refreshLiveNegotiationCandidates();
        require(sameMcmProposal(rv.plannedTrajectory(),ego.plannedTrajectory),"intention refreshed each tick");
        require(sameMcmProposal(rv.mRvRequestedTrajectory,ego.plannedTrajectory),"deferred first Request current");
        require(sameMcmProposal(rv.mPendingMcmCommand->requestedTrajectory,ego.plannedTrajectory),"DCC command and timestamp agree");
    }
    auto issued = rv.mRvRequestedTrajectory;
    rv.mRvProposalIssued=true; rv.mPendingMcmCommand.reset();
    rv.mRvRequestCommand=request; rv.mRvRequestCommand->requestedTrajectory=issued;
    ego.now=SimTime(10.6); ego.sampledAt=ego.now; ego.plannedTrajectory=trajectory(10.6,80);
    rv.updateEgoContext(ego); rv.refreshLiveNegotiationCandidates();
    require(sameMcmProposal(rv.mRvLiveRequestedTrajectory,ego.plannedTrajectory),"moving RV live candidate updates without response");
    require(sameMcmProposal(rv.mRvRequestedTrajectory,issued),"issued Request retained");
    auto requestRetry=rv.makeRvRequestRetryCommand();
    require(requestRetry.retransmission,"issued Request requests complete packet retransmission");
    require(requestRetry.cooperationType==request.cooperationType,"Request retry preserves issued protocol fields");
    require(sameMcmProposal(requestRetry.requestedTrajectory,issued),"retry retains issued Request");
    require(sameMcmProposal(rv.makeRvFollowupCommand(mcmSubtype::Confirm).requestedTrajectory,issued),"initial Confirm retains proposal");
    require(sameMcmProposal(rv.makeRvConfirmRetryCommand().requestedTrajectory,issued),"Confirm retry same proposal");
    rv.mRvNegotiatedTrajectory=issued; rv.mHasRvNegotiatedTrajectory=true;
    rv.mRvConfirmQueuedOrSent=true;
    rv.refreshLiveNegotiationCandidates();
    require(sameMcmProposal(rv.mRvNegotiatedTrajectory,issued),"confirmed agreement unchanged");
    // Both CVs' responses must echo the SAME proposal; stale origins under a
    // reused ID are rejected even when sender and target membership match.
    McmSnapshot response; response.hasNegotiationContainer=true;
    response.mcmCategory=static_cast<long>(mcmSubtype::Offer); response.requestId=7;
    require(rv.isNegotiationMessageForActiveRequest(response,mcmSubtype::Offer,7),"active ID guard");
    require(!sameMcmProposal(trajectory(9),issued),"old response origin rejected");
    require(!rv.isNegotiationMessageForActiveRequest(response,mcmSubtype::Offer,8),"old ID rejected");
    rv.mRvOfferReceived1=true; rv.mRvAcceptReceived2=true;
    rv.resetRvResponseTracking();
    require(!rv.mRvOfferReceived1 && !rv.mRvAcceptReceived2,"replacement invalidates all participant responses");
    // Full multi-CV handler guards: reject an Accept that changes a CV Offer.
    VehicleDataProvider rvVdp(1); rv.mVehicleDataProvider=&rvVdp;
    rv.mRvConfirmQueuedOrSent=false; rv.mCoordinationProgressRV=coordinationProgressRV::RequestSent;
    rv.mPendingMcmCommand.reset();
    ReceivedMcm offer1; offer1.data=response; offer1.data.requestedTrajectory=issued;
    offer1.data.stationId=2; offer1.data.offeredTrajectory=trajectory(10.5,50);
    ReceivedMcm offer2=offer1; offer2.data.stationId=3; offer2.data.offeredTrajectory=trajectory(10.5,20);
    rv.handleReceivedOfferAsRv(offer1);
    require(rv.mRvOfferReceived1 && !rv.mRvOfferReceived2,"first CV isolated");
    auto oldOffer=offer2; oldOffer.data.requestedTrajectory=trajectory(9);
    rv.handleReceivedOfferAsRv(oldOffer);
    require(!rv.mRvOfferReceived2,"second CV old proposal excluded");
    rv.handleReceivedOfferAsRv(offer2);
    require(rv.mRvConfirmQueuedOrSent,"all current CV Offers allow Confirm");
    rv.mPendingMcmCommand.reset();
    auto accept1=offer1; accept1.data.mcmCategory=static_cast<long>(mcmSubtype::Accept);
    auto corruptAccept=accept1; corruptAccept.data.offeredTrajectory[0].mY+=5;
    rv.handleReceivedAcceptAsRv(corruptAccept);
    require(!rv.mRvAcceptReceived1,"Accept cannot replace issued CV Offer");
    rv.handleReceivedAcceptAsRv(accept1);
    require(rv.mRvAcceptReceived1 && !rv.mRvAcceptReceived2,"Accepts tracked per CV");
    auto accept2=offer2; accept2.data.mcmCategory=static_cast<long>(mcmSubtype::Accept);
    rv.handleReceivedAcceptAsRv(accept2);
    require(rv.mRvExecuteQueuedOrSent,"matching agreement enters execution");
    require(sameMcmProposal(rv.mRvNegotiatedTrajectory,issued),"execution freezes agreed RV proposal");
    McApplication cv;
    cv.updateEgoContext(ego); cv.mCooperatingVehicleType=cooperatingVehicleType::CV;
    cv.mCvRvStationId=1; cv.mCvRequestId=7;
    ReceivedMcm received; received.data.requestedTrajectory=issued;
    received.generatedAt=issued.front().mReferenceTime;
    cv.mCvActiveRequest=received;
    PendingMcmCommand offer; offer.subtype=mcmSubtype::Offer; offer.requestId=7;
    offer.requestedTrajectory=issued; offer.offeredTrajectory=trajectory(10.5,50); offer.hasOfferedTrajectory=true;
    cv.mCvOfferCommand=offer; cv.mCvSelectedTrajectory=offer.offeredTrajectory; cv.mHasCvSelectedTrajectory=true;
    cv.mCvProposalIssued=true; cv.mPendingMcmCommand=offer;
    ego.plannedTrajectory=trajectory(10.7,60); cv.updateEgoContext(ego); cv.refreshLiveNegotiationCandidates();
    auto retried=cv.makeCvOfferRetryCommand();
    require(retried.retransmission,"issued Offer requests complete packet retransmission");
    require(sameMcmProposal(retried.requestedTrajectory,issued),"Offer retry retains RV echo");
    require(sameMcmProposal(retried.offeredTrajectory,offer.offeredTrajectory),"waiting Confirm does not mutate Offer");
    auto accept=cv.makeCvAcceptRetryCommand();
    require(sameMcmProposal(accept.requestedTrajectory,issued),"Accept echoes Request");
    require(sameMcmProposal(accept.offeredTrajectory,offer.offeredTrajectory),"Accept retains CV offer");
    // CV version replacement and out-of-order delivery, including two versions
    // planned from the same sampled state after a rejection.
    VehicleDataProvider vdp(2); cv.mVehicleDataProvider=&vdp;
    received.data.hasNegotiationContainer=true; received.data.mcmCategory=0;
    received.data.stationId=1; received.data.requestId=7; received.data.negotiationVehicleId1=2;
    cv.mCvActiveRequest=received;
    cv.mCoordinationProgressCV=coordinationProgressCV::SendOffer;
    cv.mPendingMcmCommand.reset();
    auto confirm=received; confirm.data.mcmCategory=static_cast<long>(mcmSubtype::Confirm);
    confirm.data.numberOfVehicles=2; confirm.data.cooperationTypeMcm=0;
    cv.handleReceivedConfirmAsCv(confirm);
    require(cv.mPendingMcmCommand && cv.mPendingMcmCommand->subtype==mcmSubtype::Accept,
        "matching Confirm queues agreement Accept");
    require(cv.makeCvAcceptRetryCommand().numberOfVehicles==cv.mPendingMcmCommand->numberOfVehicles,
        "Accept retry preserves original response participant count");
    McApplication deferred=cv;
    deferred.mCvProposalIssued=false;
    deferred.mCvActiveRequest->data.numberOfVehicles=2;
    deferred.mCvNegotiationStartedAt=SimTime(10.5);
    deferred.mHasCvNegotiationStartedAt=true;
    ego.sampledAt=SimTime(10.7); ego.now=SimTime(10.75);
    ego.plannedTrajectory=trajectory(10.75,60);
    deferred.updateEgoContext(ego);
    deferred.refreshLiveNegotiationCandidates();
    require(deferred.mPendingMcmCommand && deferred.mPendingMcmCommand->hasOfferedTrajectory,
        "DCC-deferred CV response re-evaluated");
    require(sameMcmProposal(deferred.mPendingMcmCommand->offeredTrajectory,ego.plannedTrajectory),
        "unissued CV Offer rebuilt from current state");
    require(deferred.mCvNegotiationStartedAt==SimTime(10.5),"DCC refresh retains deadline");
    require(cv.acceptRequestVersion(received),"duplicate retained Request accepted");
    auto restamped=received; restamped.generatedAt+=SimTime(0.1);
    require(!cv.acceptRequestVersion(restamped),"unchanged issued Request cannot acquire a new generation timestamp");
    auto changed=received; changed.data.requestedTrajectory[0].mX+=5;
    require(!cv.acceptRequestVersion(changed),"same ID changed content rejected");
    auto replacement=received; replacement.data.requestId=8;
    require(cv.acceptRequestVersion(replacement),"new proposal same sample time accepted");
    require(!cv.mCvActiveRequest && !cv.mPendingMcmCommand && !cv.mCvProposalIssued,"new Request invalidates old CV chain");
    require(!cv.acceptRequestVersion(received),"retired Request cannot return via reordering");
    const auto reserved=rv.makeRequestId(ego.now);
    rv.mIssuedRequestIds[reserved]=ego.now;
    require(rv.makeRequestId(ego.now)!=reserved,"existing Request ID not reused while old Execute can arrive");
    rv.mEgoContext.now=SimTime(32.7);
    const auto futureSeed=SimTime(32.8);
    const auto preferred=rv.makeRequestId(futureSeed);
    rv.mIssuedRequestIds[preferred]=SimTime(0);
    require(rv.makeRequestId(futureSeed)!=preferred,"future alternative-ID seed cannot shorten actual holdoff");
    rv.mPendingMcmCommand=request;
    rv.resetRvNegotiationAfterTimeout();
    require(!rv.mPendingMcmCommand,"timeout invalidates DCC-deferred command");
}
};
} }

int main(int argc, char** argv) {
    try {
        omnetpp::SimTime::setScaleExp(-12);
        omnetpp::cSimulation sim("mcm timing tests",new omnetpp::cNullEnvir(argc,argv,nullptr));
        omnetpp::cSimulation::setActiveSimulation(&sim);
        struct ActiveSimulationGuard { ~ActiveSimulationGuard() { omnetpp::cSimulation::setActiveSimulation(nullptr); } } guard;
        Timer timer; timer.setTimebase("2020-01-01 00:00:00");
        runtimeRoundTrip(sim,timer);
        if(argc>1 && std::string(argv[1])=="--runtime-roundtrip") {
            std::cout << "Runtime MCM byte round trips and decoded conflict/TTC checks passed\n";
            return 0;
        }
        ageAndTimestamps(sim,timer);
        retransmissions(sim,timer);
        compensation();
        McApplicationTimingTestAccess::run();
        omnetpp::cSimulation::setActiveSimulation(nullptr);
        std::cout << "MCM timing, compensation, freshness and proposal tests passed\n";
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n'; return 1;
    }
}
