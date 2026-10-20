#include "artery/application/CamGenerationPolicy.h"
#include "artery/application/CamMcmAwareness.h"
#include "artery/application/mcm/McmOperatingMode.h"
#include <omnetpp.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace artery;
using omnetpp::SimTime;
namespace {
int checks = 0;
void check(bool ok, const char* message) { ++checks; if (!ok) throw std::runtime_error(message); }
struct Rng {
    int calls = 0;
    double low = 0, high = 0;
    double operator()(double a, double b) { ++calls; low=a; high=b; return (a+b)/2; }
};
void policies() {
    Rng rng;
    const auto draw = [&](double a, double b) { return rng(a,b); };
    CamGenerationPolicy p;
    for (bool own : {false,true}) for (bool observed : {false,true}) for (double cbr : {0.,.5,.65,1.}) {
        p.update(0,cbr,own,observed,draw);
        check(!p.reasons() && p.periodicInterval()==0,"all flags false leaves original generation path");
    }
    check(rng.calls==0,"disabled features consume no RNG");
    p.config.own=true; p.update(0,0,true,false,draw);
    check(p.own && p.periodicInterval()==1,"own negotiation enters at simTime zero");
    check(!p.periodicDue(.99,.1) && p.periodicDue(1,.1),"adaptive 1Hz with facilities DCC off/minimum");
    check(!p.periodicDue(1,1.2) && p.periodicDue(1.2,1.2),"adaptive interval respects larger DCC interval");
    p.update(.1,0,false,false,draw);
    check(!p.reasons() && p.periodicInterval()==0,"own exit restores configured dynamics");
    for (double cbr : {.49,.50,.649,.65,.9}) {
        p={}; p.config.observed=true; const auto before=rng.calls;
        p.update(0,cbr,false,true,draw);
        check(p.observed.active==(cbr>=.5),"observed CBR entry boundaries");
        if (cbr<.5) continue;
        check(rng.calls==before+1,"draw once on observed entry");
        check(rng.low==(cbr<.65?.5:1.) && rng.high==(cbr<.65?1.5:2.),"observed hold range");
        auto until=p.observed.until;
        p.update(.1,.9,false,true,draw);
        check(p.observed.until==until && rng.calls==before+1,"no hold redraw on scan or CBR escalation");
        p.update(3,.5,false,false,draw);
        check(p.observed.active,"expired hold at CBR .50 does not recover");
        p.update(3,.49,false,false,draw);
        check(!p.observed.active,"expired hold plus low CBR recovers");
        p.update(4,cbr,false,true,draw);
        check(rng.calls==before+2,"reentry draws fresh hold");
    }
    p={}; p.config.observed=true; p.config.own=true;
    p.update(0,.6,true,true,draw); p.update(.1,0,false,false,draw);
    check(!p.own && p.observed.active && p.periodicInterval()==1,"own exit cannot cancel observed hold");
    p.update(3,.1,true,false,draw);
    check(p.own && !p.observed.active && p.periodicInterval()==1,"observed recovery cannot cancel own reduction");
    for (bool master : {false,true}) for (double cbr : {.50,.65,.651}) {
        p={}; p.config.observed=master; p.config.intent=true; auto before=rng.calls;
        p.update(0,cbr,false,false,draw);
        check(p.background.active==(master && cbr>.5),"Intent requires master, uses strict .50 threshold");
        if (p.background.active) {
            check(rng.calls==before+1 && rng.low==1 && rng.high==(cbr>.65?5.:2.),"Intent .65 boundary and high hold");
            auto until=p.background.until;
            p.update(until,.49,false,false,draw);
            check(p.background.active,"legacy strict hold elapsed boundary retained");
            p.update(until+SimTime(.001),.49,false,false,draw);
            check(!p.background.active,"Intent hold recovery");
        }
    }
    p={}; p.config={true,false,true,true}; auto before=rng.calls;
    p.update(0,.8,false,false,draw);
    check(p.background.active && rng.low==.1 && rng.high==1 && rng.calls==before+1,"MCO supersedes Intent: one short hold");
    p.update(.1,.8,false,true,draw);
    check(p.observed.active && p.background.active,"observed may overlap existing MCO hold");
    p.update(.6,.49,false,false,draw);
    check(!p.background.active && p.observed.active,"background recovery cannot cancel observed hold");
    p={}; p.config.mco=true; p.update(0,.5,false,false,draw);
    check(!p.background.active,"MCO .50 does not enter");
    p.update(0,.501,false,false,draw);
    check(p.background.active,"MCO independent of master flag");
    for (auto baseline : {CamBaselinePolicy::Fixed1Hz,CamBaselinePolicy::Fixed2Hz}) {
        p={}; p.baseline=baseline; p.config.mco=true; p.update(0,.9,false,false,draw);
        check(!p.background.active,"background only applies to SameAsCAM baseline");
        check(p.periodicInterval()==(baseline==CamBaselinePolicy::Fixed1Hz?1.:.5),"configured periodic policy preserved");
        p.config.own=true; p.update(.1,.9,true,false,draw);
        check(p.periodicInterval()==1,"own negotiation temporarily reduces configured 2Hz");
        p.update(.2,.9,false,false,draw);
        check(p.periodicInterval()==(baseline==CamBaselinePolicy::Fixed1Hz?1.:.5),"baseline restored after reduction");
    }
    check(parseCamBaselinePolicy("SameAsCAM")==CamBaselinePolicy::SameAsCAM,"parse normal policy");
    check(parseCamBaselinePolicy("PeriodicFixed1Hz")==CamBaselinePolicy::Fixed1Hz,"parse 1Hz");
    check(parseCamBaselinePolicy("PeriodicFixed2Hz")==CamBaselinePolicy::Fixed2Hz,"parse 2Hz");
    bool invalid=false; try { parseCamBaselinePolicy("typo"); } catch (const std::invalid_argument&) { invalid=true; }
    check(invalid,"invalid baseline rejected explicitly");
    using namespace artery::mcm;
    for (auto mode : {operationMode::Unknown,operationMode::IntentionSharingMode,operationMode::ManeuverNegotiationMode,operationMode::ManeuverExecutionMode})
        check(operatingModeFromMeasurement(operatingModeMeasurementValue(mode))==mode,"current typed mode encoding round trip");
}
void awareness() {
    check(!camObservesNegotiation(nullptr,0,0),"absent MCM map is safe");
    LocalDynamicMapMCM::AwarenessEntries entries;
    vanetza::asn1::Mcm packet;
    auto* n=vanetza::asn1::allocate<ManeuverNegotiationContainer_t>();
    packet->mcm.mcmParameters.maneuverNegotiationContainer=n;
    n->negotiationVehicleID1=42;
    std::shared_ptr<const vanetza::asn1::Mcm> sharedPacket=std::make_shared<vanetza::asn1::Mcm>(std::move(packet));
    entries.emplace(7, LocalDynamicMapMCM::AwarenessEntry(McObject(sharedPacket),0,2));
    check(camObservesNegotiation(&entries,0,0),"absent optional ID2 does not exclude station zero");
    check(!camObservesNegotiation(&entries,42,0),"ID1 excluded");
    check(camObservesNegotiation(&entries,7,0),"no invented sender exclusion");
    check(camObservesNegotiation(&entries,1,2),"expiry boundary matches LDM");
    check(!camObservesNegotiation(&entries,1,2.001),"stale negotiation inactive even before dropExpired");
    n->negotiationVehicleID2=vanetza::asn1::allocate<StationID_t>(); *n->negotiationVehicleID2=99;
    entries.clear(); entries.emplace(7,LocalDynamicMapMCM::AwarenessEntry(McObject(sharedPacket),0,2));
    check(!camObservesNegotiation(&entries,99,0),"present optional ID2 excluded");
    vanetza::asn1::Mcm intent;
    entries.clear(); entries.emplace(7,LocalDynamicMapMCM::AwarenessEntry(McObject(std::move(intent)),0,2));
    check(!camObservesNegotiation(&entries,1,0),"latest intent supersedes negotiation");
}
}
int main() {
    try { SimTime::setScaleExp(-12); policies(); awareness();
        std::cout << "CAM policy/awareness: " << checks << " checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
