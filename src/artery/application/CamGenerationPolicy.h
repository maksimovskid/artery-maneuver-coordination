#ifndef ARTERY_CAM_GENERATION_POLICY_H
#define ARTERY_CAM_GENERATION_POLICY_H

#include <omnetpp/simtime.h>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace artery {

// Configured policy never changes when a temporary reduction starts or ends.
enum class CamBaselinePolicy { SameAsCAM, Fixed1Hz, Fixed2Hz };
inline CamBaselinePolicy parseCamBaselinePolicy(const std::string& name)
{
    if (name == "SameAsCAM") return CamBaselinePolicy::SameAsCAM;
    if (name == "PeriodicFixed1Hz") return CamBaselinePolicy::Fixed1Hz;
    if (name == "PeriodicFixed2Hz") return CamBaselinePolicy::Fixed2Hz;
    throw std::invalid_argument("Unknown CAM triggeringCondition: " + name);
}

/**
 * Facilities generation policy, separated from CaService's inputs and sending.
 * Config/baseline are set at initialization; update() changes only temporary
 * reasons. Zero periodicInterval() delegates to the existing dynamics/fixedRate
 * path. Recovery restores the configured baseline, never a hard-coded policy:
 * Dynamics -> temporary 1 Hz -> Dynamics; Fixed2Hz -> 1 Hz -> Fixed2Hz.
 *
 * This policy neither interprets MCM priority nor generates trajectories.
 * LOW/MEDIUM/HIGH are MCM planning priorities; Emergency is safety notification,
 * not an additional cooperative search or CAM-reduction tier.
 */
class CamGenerationPolicy {
public:
    // All opt-in defaults are false. "observed" is the newGenCamRules master;
    // Intent requires observed && intent. MCO is independent of that master.
    struct Config {
        bool observed = false;
        bool own = false;
        bool intent = false;
        bool mco = false;
    } config;
    // Minimum RECOVERY hold, not CAM interval, packet backoff or MAC delay.
    // Explicit validity permits entry at simTime()==0; until may then be >0
    // without relying on a fragile startTime>0 sentinel.
    struct Hold {
        bool active = false;
        omnetpp::SimTime until;
    } observed, background;
    CamBaselinePolicy baseline = CamBaselinePolicy::SameAsCAM;
    bool own = false;

    // Any active reason requests 1 Hz; ending one cannot cancel another.
    // Intent and MCO share one background hold because MCO supersedes Intent.
    // enabled() guards processing only, not individual rule eligibility.
    bool enabled() const { return config.observed || config.own || config.intent || config.mco; }
    unsigned reasons() const { return (own ? 1u : 0u) | (observed.active ? 2u : 0u) |
        (background.active ? (config.mco ? 8u : 4u) : 0u); }

    // draw is supplied by CaService::uniform() on its OMNeT++ simulation RNG.
    // Each timed state draws once on entry/re-entry; scans and CBR-band changes
    // do not redraw. Recovery requires BOTH now>until AND CBR<.50, so congestion
    // can keep a reason active after the minimum hold has elapsed.
    template<class Draw> void update(omnetpp::SimTime now, double cbr,
        bool negotiating, bool seesNegotiation, Draw draw)
    {
        own = config.own && negotiating;
        const auto recover = [&](Hold& h) {
            if (h.active && now > h.until && cbr < 0.50) h.active = false;
        };
        recover(observed);
        recover(background);
        // Reference boundaries are intentionally asymmetric: observed includes
        // .50 and puts .65 in HIGH; Intent excludes .50 and puts .65 in MODERATE.
        // Eligible awareness is supplied by CamMcmAwareness, independently of
        // own negotiation. Observed holds: moderate U(.5,1.5), high U(1,2) s.
        if (config.observed && !observed.active && seesNegotiation && cbr >= 0.50) {
            observed.active = true;
            observed.until = now + omnetpp::SimTime(cbr < 0.65 ? draw(0.5, 1.5) : draw(1.0, 2.0));
        }
        // Observed negotiation wins new background entry, as in the reference.
        // A background hold already active remains independent of other reasons.
        // New background entry requires SameAsCAM and no own/observed reason.
        // MCO is independent, CBR>.50, U(.1,1) s; it supersedes Intent.
        // Historical "Intent" is congestion-only, not an MCM-intention test:
        // master && intent && !mco; .50<CBR<=.65 gives U(1,2), >.65 U(1,5) s.
        if (!background.active && !observed.active && !own && baseline == CamBaselinePolicy::SameAsCAM) {
            if (config.mco && cbr > 0.50) {
                background.active = true;
                background.until = now + omnetpp::SimTime(draw(0.1, 1.0));
            } else if (!config.mco && config.observed && config.intent && cbr > 0.50) {
                background.active = true;
                background.until = now + omnetpp::SimTime(cbr > 0.65 ? draw(1.0, 5.0) : draw(1.0, 2.0));
            }
        }
    }

    // Zero means use the original dynamics/fixedRate code, unchanged.
    omnetpp::SimTime periodicInterval() const {
        omnetpp::SimTime interval;
        if (baseline == CamBaselinePolicy::Fixed1Hz) interval = 1;
        else if (baseline == CamBaselinePolicy::Fixed2Hz) interval = 0.5;
        if (reasons()) interval = std::max(interval, omnetpp::SimTime(1));
        return interval;
    }
    // Facilities restriction composes with the effective period via max().
    // With restriction off, CaService passes minInterval instead. The caller
    // retains its existing DCC clamp; access DCC and radio airtime are separate.
    bool periodicDue(omnetpp::SimTime elapsed, omnetpp::SimTime facilitiesInterval) const {
        return elapsed >= std::max(periodicInterval(), facilitiesInterval);
    }
};
} // namespace artery
#endif
