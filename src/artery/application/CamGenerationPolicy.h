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

class CamGenerationPolicy {
public:
    struct Config {
        bool observed = false;
        bool own = false;
        bool intent = false;
        bool mco = false;
    } config;
    struct Hold {
        bool active = false;
        omnetpp::SimTime until;
    } observed, background;
    CamBaselinePolicy baseline = CamBaselinePolicy::SameAsCAM;
    bool own = false;

    bool enabled() const { return config.observed || config.own || config.intent || config.mco; }
    unsigned reasons() const { return (own ? 1u : 0u) | (observed.active ? 2u : 0u) |
        (background.active ? (config.mco ? 8u : 4u) : 0u); }

    // draw is supplied by the simulation module (uniform on its OMNeT++ RNG).
    template<class Draw> void update(omnetpp::SimTime now, double cbr,
        bool negotiating, bool seesNegotiation, Draw draw)
    {
        own = config.own && negotiating;
        const auto recover = [&](Hold& h) {
            if (h.active && now > h.until && cbr < 0.50) h.active = false;
        };
        recover(observed);
        recover(background);
        if (config.observed && !observed.active && seesNegotiation && cbr >= 0.50) {
            observed.active = true;
            observed.until = now + omnetpp::SimTime(cbr < 0.65 ? draw(0.5, 1.5) : draw(1.0, 2.0));
        }
        // Observed negotiation wins new background entry, as in the reference.
        // A background hold already active remains independent of other reasons.
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
    bool periodicDue(omnetpp::SimTime elapsed, omnetpp::SimTime facilitiesInterval) const {
        return elapsed >= std::max(periodicInterval(), facilitiesInterval);
    }
};
} // namespace artery
#endif
