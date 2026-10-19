#ifndef ARTERY_MCM_TIMING_H
#define ARTERY_MCM_TIMING_H
#include <cstdint>
#include <omnetpp/simtime.h>
#include <stdexcept>
namespace artery { namespace mcm {
// Match the millisecond precision of generationDeltaTime. Planning uses the
// latest state at generation; the reference is its representable header time.
inline omnetpp::SimTime generationReferenceTime(omnetpp::SimTime now)
{
    return omnetpp::SimTime(now.inUnit(omnetpp::SIMTIME_MS), omnetpp::SIMTIME_MS);
}
// Nearest epoch, half-range is deliberately ambiguous and rejected. A packet
// lifetime / proposal lifetime must be shorter than 32.768 s.
inline int64_t timestampOffsetMilliseconds(long raw, uint64_t taiNowMs)
{
    if (raw < 0 || raw > 65535) throw std::invalid_argument("MCM timestamp outside 0..65535");
    int64_t delta = raw - static_cast<int64_t>(taiNowMs & 0xffff);
    if (delta > 32768) delta -= 65536;
    if (delta < -32768) delta += 65536;
    if (delta == 32768 || delta == -32768) throw std::invalid_argument("ambiguous MCM timestamp epoch");
    if (delta > 0) throw std::invalid_argument("MCM timestamp is in the future");
    return delta;
}
} }
#endif
