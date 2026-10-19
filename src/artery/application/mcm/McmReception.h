#ifndef ARTERY_MCM_RECEPTION_H
#define ARTERY_MCM_RECEPTION_H
#include "artery/application/mcm/McApplication.h"
#include <vanetza/asn1/mcm.hpp>
namespace artery {
class Timer;
namespace mcm {
// Throws invalid_argument on malformed, ambiguous or genuinely future timing.
ReceivedMcm decodeReceivedMcm(const MCM_t&, omnetpp::SimTime now, const Timer&);
} }
#endif
