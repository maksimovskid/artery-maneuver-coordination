#ifndef ARTERY_MCM_OPERATING_MODE_H
#define ARTERY_MCM_OPERATING_MODE_H
#include "artery/application/mcm/McApplication.h"
namespace artery { namespace mcm {
// Adapter for McService's existing currentMCSoperatingMode double signal.
// The single enum definition stays in McApplication.h; do not duplicate it or
// depend on legacy McmService. Signal values remain unknown=0, intention=1,
// negotiation=2, execution=3. This is a measurement encoding, not a new MCM
// wire field. McService emits mode changes; CaService observes/decodes them to
// its typed state (Unknown before the first update), without controlling MCM.
// Maneuver priority, including Emergency notification, is a separate concept.
inline constexpr const char* operatingModeSignalName() { return "currentMCSoperatingMode"; }
inline double operatingModeMeasurementValue(operationMode mode)
{
    switch (mode) {
        case operationMode::IntentionSharingMode: return 1.0;
        case operationMode::ManeuverNegotiationMode: return 2.0;
        case operationMode::ManeuverExecutionMode: return 3.0;
        default: return 0.0;
    }
}
// Unrecognized measurements map safely to Unknown, never to negotiation.
inline operationMode operatingModeFromMeasurement(double value)
{
    for (auto mode : {operationMode::IntentionSharingMode, operationMode::ManeuverNegotiationMode,
            operationMode::ManeuverExecutionMode})
        if (operatingModeMeasurementValue(mode) == value) return mode;
    return operationMode::Unknown;
}
}}
#endif
