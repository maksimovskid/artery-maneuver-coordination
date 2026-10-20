#ifndef ARTERY_MCM_OPERATING_MODE_H
#define ARTERY_MCM_OPERATING_MODE_H
#include "artery/application/mcm/McApplication.h"
namespace artery { namespace mcm {
// Existing currentMCSoperatingMode double signal encoding, shared with CAM.
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
inline operationMode operatingModeFromMeasurement(double value)
{
    for (auto mode : {operationMode::IntentionSharingMode, operationMode::ManeuverNegotiationMode,
            operationMode::ManeuverExecutionMode})
        if (operatingModeMeasurementValue(mode) == value) return mode;
    return operationMode::Unknown;
}
}}
#endif
