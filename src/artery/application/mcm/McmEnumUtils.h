#ifndef ARTERY_APPLICATION_MCM_MCMENUMUTILS_H_
#define ARTERY_APPLICATION_MCM_MCMENUMUTILS_H_

#include "artery/application/mcm/McApplication.h"

namespace artery
{
namespace mcm
{

// Shared, behavior-preserving domain enum display and mapping utilities.
// These helpers are intentionally separate from ASN.1 serialization mappings.

inline constexpr const char* priorityName(long priority)
{
    if (priority == static_cast<long>(priorityMcmCategory::LowPriority)) {
        return "LowPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::MediumPriority)) {
        return "MediumPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::HighPriority)) {
        return "HighPriority";
    }
    if (priority == static_cast<long>(priorityMcmCategory::EmergencyPriority)) {
        return "EmergencyPriority";
    }
    return "NoPriority";
}

inline constexpr const char* operationModeName(operationMode mode)
{
    switch (mode) {
        case operationMode::IntentionSharingMode: return "IntentionSharingMode";
        case operationMode::ManeuverNegotiationMode: return "ManeuverNegotiationMode";
        case operationMode::ManeuverExecutionMode: return "ManeuverExecutionMode";
        default: return "Unknown";
    }
}

inline constexpr const char* controlManeuverName(controlManeuver maneuver)
{
    switch (maneuver) {
        case controlManeuver::Decelerate: return "Decelerate";
        case controlManeuver::Accelerate: return "Accelerate";
        case controlManeuver::ChangeLane: return "ChangeLane";
        case controlManeuver::LaneChangeExecution: return "LaneChangeExecution";
        case controlManeuver::EmergencyDeceleration: return "EmergencyDeceleration";
        case controlManeuver::DoNothing:
        default:
            return "DoNothing";
    }
}

inline constexpr double costThresholdForPriority(priorityMcmCategory priority)
{
    switch (priority) {
        case priorityMcmCategory::LowPriority: return 0.20;
        case priorityMcmCategory::MediumPriority: return 0.40;
        case priorityMcmCategory::HighPriority: return 0.60;
        case priorityMcmCategory::EmergencyPriority: return 0.80;
        case priorityMcmCategory::NoPriority: return 0.0;
    }

    return 0.0;
}

inline constexpr int priorityLevel(priorityMcmCategory priority)
{
    switch (priority) {
        case priorityMcmCategory::LowPriority: return 0;
        case priorityMcmCategory::MediumPriority: return 1;
        case priorityMcmCategory::HighPriority: return 2;
        case priorityMcmCategory::EmergencyPriority: return 3;
        case priorityMcmCategory::NoPriority: return -1;
    }

    return -1;
}

} // namespace mcm
} // namespace artery

#endif
