#ifndef ARTERY_MCM_TRAJECTORY_H
#define ARTERY_MCM_TRAJECTORY_H
#include "artery/application/mcm/TrajPointMCM.h"
#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace artery { namespace mcm {
using McmTrajectory = std::vector<TrajPointMCM>;
inline bool sameMcmProposal(const McmTrajectory& a, const McmTrajectory& b)
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::lround(a[i].mX) != std::lround(b[i].mX) ||
            std::lround(a[i].mY) != std::lround(b[i].mY) ||
            std::abs(std::remainder(a[i].mHeading - b[i].mHeading, 2 * M_PI)) > M_PI / 3600.0 + 1e-10 ||
            std::lround(a[i].mTime.dbl() * 100) != std::lround(b[i].mTime.dbl() * 100)) return false;
    }
    return true;
}
// A generation-time assertion for live predictions only. Agreement snapshots
// in Confirm/Accept and an Offer's RV echo intentionally retain their samples.
inline void requireGenerationReference(const McmTrajectory& trajectory, omnetpp::SimTime generatedAt)
{
    for (const auto& point : trajectory)
        if (point.mReferenceTime != generatedAt)
            throw std::logic_error("stale live trajectory at MCM generation");
}
inline uint64_t mcmProposalDigest(const McmTrajectory& trajectory)
{
    uint64_t hash = 14695981039346656037ULL;
    const auto mix = [&](int64_t value) {
        for (unsigned i = 0; i < 8; ++i) {
            hash ^= (static_cast<uint64_t>(value) >> (i * 8)) & 255;
            hash *= 1099511628211ULL;
        }
    };
    mix(trajectory.size());
    for (const auto& point : trajectory) {
        mix(std::lround(point.mX)); mix(std::lround(point.mY));
        mix(std::lround(std::remainder(point.mHeading, 2*M_PI) * 1800/M_PI));
        mix(std::lround(point.mTime.dbl() * 100));
    }
    return hash;
}
// Research model: local segment speed (metres/second) times total information
// age (seconds), applied along the scenario's longitudinal axis. This is NOT
// time-aligned interpolation and does not predict curvature or lateral motion.
enum class CompensationDirection { NegativeY, PositiveX };
inline double trajectorySegmentSpeed(const McmTrajectory& trajectory, std::size_t index)
{
    if (trajectory.size() < 2 || index >= trajectory.size()) return 0;
    const auto first = index == 0 ? 0 : index - 1;
    const auto second = index == 0 ? 1 : index;
    const auto dt = (trajectory[second].mTime - trajectory[first].mTime).dbl();
    if (dt <= 0) return 0;
    return std::hypot(trajectory[second].mX - trajectory[first].mX,
        trajectory[second].mY - trajectory[first].mY) / dt;
}
inline TrajPointMCM compensateTrajectoryPoint(const McmTrajectory& trajectory,
    std::size_t index, omnetpp::SimTime age, CompensationDirection direction)
{
    if (index >= trajectory.size()) return {};
    auto point = trajectory[index];
    if (trajectory.size() < 2 || age <= omnetpp::SimTime(0.05)) return point;
    const auto& last = trajectory.back();
    const auto& previous = trajectory[trajectory.size() - 2];
    const bool supported = direction == CompensationDirection::NegativeY ?
        last.mY < previous.mY : last.mX > previous.mX;
    if (!supported) return point;
    const double correction = trajectorySegmentSpeed(trajectory, index) * age.dbl();
    if (direction == CompensationDirection::NegativeY) point.mY -= correction;
    else point.mX += correction;
    return point;
}
} }
#endif
