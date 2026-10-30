#pragma once

// Which retained frames go when the history is full. Pure policy: the viewer
// describes each frame (its arrival order, whether it must stay, whether it
// is protected) and evicts the one chosen, until the history fits.

#include <algorithm>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace RayFEM::UI {

struct RetentionPolicy {
    int max_frames = 1000;
    double max_mib = 2048;  // a conservative estimate of field and mesh memory
    enum class Strategy { Oldest, Thin };
    // Oldest evicts the oldest frame first. Thin keeps the newest half of the
    // capacity dense and thins older frames to every nth, then every n²th,
    // and so on, so the whole run stays represented ever more sparsely.
    Strategy strategy = Strategy::Oldest;
    int thin_every = 4;
    // Protected frames are evicted only after every unprotected one: frames
    // tagged with any of these, or, with protect_shown, frames the timeline
    // filter shows.
    std::set<std::string> protect_tags;
    bool protect_shown = false;
};

struct RetentionCandidate {
    uint64_t order = 0;      // arrival order; later frames have larger numbers
    bool keep = false;       // pinned, viewed while paused, or being exported
    bool protect = false;
};

// The index of the frame to evict, given the frames oldest first; none when
// every frame must stay.
inline std::optional<size_t> ChooseEviction(const std::vector<RetentionCandidate>& frames, const RetentionPolicy& policy) {
    for (const bool protectedClass : {false, true}) {
        std::vector<size_t> pool;
        for (size_t i = 0; i < frames.size(); ++i)
            if (!frames[i].keep && frames[i].protect == protectedClass) pool.push_back(i);
        if (pool.empty()) continue;
        if (policy.strategy == RetentionPolicy::Strategy::Thin && policy.thin_every >= 2) {
            // Older frames: all but the newest half of the capacity.
            const size_t dense = static_cast<size_t>(std::max(1, policy.max_frames / 2));
            const size_t older = frames.size() > dense ? frames.size() - dense : 0;
            uint64_t stride = static_cast<uint64_t>(policy.thin_every);
            uint64_t newest = 0;
            for (const auto& f : frames) newest = std::max(newest, f.order);
            while (true) {
                for (size_t i : pool)
                    if (i < older && frames[i].order % stride != 0) return i;
                if (stride > newest / static_cast<uint64_t>(policy.thin_every) + 1) break;
                stride *= static_cast<uint64_t>(policy.thin_every);
            }
        }
        return pool.front();
    }
    return std::nullopt;
}

} // namespace RayFEM::UI
