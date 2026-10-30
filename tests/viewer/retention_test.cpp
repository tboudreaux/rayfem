#include "pipeline/retention.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

using namespace RayFEM::UI;

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Frame { uint64_t order; std::string tag; bool pinned = false; };

// Stream count frames through a history with the policy; returns what stays.
std::vector<Frame> Run(const RetentionPolicy& policy, int count, auto tagOf) {
    std::vector<Frame> history;
    for (int i = 0; i < count; ++i) {
        history.push_back({static_cast<uint64_t>(i), tagOf(i)});
        while (static_cast<int>(history.size()) > policy.max_frames) {
            std::vector<RetentionCandidate> candidates;
            for (const auto& f : history) candidates.push_back({f.order, f.pinned, policy.protect_tags.count(f.tag) > 0});
            const auto victim = ChooseEviction(candidates, policy);
            if (!victim) break;
            history.erase(history.begin() + static_cast<long>(*victim));
        }
    }
    return history;
}

void Oldest() {
    RetentionPolicy policy;
    std::vector<RetentionCandidate> frames{{0}, {1}, {2}};
    Check(ChooseEviction(frames, policy) == 0, "The oldest frame goes first");
    frames[0].keep = true;
    Check(ChooseEviction(frames, policy) == 1, "Kept frames are skipped");
    for (auto& f : frames) f.keep = true;
    Check(!ChooseEviction(frames, policy), "Nothing goes when every frame must stay");
    Check(!ChooseEviction({}, policy), "An empty history has nothing to evict");
}

void Protection() {
    RetentionPolicy policy;
    std::vector<RetentionCandidate> frames{{0, false, true}, {1, false, false}, {2, false, true}, {3, false, false}};
    Check(ChooseEviction(frames, policy) == 1, "Unprotected frames go before protected ones");
    frames[1].keep = frames[3].keep = true;
    Check(ChooseEviction(frames, policy) == 0, "Protected frames go once no unprotected frame can");

    // Converged and accepted frames: protect accepted, so converged frames
    // are evicted once the history is full.
    policy.max_frames = 10;
    policy.protect_tags = {"accepted"};
    auto kept = Run(policy, 40, [](int i) { return std::string(i % 2 ? "converged" : "accepted"); });
    int accepted = 0;
    for (const auto& f : kept) accepted += f.tag == "accepted";
    Check(kept.size() == 10 && accepted == 10, "Protected accepted frames fill the history: " + std::to_string(accepted));

    // Evict everything neither accepted nor converged first.
    policy.protect_tags = {"accepted", "converged"};
    kept = Run(policy, 60, [](int i) { return std::string(i % 3 == 0 ? "accepted" : i % 3 == 1 ? "converged" : "iterate"); });
    for (const auto& f : kept) Check(f.tag != "iterate", "Iterates go first");
}

void Thinning() {
    RetentionPolicy policy;
    policy.max_frames = 20;
    policy.strategy = RetentionPolicy::Strategy::Thin;
    policy.thin_every = 2;
    const auto kept = Run(policy, 200, [](int) { return std::string(); });
    Check(kept.size() == 20, "The history holds its capacity");
    // The newest half of the capacity is dense.
    for (int i = 0; i < 10; ++i) Check(kept[kept.size() - 1 - i].order == static_cast<uint64_t>(199 - i), "Recent frames stay dense");
    // Older frames thin out but still reach back to the start of the run.
    Check(kept.front().order == 0, "The first frame survives thinning");
    for (size_t i = 1; i + 10 < kept.size(); ++i)
        Check(kept[i].order % 2 == 0 && kept[i].order > kept[i - 1].order, "Older frames are a thinned, ordered subset");
    Check(kept[kept.size() - 11].order - kept[1].order > 40, "Thinned frames span the run");
    policy.thin_every = 1;  // not a valid stride: behaves like Oldest
    Check(Run(policy, 30, [](int) { return std::string(); }).front().order == 10, "A stride below 2 evicts the oldest");
}
} // namespace

int main() {
    try {
        Oldest();
        Protection();
        Thinning();
        std::cout << "retention passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "retention test failed: " << e.what() << '\n';
        return 1;
    }
}
