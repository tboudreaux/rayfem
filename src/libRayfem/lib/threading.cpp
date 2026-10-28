#include "libRayfem/threading.hpp"

namespace RayFEM {
std::mutex& FiniteElementConstructionMutex() {
    static std::mutex mutex;
    return mutex;
}
} // namespace RayFEM
