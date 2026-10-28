#pragma once

#include <mutex>

namespace RayFEM {
// MFEM shares 1D basis tables between all finite element collections
// (mfem::poly1d) and fills them lazily while collections, spaces and meshes
// with nodes are constructed. Hold this lock around such construction when
// another thread may construct them at the same time. Evaluating existing
// elements does not need it.
std::mutex& FiniteElementConstructionMutex();
} // namespace RayFEM
