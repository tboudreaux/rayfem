#include <mfem.hpp>
#include <type_traits>

// Checked at compile time, after any parent-provided MFEM generation target.
// A configure-time probe cannot inspect headers that do not exist yet.
#ifndef MFEM_USE_EXCEPTIONS
#error "RayFEM requires MFEM_USE_EXCEPTIONS for recoverable mesh/field decoding"
#endif
#if defined(MFEM_USE_OPENMP) && !defined(_OPENMP)
#error "MFEM_USE_OPENMP requires the compiler flags exported by the MFEM dependency"
#endif
static_assert(std::is_same_v<mfem::real_t, double>, "RayFEM requires double-precision MFEM");
