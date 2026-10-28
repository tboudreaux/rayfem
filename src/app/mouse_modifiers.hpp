#pragma once

#include "raylib.h"

namespace RayFEM::UI {
namespace detail {
inline bool PhysicalSelectionShift() {
    return IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
}
// Optional native event source. Keeping this registration inline lets UI-only
// CPU tests and other platforms use the panel without a native object file.
inline bool (*mouseSelectionShiftGetter)() = nullptr;
}

// Query only for mouse selection: the native value is the modifier state at
// left-button press, retained through release, rather than current key state.
inline bool MouseSelectionShift() {
    return detail::mouseSelectionShiftGetter ? detail::mouseSelectionShiftGetter() :
                                             detail::PhysicalSelectionShift();
}

#if defined(__APPLE__)
void InitMouseModifiers();
void ShutdownMouseModifiers();
#else
inline void InitMouseModifiers() {}
inline void ShutdownMouseModifiers() {}
#endif
}
