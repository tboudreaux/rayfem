#import <AppKit/AppKit.h>

#include "ui/mouse_modifiers.hpp"

namespace RayFEM::UI {
namespace {
id monitor = nil;
NSWindow* boundWindow = nil;
bool hasPress = false;
bool pressShift = false;

bool NativeSelectionShift() {
    return hasPress ? pressShift : detail::PhysicalSelectionShift();
}
}

void ShutdownMouseModifiers() {
    if (monitor) [NSEvent removeMonitor:monitor];
    monitor = nil;
    boundWindow = nil;
    hasPress = false;
    pressShift = false;
    detail::mouseSelectionShiftGetter = nullptr;
}

void InitMouseModifiers() {
    ShutdownMouseModifiers();
    boundWindow = static_cast<NSWindow*>(GetWindowHandle());
    if (!boundWindow) return;
    // raylib's GLFW mouse callback discards event modifier flags. Accessibility
    // input can carry Shift on the mouse event without any separate key event,
    // so capture it locally while leaving normal window dispatch untouched.
    monitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskLeftMouseDown
        handler:^NSEvent* (NSEvent* event) {
            if ([event window] == boundWindow) {
                hasPress = true;
                pressShift = ([event modifierFlags] & NSEventModifierFlagShift) != 0;
            }
            return event;
        }];
    if (monitor) detail::mouseSelectionShiftGetter = NativeSelectionShift;
}
}
