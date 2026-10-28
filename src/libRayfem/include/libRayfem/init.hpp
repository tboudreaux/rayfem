#pragma once

#include <string>

namespace RayFEM {

struct WindowParams {
    std::string host = "127.0.0.1";
    int port = 5555;
    // Zero uses port+1. Specify this explicitly when port is 65535.
    int control_port = 0;
    int width = 1440;
    int height = 900;
    int fps = 60;
    int history = 120;
    int timeout_ms = 5000;
    // Search order: this path, RAYFEM_EXECUTABLE, sibling RayFEM, then PATH.
    std::string executable;
};

enum class InitStatus { Started, AlreadyRunning, Failed };

struct InitResult {
    InitStatus status = InitStatus::Failed;
    std::string error;
    std::string executable;
    explicit operator bool() const { return status != InitStatus::Failed; }
};

// Verify a compatible viewer's control handshake before returning success.
// Launching is supported on macOS/Linux and only for loopback hosts. Remote
// viewers can be reused after verification. Window options apply only when
// starting a viewer. The viewer survives the caller unless shutdown is called.
InitResult init(const WindowParams& parameters = {});

enum class ShutdownStatus { Stopped, NotRunning, Failed };

struct ShutdownResult {
    ShutdownStatus status = ShutdownStatus::Failed;
    std::string error;
    explicit operator bool() const { return status != ShutdownStatus::Failed; }
};

// Ask the viewer at host/port/control_port to close through its normal cleanup
// path, and wait up to timeout_ms for its control connection to close. Other
// WindowParams members are ignored. This also works with an existing viewer
// that init reused. No process is killed, and no viewer is started.
// NotRunning is returned only when both local ports are free. An unreachable
// remote endpoint, incompatible viewer, or shutdown timeout returns Failed.
ShutdownResult shutdown(const WindowParams& parameters = {});

} // namespace RayFEM
