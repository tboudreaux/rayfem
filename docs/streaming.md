# Streaming from C++

## Start a viewer from C++

Call `RayFEM::init` before creating a streaming client. The client side never takes over the program's Ctrl-C: CZMQ would install its own SIGINT and SIGTERM handlers with its first socket, and RayFEM keeps (or restores) the program's instead. It returns once a compatible viewer reports that its network listeners and window are ready, or when startup fails or times out.

```cpp
#include "libRayfem/init.hpp"
#include "libRayfem/network/client.hpp"
#include <stdexcept>

RayFEM::WindowParams window;
window.width = 1280;
window.height = 800;
window.port = 5555;
// Optional: window.executable = "/path/to/RayFEM";
const auto result = RayFEM::init(window);
if (result.status == RayFEM::InitStatus::Failed) {
    throw std::runtime_error(result.error);
}
// result.status is Started or AlreadyRunning.
RayFEM::Client viewer(window.host, window.port);
```

`WindowParams` also accepts `fps`, `history`, `timeout_ms` (5,000 by default), and `control_port`. The control port defaults to the streaming port plus one; set it explicitly if that port is unavailable or the stream uses port 65535. A manually started viewer accepts the matching `--control-port` option. The info menu's Listener section changes the host, stream port and control port while the viewer runs: Apply opens the new endpoint before closing the old one, so a port in use or an unknown address leaves the viewer where it was. Received frames are kept. Clients of the old address have their sends refused and must reconnect, and `stats` and `shutdown` need the new ports. Hosts such as `0.0.0.0` accept connections from other machines without authentication; the menu warns before applying one. A nonce-based, versioned handshake verifies the viewer service; an unrelated process occupying either port yields `Failed`. This is service discovery on the existing trusted transport, not authentication.

Executable discovery checks the explicit `executable` setting, then `RAYFEM_EXECUTABLE`, a `RayFEM` executable beside the calling program, and finally `PATH`. Arguments are passed directly to the executable, so paths containing spaces are supported. Automatic launching is available on macOS and Linux for `localhost`, `127.0.0.1`, or `::1`; an existing remote viewer can be reused after verification. Window settings apply only to a newly launched viewer. Repeated calls reuse the running instance, and an instance still starting is awaited without opening another window.

The viewer runs independently of the producer and remains open when the producer exits. Startup timeouts leave any launched viewer running; a later call can reuse it if initialization subsequently completes. `sandbox --viewer /path/to/RayFEM` selects an explicit executable, and `--no-stream` skips startup and networking entirely.

Call `shutdown` explicitly when the producer should close the viewer. Pass the same endpoint settings used for `init` (or omit the argument for the default viewer):

```cpp
const auto closed = RayFEM::shutdown(window);
if (!closed) {
    throw std::runtime_error(closed.error);
}
// closed.status is ShutdownStatus::Stopped or ShutdownStatus::NotRunning.
```

Shutdown verifies the viewer, requests a normal exit, and waits for its control connection to close after cleanup. It releases the viewer's graphics, stream listener, and retained frames through the normal exit path. `ShutdownStatus::Failed` includes an error for an incompatible viewer or timeout; no process is forcibly killed. Only `host`, `port`, `control_port`, and `timeout_ms` are used. This closes the selected viewer even if another producer also uses it. Older viewers without shutdown support must be upgraded.

## Stream from C++

Include `libRayfem/network/client.hpp` and link the RayFEM library and its MFEM/CZMQ dependencies. Grid functions and their spaces must remain alive throughout each send call. All fields in a frame must belong to the registered mesh.

```cpp
#include "libRayfem/network/client.hpp"

// mesh, temperature, and velocity are existing MFEM objects.
RayFEM::Client viewer("localhost", 5555);
const std::uint64_t mesh_id = 1;
if (!viewer.sendMesh(mesh_id, mesh)) {
    throw std::runtime_error("Mesh send failed");
}

std::vector<RayFEM::NamedField> fields = {
    {"Temperature", &temperature},
    {"Velocity", &velocity},
};
for (std::uint32_t step = 0; step < number_of_steps; ++step) {
    // Update every grid function to the same simulation state here.
    RayFEM::FrameMetadata metadata;
    metadata.frame_index = step;
    metadata.sim_time = step * dt;
    if (!viewer.sendFrame(mesh_id, fields, metadata)) {
        throw std::runtime_error("Frame send failed");
    }
}
```

A frame is one complete state: send all of its named grid functions together. Use increasing frame indices. To replace a mesh or begin a new sequence for the same identifier, call `sendMesh` again before sending frames. The original `send(mesh, grid_function, metadata)` API remains available for single-field producers. A successful send means the packet was accepted by the local transport, not that the viewer has rendered or acknowledged it; `getLastError()` explains refusals. Use one `Client` from one thread.

Streaming is one-way, but the client watches its connection and refuses sends the viewer could not use instead of queueing them. A send fails when no viewer is listening at the client's address, after waiting up to a second for one: the viewer closed, or moved to another host or port. A frame fails when the viewer's listener has restarted since its mesh was sent, for example after the viewer changed its host or control port and came back on the same stream port, because that listener no longer has the mesh; call `sendMesh` again. `connected()` reports whether a viewer is connected now. A packet sent at the moment the viewer moves can still be lost; the next send reports it.

### Describe frames: coordinates, values and tags

Besides an index and a time, frames can carry values and tags a client declares up front. Register a `FrameSchema` once, before frames that use it:

- **Coordinates** (`coordinate = true`, integer or real) are progress axes such as an iteration count or Newton step. The viewer labels frames by the schema's `primary` coordinate, filters by coordinate ranges, and plots tracks against any coordinate.
- **Other values** (integer, real or text) are shown with each frame, for example a virial ratio or the solver in use, and numeric ones can be tracked across frames.
- **Tags** mark kinds of frames, for example `iterate` and `converged`, so a client can send both intermediate steps and converged states and the viewer can show either.
- `has_time = false` says simulation time means nothing, as for a static model being converged.

```cpp
RayFEM::FrameSchema schema;
schema.values = {
    {"iteration", RayFEM::ValueType::Integer, true, "", "Newton iteration"},
    {"virial ratio", RayFEM::ValueType::Real, false, "", "2K/|W|"},
    {"solver", RayFEM::ValueType::Text},
};
schema.tags = {{"iterate", "An intermediate iterate"}, {"converged", "The converged state"}};
schema.has_time = false;        // a static solve: label frames by iteration
schema.primary = "iteration";
viewer.registerSchema(schema);

RayFEM::FrameMetadata metadata;
metadata.frame_index = n;
metadata.values = {{"iteration", std::int64_t{n}}, {"virial ratio", ratio}, {"solver", std::string("Newton")}};
metadata.tags = {converged ? "converged" : "iterate"};
viewer.sendFrame(mesh_id, fields, metadata);
```

A frame may carry any subset of the declared values, each at most once; integers are accepted for real values. Undeclared names or tags, wrong types, non-finite numbers and text over 1024 bytes are refused by `sendFrame` with a reason in `getLastError()`. Registering again replaces the schema for later frames, and the client re-sends it with every `sendMesh`, so a restarted viewer recovers. Schemas allow up to 64 values and 64 tags. In the viewer, the funnel button on the timeline filters frames by tag and coordinate range (optionally discarding others as they arrive), and Analyze > Frame lists the shown frame's values and tags. `exampleClient --iterations 5` demonstrates time steps converged in five iterates; add `--static` for one static solve.

### Model axes

A schema can also describe the model's axes, such as a star's rotation axis or its magnetic poles, in mesh coordinates: a name, a direction toward the positive (north) end and a point the axis passes through. They are purely descriptive. The viewer can put the first axis up the screen for Home and Fit, draw the axes and spin the view about any of them; it never changes the data.

```cpp
RayFEM::FrameSchema schema;
schema.axes = {{"rotation", {0, 0, 1}, {0, 0, 0}, "Rotation axis"},
               {"magnetic", {0.2, 0, 1}, {0, 0, 0}, "Dipole axis"}};
client.registerSchema(schema);
```

A schema may hold up to 8 axes with unique names and finite, nonzero directions; it can declare axes alone. Axes are sent only when present, so a schema without them is encoded exactly as before. `exampleClient --axis 0 1 0.3` reports a rotation axis through the origin.

### Reuse geometry with displacement

A displacement specifies offsets from the registered reference mesh, not offsets from the previous frame. Frames therefore do not accumulate drift when dropped or viewed out of order. Use the mesh geometry's own finite element space for one displacement per geometry DOF. RayFEM also accepts a different scalar-basis finite element space on the same base mesh, provided it has one vector component per spatial dimension; it evaluates that vector field at each rendered point. ND/RT directional fields are not valid pointwise displacements:

```cpp
if (!mesh.GetNodes()) { mesh.EnsureNodes(); } // Before registering the mesh.
mfem::GridFunction displacement(mesh.GetNodes()->FESpace());
displacement = 0.0;

// Each frame: fill displacement with offsets for the reference geometry's DOFs.
// Use space.DofToVDof(dof, component) when assigning vector coefficients.
viewer.sendFrame(mesh_id, fields, metadata, &displacement);
```

For nodal polynomial geometry, those coefficients are point displacements at the mesh nodes. For NURBS geometry, they are control-point displacements with the original rational weights. They are not samples at arbitrary physical points. Call `EnsureNodes()` only if nodes are absent: MFEM converts existing NURBS nodes to polynomial H1 nodes when it is called unconditionally. The example applies an affine transformation to the original coordinate coefficients, which preserves NURBS geometry exactly. It never changes the producer's reference mesh.

### Degrees of freedom and field interpretation

MFEM's finite element transformations determine physical positions and values. A vector grid function can be a vector-valued H1 field or an ND/RT field whose basis functions are intrinsically vector-valued. Using only the grid function's vector dimension to distinguish those cases is insufficient.

DOF markers represent coefficients in the selected finite element space. For nodal H1/L2 bases, they correspond to interpolation nodes. For ND and RT spaces, DOFs represent oriented tangential or normal functionals; a marker is a representative location, and the coefficient is not a Cartesian vector component. NURBS coefficients belong to control points rather than interpolatory nodes; the viewer places their markers at evaluated Greville locations. Element-local DOFs can share a global coefficient and can have orientation signs or higher-order transformations. Hover information distinguishes coefficient values from evaluated field values. Native `NURBS_HCurl` and `NURBS_HDiv` basis markers are omitted because those directional spline bases do not expose point nodes. Standard ND/RT fields on NURBS geometry do have markers.

The scalar display preserves signed values. Vector displays offer magnitude and individual components. Divergence is available for RT fields and spatial H1 vectors; curl magnitude/components are available for ND fields and spatial H1 vectors. Unsupported field/view combinations report an error. DOF overlays are capped at 20,000 markers; vector visualization has separate sampling and computation budgets described below.
