# RayFEM

RayFEM is an MFEM mesh and field viewer built with raylib and Dear ImGui. A simulation can send a mesh once, then stream complete frames containing several named grid functions and an optional displacement. The viewer can switch fields within a frame and inspect retained frames.

## Build and run

The project uses Meson, Ninja, CMake, and a C++23 compiler. It reuses a parent-provided or installed MFEM when available, and otherwise builds its bundled MFEM. Subprojects provide raylib, CLI11, CZMQ, and Dear ImGui; ZeroMQ and its platform dependencies must be available. On macOS, install the command-line developer tools and the build dependencies, for example:

```sh
brew install meson ninja cmake pkg-config zeromq
meson setup build-rayfem --buildtype=release
meson compile -C build-rayfem
meson test -C build-rayfem --print-errorlogs
```

CZMQ is built from the included subproject; a separate system CZMQ install is unnecessary. If ZeroMQ is installed in a local prefix such as `.rayfem-deps` instead of the package manager's prefix, point both dependency discovery mechanisms at it during setup:

```sh
PKG_CONFIG_PATH="$PWD/.rayfem-deps/lib/pkgconfig" CMAKE_PREFIX_PATH="$PWD/.rayfem-deps" \
meson setup build-rayfem --buildtype=release \
  -Dpkg_config_path="$PWD/.rayfem-deps/lib/pkgconfig" \
  -Dcmake_prefix_path="$PWD/.rayfem-deps"
```

The bundled MFEM build enables exceptions, which the streaming decoder requires. The viewer's default build does not require GLVis, MPI, HYPRE, or the former fourdst configuration dependency.

Meson 1.8.2 and AppleClang were used for validation. Use a fresh `build-rayfem` directory when changing Meson versions; older Meson build directories may not be compatible. For an existing compatible build directory, use `meson setup build-rayfem --reconfigure --buildtype=release`. Start the viewer, then start a producer in another terminal:

```sh
./build-rayfem/src/RayFEM --port 5555
./build-rayfem/examples/exampleClient --geometry quad --order 4 --frames 300 --fps 30
```

The example sends five named fields per frame: H1 temperature, discontinuous L2 scalar data, H1 vector velocity, a Nédélec H(curl) field, and a Raviart–Thomas H(div) field. Each frame also contains a displacement in the mesh's own geometry space. The geometry is registered once.

Use `--help` on either executable for its options. The optional older GLVis integration experiment can be enabled with `-Dglvis_experiment=true`.

The **Light mode / Dark mode** button changes the controls, viewport, plots, legends, DOF markers, and flow overlays together. Scientific surface and heatmap colors retain their data meaning. `RayFEM --light` starts in light mode.

The orientation cube in the viewport's upper-right corner follows the camera. Click a face, edge, or corner for one of 26 views; the camera eases smoothly into place while preserving its target and zoom. Axis signs label the faces (Y is up, +Z is front). Dragging, panning, zooming, or fitting the mesh interrupts the transition.

The arrows below the cube rotate the image counterclockwise/clockwise around the viewing direction: single arrows move 5° and double arrows move 45°, with the same smooth easing. Repeated clicks accumulate their full increments during animation. Their artwork comes from `assets/rotation`. Rebuilds automatically regenerate the embedded icons when an SVG changes if `rsvg-convert` is available. Builds without it use the checked-in artwork after verifying that it matches the SVGs; stale artwork causes an explicit build error. Run `python3 tools/generate_rotation_icons.py` to refresh that portable fallback after editing assets. Installed viewers do not need an SVG renderer.

**View → Appearance → Keep reference plane** keeps the grid visible alongside loaded meshes. It is off by default, so the grid normally disappears when a mesh arrives; the setting can also be changed while waiting for frames. The plane defaults to XY at Z = 0. **Plane orientation** offers XY/XZ/YZ presets and X/Y/Z rotation angles about the world origin. The **Home** button between the rotation arrows (or the Home key over the viewport) smoothly returns to a view looking down at the plane from 30° above it, keeping the current focus and zoom. The initial view and **Fit** use the same home orientation.

Surface detail is automatic by default and follows the mesh and displacement orders. Curved surfaces use normals evaluated from their geometry, so high-order Stroid and NURBS meshes retain their shape and curved appearance. Under **Appearance → Detail**, disable **Automatic detail** to choose the number of subdivisions manually. This changes only the displayed triangulation; it does not refine or replace the MFEM mesh.

The color scale defaults to the current field's range throughout the selected volume, including the interior and regions hidden by clipping. It respects hidden elements, material exclusions, and an imported field's saved material support. Ranges are sampled from the finite element field, including interior points and element nodes; they are numerical estimates rather than guaranteed continuous extrema. Rotation, clipping, deformation display, and surface-detail changes reuse the same range. Under **Color scale**, turn off **Automatic volume range** and enter minimum/maximum values to keep a fixed scale across frames. Changing the field or evaluated component resets automatic scaling. Hover over the legend for precise limits and the volume sample count.

**View → Color scale → Color map** selects Viridis (default), Plasma, Inferno, Cividis, Coolwarm, or legacy Jet, with optional reversal. Surface colors, clipping sections, legends, 2D profile heatmaps, and flow magnitude colors share the selected map. The sequential maps use Matplotlib's published lookup tables; Coolwarm is a diverging map (set symmetric manual limits to put zero at its midpoint). Material/boundary ID colors and categorical tracking lines retain their separate palettes. Profile heatmaps use their sampled profile range, and flow colors use the magnitude range shown in the flow controls. The library exposes `ColorMap`, `SampleColorMap`, and `FieldColor` in `libRayfem/mesh/colormap.hpp`; set `RenderOptions::color_map` and `reverse_colors` for CPU surface generation.

The **Scale** selector offers **Linear** (default), **Signed log**, and **ZScale**. Signed log uses `sign(x) * log1p(abs(x)/threshold)`, which is continuous at zero and supports either sign. Its automatic threshold is 1% of the largest absolute display limit; a custom threshold is in field units. Legend labels and hover values invert the selected transform. ZScale uses the [IRAF/DS9 ranked-sample line fit](https://iraf.readthedocs.io/en/latest/tasks/images/tv/display.html#zscale-algorithm), with iterative outlier rejection and adjustable contrast (default 0.25), then maps the fitted limits linearly. Higher contrast narrows the limits; zero uses the full range. Turning off **Automatic volume range** freezes the displayed limits, including fitted ZScale limits, across frames. The legend tooltip retains the full volume extrema.

ZScale takes up to 1,000 deterministic samples spread throughout the selected field volume, including clipped regions. This samples reference-cell lattices and FE nodes; it approximates the distribution and is not weighted by physical volume. Material exclusions still apply. Flow uses its whole-volume magnitude distribution; 2D heatmaps use their valid plane samples. Scalar profile and tracking axes remain unchanged. Library consumers can set `RenderOptions::color_scale`, or use `ResolveColorScale`, `ColorScaleFraction`, and `ColorScaleValue` from `libRayfem/mesh/color_scale.hpp`. To reproduce the viewer's volume-based limits, request `FieldRangeOptions::distribution_samples`, resolve the mapping from the resulting `FieldRange::samples`, then call `ApplyFieldColorRange(surface, mapping)`.

Tests are grouped by what they need: `tests/library` (libRayfem), `tests/network` (streaming, launch and shutdown) and `tests/viewer` (the viewer's headers) run without a window; `tests/graphics` needs a desktop session; `tests/integration` holds programs and scripts run against a viewer, an installed package or a parent project. CPU checks (geometry, volume color ranges, streaming, viewer startup, clipping, profiles, camera controls, animated displacement, frame tracking, chart helpers and themes, surface traces, material selection, vector flow, archive loading, and fixed/pulsating diffusion) run through `meson test`. For a local ZeroMQ prefix, also set `PKG_CONFIG_PATH="$PWD/.rayfem-deps/lib/pkgconfig"` and `CMAKE_PREFIX_PATH="$PWD/.rayfem-deps"` on rebuild commands so automatic CMake regeneration finds its headers. Camera tests cover continuous orbit/pan input, drag ownership, release, and zoom limits without a graphics window. Separate graphics integration executables need an active graphical session and OpenGL context:

```sh
./build-rayfem/tests/graphics/viewer_test build-rayfem/test-output subprojects/mfem/data
# Append --visible to watch the same checks in a window.
./build-rayfem/tests/graphics/flow_viewer_test build-rayfem/test-output \
  subprojects/mfem/data/ball-nurbs.mesh
# Additional mesh filenames may follow, including the Stroid fixtures below.
./build-rayfem/tests/graphics/imgui_backend_test build-rayfem/test-output/large-ui-list.png
```

It renders 168 frames across curved quadrilateral, triangular, hexahedral, tetrahedral, and three original NURBS meshes, checks framebuffer changes when switching fields or applying displacement, and saves 48 screenshots. The checks include ND curl, RT divergence, vector arrows, DOF markers, original element boundaries, both cutting algorithms, whole-element cuts, and repeated GPU resource cleanup. An additional sphere-to-ellipsoid-to-sphere cycle checks fixed-camera images for exact NURBS and order-5 H1 geometry. The displacement suite separately streams 132 states across both geometries and both vector orderings, verifying immutable reference nodes, independent snapshots, analytic axes, and scrambled replay. This exercises the rendering pipeline; interactive controls and hover behavior are separate UI interactions.

`flow_viewer_test` exercises the viewer's actual flow overlay in both dark and light modes, writing separate `dark` and `light` screenshot directories. It checks distributed arrows within one P5 cell, visibility through opaque surfaces, streamlines, clipping, material exclusion/restoration, and the arrow fallback on embedded curved surfaces. Additional mesh arguments exercise the same checks on NURBS or generated Stroid meshes.

`imgui_backend_test` renders 216,000 UI vertices across multiple 16-bit index ranges and checks their pixels. This guards the backend vertex-offset support needed by dense DOF overlays.

## Install and link with pkg-config

Install the library, public headers, viewer, and package metadata into a prefix:

```sh
meson setup build-rayfem --reconfigure --prefix="$HOME/.local" --libdir=lib
meson compile -C build-rayfem
meson install -C build-rayfem
export PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export PATH="$HOME/.local/bin:$PATH"
g++ $(pkg-config librayfem --cflags --libs) --std=c++23 test.cpp -o test
./test
```

The package is named **`librayfem`**, and the installed library is `librayfem`. Its public headers remain under `libRayfem/`, for example `#include <libRayfem/init.hpp>` and `#include <libRayfem/network/client.hpp>`. The package exports the MFEM, raylib, and CZMQ requirements as well as RayFEM's own flags. A custom ZeroMQ installation also needs its `lib/pkgconfig` directory on `PKG_CONFIG_PATH`.

The command above is supported on macOS. For portability to linkers that process libraries strictly from left to right (including GNU `--as-needed` configurations), put libraries after the source:

```sh
g++ --std=c++23 $(pkg-config --cflags librayfem) test.cpp \
    $(pkg-config --libs librayfem) -o test
```

Shared libraries are the default. Runtime search paths are included for the installation and its configured external libraries, so running a consumer does not require a `DYLD_LIBRARY_PATH` or `LD_LIBRARY_PATH` workaround. Use a compiler and C++ runtime compatible with the MFEM/SERiF installation you link.

`python3 tests/integration/package_smoke.py /your/install/prefix` compiles and runs a separate C++23 consumer with both command layouts and no library-path overrides.

## Share MFEM with another project

`-Dmfem_provider=external` requires an existing MFEM and prevents the bundled fallback. `-Dmfem_prefix=/path/to/mfem-prefix` selects an existing installation through its CMake package. MFEM must provide double precision and exception support.

When RayFEM is a Meson subproject, a parent's `dependency('mfem')` or `meson.override_dependency('mfem', mfem_dep)` is reused before searching for another MFEM. SERiF's source provider already registers that override. Add RayFEM after the parent's dependencies and libraries have been defined:

```meson
# In a parent that has not already registered its MFEM:
# meson.override_dependency('mfem', mfem_dep)

# Optional: expose the parent's existing SERiF library to the archive reader.
meson.override_dependency('serif', serif_dep)

rayfem_project = subproject('rayfem', default_options: [
  'mfem_provider=external',
  'build_viewer=enabled',
])
rayfem_dep = rayfem_project.get_variable('rayfem_dep')
executable('simulation', 'simulation.cpp', dependencies: [serif_dep, rayfem_dep])
```

Place this repository at `subprojects/rayfem`, or supply it through the parent's existing dependency arrangement. RayFEM does not fetch or build SERiF. Viewer, examples, and tests default to enabled for standalone builds and disabled for subproject builds; their `build_viewer`, `build_examples`, and `build_tests` feature options can override that choice.

`tests/integration/parent_integration.py --mfem-prefix /existing/mfem-prefix` creates an independent temporary parent project and verifies that a parent-created field evaluates correctly through RayFEM with exactly one MFEM library loaded. Add `--serif-source /existing/serif --serif-build /existing/serif/build` to also exercise a parent-provided SERiF dependency.

## Optional SERiF archives

The archive adapter uses SERiF's own checked reader, resource reconstruction, and coefficient maps. A normal RayFEM build needs no SERiF headers or library. An already registered/installed `serif` dependency can enable the adapter; for the current uninstalled development checkout, point RayFEM at its existing source and build:

```sh
meson setup build-rayfem-serif \
  -Dserif=enabled \
  -Dserif_source_dir=/Users/tboudreaux/Programming/4DSTAR/MeanField_SERiF \
  -Dserif_build_dir=/Users/tboudreaux/Programming/4DSTAR/MeanField_SERiF/build
meson compile -C build-rayfem-serif
./build-rayfem-serif/src/RayFEM saved-star.serif
```

Add the same ZeroMQ discovery flags used for a standalone build if needed. This adapter is compiled privately as C++26 to match SERiF's current headers; RayFEM's public API and consuming programs can still use C++23. The adapter shares the existing SERiF MFEM build. Rebuild RayFEM after changing that external development library's ABI. `-Dserif=disabled` explicitly removes archive support.

Use **Open**, **Ctrl/Cmd+O**, drag one `.serif` file into the viewer, or pass `--open filename.serif`. The Open dialog browses folders or accepts a full path. A successful open replaces the workspace; a failed open preserves it and shows the reader's error. Imported fields use the normal field selector, material controls, profiles, flow views, plots, and CSV export. Regions outside a field's saved material support are excluded from both display and analysis.

The loader currently reconstructs SERiF's serial 3D H1, L2, and H(div) resources, including curved and nonconforming meshes. It checks schema versions, coefficient identities, signed/reduced/true numbering, material support, and payload checksums. Identical duplicate checkpoint states are deduplicated; distinct states require explicit selection through the API:

```cpp
#include <libRayfem/io/serif.hpp>

auto loaded = RayFEM::LoadSerifFile("star.serif", {
    .discretization_path = "configuration/finite_elements",
    .state_path = "solution/state",
});
if (!loaded) throw std::runtime_error(loaded.error);
// loaded.frames own all required MFEM resources.
// loaded.warnings describe non-spatial coordinates and mapping limitations.
```

Paths are archive subtree names, so omit options for automatic selection. `RayFEM::SerifLoadingAvailable()` reports the compiled capability. A disabled build keeps this API and returns an explanatory error.

Saved nodal `displacement.vector` fields are applied. Surface-model coefficients and global scalars are reported rather than treated as nodal data. Without a saved nodal displacement, the reference geometry is displayed; the compactified exterior remains in saved mesh coordinates rather than applying SERiF's exterior mapping. An archive snapshot has no assumed physical time, so its displayed time is zero. Multiple referenced meshes appear as separate timeline views. Meshes omitted from an archive require an external resolver that this viewer adapter does not yet expose. Files over 512 MiB or selected payloads over 256 MiB are rejected with a clear error.

## Exercise the viewer

These commands provide small reproducible streams. Run them with the viewer already listening:

```sh
# Curved quadrilaterals and triangles.
./build-rayfem/examples/exampleClient --geometry quad --order 5 --resolution 5 --frames 180
./build-rayfem/examples/exampleClient --geometry tri --order 4 --resolution 5 --frames 180

# Curved hexahedra and tetrahedra.
./build-rayfem/examples/exampleClient --geometry hex --order 3 --resolution 3 --frames 120
./build-rayfem/examples/exampleClient --geometry tet --order 3 --resolution 3 --frames 120

# Rapid streaming: simulation time still advances by --dt per frame.
./build-rayfem/examples/exampleClient --geometry quad --order 3 --frames 1000 --interval 0 --quiet

# Original NURBS geometry is retained; --order controls the example fields.
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/disc-nurbs.mesh --order 3 --frames 120
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/beam-hex-nurbs.mesh --order 3 --frames 120

# Sphere to ellipsoid and back, with one mesh registration.
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/ball-nurbs.mesh \
  --motion ellipsoid --deformation 0.8 --frames 241 --dt 0.02617993877991494 --fps 30

# Straight geometry and no displacement for analytic derivative checks.
./build-rayfem/examples/exampleClient --geometry hex --curvature 0 --no-displacement --frames 1

# Three material bands for categorical coloring and analysis selection.
./build-rayfem/examples/exampleClient --geometry hex --resolution 3 --materials 3 --frames 120
```

`--fps` and `--interval` are alternative ways to request the send cadence. `--interval 0` sends as quickly as projection, serialization, and transport permit. `--dt` controls simulation time independently. `--motion ellipsoid` starts from the reference geometry, reaches axis factors `(1+A, 1-A/2, 1+A/4)` at simulation time pi, and returns at 2pi (`A` is `--deformation`). With the ball mesh this is a sphere-to-ellipsoid cycle. `--deformation` controls the displacement amplitude, and `--no-displacement` leaves frames on the registered geometry. Imported meshes must be full-dimensional 2D or 3D meshes for this example's ND/RT spaces.

On the undeformed straight geometry, the example's vector fields have these analytic values:

| Field | Definition | Divergence | Curl |
| --- | --- | --- | --- |
| H1 velocity, 2D | `(0.2x-y, x+0.3y)` | `0.5` | `2` |
| H1 velocity, 3D | `(0.2x-y, x+0.3y, 0.4z)` | `0.9` | `(0,0,2)` |
| ND circulation | `(-y,x[,0])` | `0` | `2` in 2D; `(0,0,2)` in 3D |
| RT flux | `(x,y[,z])` | Spatial dimension | `0` |

These are useful checks for the appropriate derivative view. On curved geometry, projection into a chosen finite element space can introduce approximation error. Displacement changes displayed positions only: field values, vector components, and derivatives are evaluated on the registered, undeformed mesh. RayFEM does not transport or recompute ND/RT coefficients on the displayed geometry.

## Moving-mesh diffusion example

`examples/sandbox.cpp` is a small MFEM solver that diffuses a smooth temperature field on an expanding and contracting square. It starts RayFEM automatically, or reuses the compatible viewer already listening:

```sh
./build-rayfem/examples/sandbox
# Run the same calculation and numerical checks without a viewer or pacing.
./build-rayfem/examples/sandbox --no-stream
# Compare diffusion on a fixed mesh.
./build-rayfem/examples/sandbox --no-stream --amplitude 0
```

The example streams one immutable reference mesh, then **Temperature**, **Mesh velocity**, and a displacement for every frame. A separate solver mesh moves according to `s(t) = 1 + amplitude*sin(2*pi*t/period)`. MFEM reassembles mass and diffusion matrices on that geometry and solves `(M_new + dt*K_new)*T_new = M_old*T_old` with backward Euler and conjugate gradients.

This is conservative diffusion, `partial_t T + div(w*T) = diffusivity*Laplacian(T)`, with zero diffusive flux at the moving boundary. Here `T` represents heat per unit area: expansion dilutes it and contraction concentrates it while total heat remains constant. Each step checks the linear residual, total heat, and domain area; fixed-mesh runs also check energy decay. `--dt` controls simulation time, while `--fps` controls streaming pace. Use `--help` for mesh order/resolution, time-step, diffusivity, and pulsation settings.

## Python

`pip install rayfem` installs a wheel that includes the viewer; nothing is
compiled. Wheels use the stable ABI (one per platform for CPython 3.12 and
later) and cover macOS and Linux on x86_64 and arm64. See
[python/README.md](python/README.md) for the API.

The Python package shares no C++ types with other extensions: meshes travel
as MFEM mesh text and fields as a space header plus a float64 array, so
PyMFEM, SERiF or plain NumPy (`rayfem.mesh`) can all send to the viewer
whatever MFEM build they use. In C++, `RayFEM::StreamClient`
(`libRayfem/network/stream.hpp`) is the same MFEM-free client;
`RayFEM::Client` serializes MFEM objects and sends through it.

Build wheels with [cibuildwheel](https://cibuildwheel.pypa.io) (configured in
`pyproject.toml`; `.github/workflows/wheels.yml` runs it on each platform), or
locally with `python -m build --wheel`. Both use `-Dbundle=true`, which builds
MFEM, raylib, CZMQ and libzmq statically from source. For development,
`-Dpython=enabled` adds the extension to a normal build; it needs nanobind in
the configuring Python. `python/tests/test_wheel.py` checks an installed
package, and `python/tests/test_diffusion_stream.py BUILD/tests/integration/diffusion_stream`
starts a viewer from Python, streams a 2D heat-equation solve to it from C++
(`tests/integration/diffusion_stream.cpp`), and confirms every frame arrived with
`rayfem.stats()`.

## Start a viewer from C++

Call `RayFEM::init` before creating a streaming client. It returns once a compatible viewer reports that its network listeners and window are ready, or when startup fails or times out.

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

## Vector flow

Select a vector field, then click **Flow**. **View → Vector flow** offers **Arrows**, **Streamlines**, or both. Arrows sample throughout the domain, independently of the number of elements; a single high-order element can therefore display many arrows. Seeds are spread within elements and across elements, weighted by an estimate of their physical volume. They map directly through the curved geometry, avoiding a costly inverse search for every seed. Arrows have a common adjustable length, with color indicating magnitude. **Through surfaces** keeps interior vectors visible through an opaque mesh. Turn it off to use ordinary surface occlusion.

Streamlines follow the selected frame's finite element vector field using selectable RK2 or RK4 integration. **Vector flow** provides separate **Arrow count** and **Streamline count** controls, requesting a total across the included domain rather than a resolution raised to mesh dimension. The display reports actual/requested counts; zero vectors, clipping, and bounded preview work can produce fewer paths. **Update flow** allows a longer calculation for dense views. **Flow settings** controls step length, maximum path length, integration in both directions, and the moving highlight. A bright, smooth pulse travels along each line, with adjustable pulse length and optional gradual thickening. Cached GPU ribbons animate without rebuilding finite-element samples or drawing moving spheres. These are steady streamlines, not particle trajectories integrated through successive simulation times. The library's `FlowOptions::seed_count` requests a direct total; zero preserves `seed_resolution` behavior, and the existing seed, point, and evaluation budgets still apply.

Automatic previews update at most four times per second with a short computation budget. Pause on a frame and press **Update flow** for a larger budget. The panel reports the sampled frame, arrow/path counts, and whether a preview is incomplete. Material exclusions and either clipping mode apply to flow sampling and display. Switching frames, fields, displacement, or selection invalidates the cached result.

`libRayfem/vector/flow.hpp` exposes `SampleVectorField` and `BuildStreamlines`, with explicit seed positions, RK2/RK4 selection, point/evaluation/search limits, and a cancellation callback. Location and evaluation use the actual curved MFEM mapping, including NURBS and geometry-space displacement. H1 vectors and ND/RT vector bases retain their reference-mesh field interpretation under displacement. Streamline integration uses normalized vectors so step length and path limits are physical distances. Paths stop at the domain or selection boundary, stagnation, invalid values, or a failed point location. Sampling is bounded and approximate: finite seed sets can miss very thin regions, and unusually distorted geometry can require smaller steps or different seeds.

Streamlines currently require full-dimensional 2D or 3D meshes. Arrow sampling also supports 2D surfaces embedded in 3D; streamlines on those surfaces need a separate surface-constrained integrator.

The explicit desktop test `build-rayfem/tests/graphics/streamline_renderer_test build-rayfem/test-output/streamline-pulses` checks moving highlights, joins, thickness, clipping phase, depth occlusion, and near-camera rendering with framebuffer readback. `build-rayfem/tests/graphics/flow_viewer_test build-rayfem/test-output/flow-pulses subprojects/mfem/data/disc-nurbs.mesh subprojects/mfem/data/ball-nurbs.mesh` checks field sampling and rendering together in both themes. These tests require a desktop graphics session; the ordinary Meson suite also checks pulse math and direct seed counts without a window.

## Frame tracking and in-window plots

The compact **View** inspector controls the field, regions, appearance, deformation, and clipping. **Analyze** contains profile and tracking controls; **Info** holds connection and performance details. **Live** follows incoming frames. Playback and the slider inspect retained frames. **Plots** hides or shows the lower plot area.

**View → Appearance → Explode elements** opens gaps between every element, from zero (the intact mesh) to 0.95 (small separated cells). It uniformly shrinks each displayed element about its mapped center after applying displacement, preserving curved and NURBS shapes. Interior faces become visible, including distinct traces of discontinuous fields. Element edges and DOF markers follow the separated elements; shared DOFs retain their global IDs and appear on each owning cell. Clipping operates on the displayed separated geometry. Material exclusions and boundary IDs are preserved; newly exposed internal faces have boundary ID 0. The same setting is available as `RenderOptions::element_explosion` or the viewer's `--explode` option.

Explosion changes display geometry only. Profile coordinates, integrals, tracking, and vector-flow paths continue to use the physical domain. The reference mesh and field coefficients are never modified, and the automatic color range continues to cover the included field volume.

Click a visible element to select it; Shift-click adds or removes elements from the selection. Clicking empty space clears the selection, while dragging still orbits. Selected cells are highlighted in gold. Picking follows the actual curved, displaced, exploded, or clipped surface, including section caps.

The **Elements** tab provides **Hide selected**, **Show selected**, and a scrollable object explorer with visibility controls for every element. Filter the list to selected or hidden cells to restore individual elements, or use **Show all**. IDs are zero-based MFEM element indices. Selection and visibility persist across frames that share the same mesh registration; a different mesh registration or revision resets them. Material exclusions still apply when an element's visibility checkbox is on.

The collapsible **Selection statistics** pane reports physical area/volume, centroid, bounds, materials, geometry/order information, and the current field operation's minimum, maximum, mean, and integral. Bounds and extrema are sampled estimates; integrals use physical quadrature weights. Measurements include displacement and exclude hidden cells and materials, while display clipping and explosion do not alter their physical meaning. On a new streamed frame, use **Update** to refresh the snapshot without repeatedly performing expensive integration during playback. The library API is `ComputeElementStatistics` in `libRayfem/mesh/element_statistics.hpp`.

Hidden cells are omitted from rendering, DOF/vector overlays, field ranges, profiles, surface traces, and tracking. Changing visibility marks profile snapshots stale and rebuilds tracks from retained frames. CSV exports record excluded element IDs. The library equivalents are `RenderOptions::excluded_elements` and scoped `TrackingRequest::excluded_elements`; indices remain those of the original registered mesh.

Surface construction batches element field evaluation and reuses polynomial reference bases within each build, with bounded memory and direct MFEM evaluation for rational NURBS bases. Element outlines are cached in GPU buffers, including their camera-dependent depth offset. Color-map, scale, and range changes update only the color buffer instead of rebuilding the mesh. View caches remain bounded. For repeatable performance measurements, run `build-rayfem/tests/graphics/render_benchmark`; the explicit desktop test `build-rayfem/tests/graphics/edge_renderer_test build-rayfem/test-output` checks cached outlines and compares their CPU submission cost with the former per-edge path. `build-rayfem/tests/integration/viewer_lifecycle_test /absolute/path/to/RayFEM` exercises the real window's startup, reuse, shutdown, and restart on ports 5696–5697.

Click **Pick DOF**, then click a marker on the mesh. RayFEM adds a colored coefficient trace to **Tracks**, backfills the retained history, and samples incoming frames. The selected marker uses its trace color. Alternatively, enter a scalar-space DOF index and coefficient component under **Analyze → Frame tracking**. `DofToVDof` maps vector ordering; ND/RT traces represent oriented functional coefficients, and NURBS traces represent control coefficients. They do not become physical point values merely because their markers move with displacement.

Tracks retain up to 4,096 scalar samples each, with up to eight visible/configured traces. They continue after full frames leave the history. Time/frame axes, hover crosshairs, sample values, and a cursor for the displayed frame are available in the chart. Missing fields, skipped frame indices, and incompatible DOF associations produce gaps. A selected DOF stays bound to its original session, mesh registration, and finite element basis/layout; add a new track after changing topology or basis. Backfill and incoming work share a bounded queue and sampling budget; frames evicted before sampling are counted in the inspector.

**Save CSV** exports a completed trace to `rayfem-exports/track-N.csv`, with frame/time, validity, coefficient/view settings, mesh/session identity, and errors. Files contain the currently retained scalar samples. Removal deletes a trace from the UI only. Point probes, quadrature-sampled extrema, physical mean/integral, and L2 norm are also available. Statistics use the selected field operation and geometry settings at track creation. Integration weights follow the displayed displaced geometry; values and derivatives retain the reference-mesh convention. The exact displacement geometry requirements match profiles. Minimum/maximum are sampled estimates, not guaranteed global finite-element extrema.

`libRayfem/frame/tracking.hpp` exposes `TrackingRequest`, `FrameTracker`, and `ExportTrackingCSV` independently of the UI. Call `bindDof(currentFrame)` before backfilling mixed histories for a picked DOF. Tracks store scalar results and bounded identity metadata, never frame pointers. A track stops after 256 distinct stream/mesh/revision identities; remove it and add another to start fresh.

## Ray and plane profiles

Use **1D profile** or **2D profile** in the viewport toolbar. The **Analyze** inspector holds the sampling controls, and the **Profile** tab below the mesh shows the result. Detailed coordinates and the replaceable sampling method are under **Sampling settings**. Edit the coordinates or drag the gold handles in the viewport, select a sampling algorithm, then press **Sample profile**. The ray plot preserves gaps outside the mesh; the plane heatmap preserves physical aspect ratio and supports value hover. Profiles sample the currently selected field and view operation, independently of the display clipping plane.

Choose a path and press **Export CSV** to save the sampled result. Relative paths are resolved from the viewer's working directory. Changing the frame, field, displacement, profile geometry, or algorithm marks the preview as stale and disables export until it is resampled. Sampling is explicit so it does not interrupt every streamed frame.

`libRayfem/profile/sampler.hpp` provides ray and plane sampling independently of the viewer. Ray samples include both endpoints and use physical distance along a normalized direction. Plane samples use an orthonormal basis with physical `u` and `v` extents, stored row by row with `u` varying fastest. Each returned sample includes its physical position, value, validity, and containing element.

The built-in `exhaustive` and `coherent` algorithms locate points through MFEM inverse element mappings and evaluate the finite element field there. `coherent` uses the preceding sample to accelerate the search. Both choose the lowest element ID when a point lies on a shared boundary, making discontinuous-field traces reproducible. Locations outside the mesh remain in the result with `valid=false`, `element=-1`, and a NaN value; they are not filled with a nearby node's value.

```cpp
#include "libRayfem/profile/sampler.hpp"
#include <fstream>

RayFEM::ProfileRegistry registry;
auto locations = RayFEM::MakeRaySamples({0, 0.5, 0}, {1, 0, 0}, 1.0, 101);
auto samples = registry.sample("coherent", mesh, temperature, locations);

RayFEM::ProfileMetadata metadata;
metadata.field_name = "Temperature";
metadata.frame = step;
metadata.time = step * dt;
metadata.algorithm = "coherent";
std::ofstream csv("temperature-profile.csv");
RayFEM::ExportProfileCSV(csv, samples, metadata);
```

Pass `RenderOptions` to choose magnitude, a component, divergence, or curl using the same field semantics as surface views. For profiles through displaced geometry, the displacement space must match the mesh nodes' finite element collection and degrees of freedom; either MFEM vector ordering is accepted. The sampler locates points in the displayed geometry while evaluating field values and derivatives on the original reference mesh. Other displacement spaces can be displayed but are explicitly unavailable for displaced-profile sampling.

Applications can subclass `ProfileAlgorithm` and call `ProfileRegistry::registerAlgorithm` to add or replace an algorithm by its ID. `algorithms()` supplies the available IDs and display names. CSV exports include the selected field, frame, time, view, component, and algorithm on every row, followed by `s/u/v`, physical coordinates, value, validity, and element ID. Field names are escaped for ordinary CSV readers.

## Surface traces

Choose **Surface trace** to measure a field along a parameterized path on a 3D object's exterior. A **Meridian / colatitude** trace varies the angle from the chosen axis; a **Latitude / azimuth** trace circles that axis at a fixed colatitude. **Trace settings** holds the center, axis, angular range, sample count, refinement, and selectable sampling method. Place the center inside the object for a radial trace, then press **Sample surface**. The gold line follows the sampled surface, and the **Surface** plot displays values against angle or approximate arc length. Hover to inspect values and use **Export → Save CSV** to save the result.

Each path sample casts an independent ray and selects its nearest forward hit on an original exterior face. Hidden materials are skipped; a missed ray stays a gap. Newly exposed internal material interfaces and clipping caps are not surface-trace targets. Volume meshes and 2D surfaces embedded in 3D are supported. The center and axis can be moved in the inspector; azimuth starts at projected world X, or world Y when X is nearly parallel to the axis. Field values and derivatives retain their reference-mesh meaning while hit positions follow displacement.

The default **curved** algorithm refines faces to find initial intersections, then solves the ray intersection against the actual curved finite-element face. The **refined** algorithm uses triangle intersections to choose a reference point and evaluates that point on the actual face; its distance from the ray is exported as a residual. Refinement improves search coverage, but highly folded faces can require more refinement and are not guaranteed to have every intersection discovered. Arc length sums chords between consecutive valid hits without bridging gaps. Increasing the sample count improves this approximation.

`libRayfem/profile/surface_trace.hpp` exposes the same mechanism independently of the UI. `MakeMeridianRays` and `MakeAzimuthRays` create angular paths in radians; arbitrary paths can supply their own ordered `SurfaceTraceRay` list with an application-defined parameter. Subclass `SurfaceTraceAlgorithm` and register it with `SurfaceTraceRegistry::registerAlgorithm` to add or replace a selectable algorithm. Samples and CSV include the parameter, arc distance, ray distance, physical and reference positions, field value, validity, intersection residual, and element/face/boundary/material IDs, plus field/frame/algorithm and material-selection metadata. Requests exceeding 250,000 seed triangles, 24 million ray–triangle checks, or 100,000 curved ray–face solves are rejected with a request to reduce refinement or sampling, keeping synchronous work bounded.

## Materials and boundaries

In **View → Regions**, choose field, material ID, or boundary ID coloring. Material colors follow element attributes; boundary colors follow MFEM boundary attributes. On a 2D mesh, boundary IDs color the element-edge overlay, while area interiors use ID 0. Synthetic exposed material interfaces and cutting sections also use boundary ID 0. Categorical coloring does not change the field used for analysis.

Clear a material checkbox to remove those elements from surfaces, element edges, DOF/vector overlays, ray/plane profiles, surface traces, and tracked measurements. The remaining volume displays newly exposed faces with their own outward orientation and field trace. Material selection is independent of the display clipping plane. Shared DOFs remain available when at least one incident element is included; a DOF supported only by hidden elements yields an invalid tracking sample. Point searches on shared boundaries choose the lowest included element ID.

Changing material selection marks profile snapshots stale. Existing tracks keep their original DOF association but rebuild their results from the full frames still retained by the viewer; older scalar-only history is discarded because its fields are no longer available for reanalysis. CSV exports record the excluded material IDs. The same selection is available through `RenderOptions::excluded_materials` and `TrackingRequest::excluded_materials`; `FrameTracker::setExcludedMaterials` resets samples while preserving a selected DOF's binding. **All** and **None** quickly reset the selection, and `exampleClient --materials N` assigns up to 16 element-attribute bands along X for testing.

## Scope

RayFEM renders a tessellated approximation of curved geometry. Increase the refinement setting when viewing strongly curved or high-order meshes. Volume meshes are displayed through their exterior surfaces, with an interior section added when cutting through elements. The finite element field remains attached to the selected frame when its display mode is changed.

Element boundaries follow the original finite element edges through their curved geometry map. They omit the extra diagonals and subdivision edges used to render a smooth surface. Boundaries on a cutting section follow intersections with original element faces.

The cutting modes follow GLVis's distinction: **cut through elements** adds an interior section with sampled field values; **whole elements** retains only cells whose corners lie behind the plane and displays their newly exposed faces. Cut-through offers two algorithms: **Refined** intersects mapped volume subdivisions and is intended for curved and NURBS geometry; **Planar** intersects original corner edges and is faster for planar-faced meshes. The planar approximation can differ visibly on curved meshes. Refinement improves the curved approximation; cuts retain the half-space where `dot(position - plane.point, plane.normal) <= 0`. The CPU APIs are in `libRayfem/mesh/clipping.hpp`.

The viewer retains at most 120 frames by default, also bounded by a 256 MiB retention estimate covering field coefficients, finite-element bookkeeping, and distinct shared meshes; `--history` adjusts the frame limit. Shared geometry is counted once. This estimate is not an exact allocator or total process-memory limit, and the viewer keeps at least one complete frame. Its receive queue holds at most 64 complete frames or 128 MiB and drops the oldest complete frames when overloaded. The status panel reports these drops. Mesh registration storage is limited to 16 meshes or 128 MiB; re-register a mesh if its registration is evicted. Each frame supports up to 64 unique named fields, with names up to 255 bytes, and each packet has a 64 MiB payload limit. Rapid streams can outrun rendering: a displayed frame rate is distinct from the producer's send rate. For persistent storage or exact replay of an entire long run, keep simulation output separately.

The streaming transport is intended for trusted simulations. It is not an authenticated public service. The example uses serial MFEM meshes and grid functions; distributed parallel mesh assembly is outside this example's scope.

The receiver accepts polynomial field orders up to 32 and NURBS degrees up to 10. The NURBS bound protects the fixed-size evaluator in the bundled MFEM version; higher degrees are rejected before evaluation.

## Optional Stroid integration check

An existing [Stroid](https://github.com/4D-STAR/stroid) executable can generate real order-five, cube-projected spheroid fixtures for RayFEM. Stroid is optional; the generator writes its configurations, meshes, and provenance into the build directory and does not modify the Stroid checkout.

```sh
python3 tests/integration/generate_stroid_fixtures.py /path/to/stroid/build/tools/stroid
./build-rayfem/tests/integration/stroid_flow_test \
  build-rayfem/test-output/stroid-order5.mesh \
  build-rayfem/test-output/stroid-order5-nc.mesh \
  build-rayfem/test-output --graphics
```

Omit `--graphics` for the CPU and local-socket checks. The first fixture has spheroidal axes `(1, 1, 0.75)` and core/envelope materials 11 and 17. The second adds vacuum material 23, with different stellar and vacuum refinement levels to exercise nonconforming interfaces. `stroid-provenance.json` records the actual generator version and executable hash, exact commands, configurations, mesh hashes, and the current source checkout revision when available.

The integration test checks curved Jacobians, boundary shape, evaluated vector values, rotational streamline invariants, traversal across all three materials, and termination at an excluded vacuum. It streams each mesh once with three independent displacement frames and two fields, then evaluates the received deformed geometry. The optional graphics pass saves images of both meshes and their streamlines. To inspect either fixture interactively, start RayFEM and run `exampleClient --mesh-file build-rayfem/test-output/stroid-order5.mesh --frames 120 --fps 30`.
