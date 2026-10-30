# Building and testing

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

## Tests

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
