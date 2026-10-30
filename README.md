# RayFEM

RayFEM is an MFEM mesh and field viewer built with raylib and Dear ImGui. A simulation can send a mesh once, then stream complete frames containing several named grid functions and an optional displacement. The viewer can switch fields within a frame and inspect retained frames.

## Quick start

On macOS, with the command-line developer tools installed:

```sh
brew install meson ninja cmake pkg-config zeromq
meson setup build-rayfem --buildtype=release
meson compile -C build-rayfem
meson test -C build-rayfem --print-errorlogs
```

Start the viewer, then start a producer in another terminal:

```sh
./build-rayfem/src/RayFEM --port 5555
./build-rayfem/examples/exampleClient --geometry quad --order 4 --frames 300 --fps 30
```

From Python, `pip install rayfem` installs a wheel that includes the viewer
and a `rayfem` command that runs it (it prints the ports it chose; Ctrl-C
closes it); see [docs/python.md](docs/python.md).

## Documentation

- [Building and testing](docs/building.md): dependencies, local prefixes, tests,
  installing with pkg-config, sharing MFEM with a parent project, SERiF archives
  and the Stroid check.
- [Using the viewer](docs/viewer.md): appearance and navigation, spin, model
  axes, movies, color scales, the example producers, vector flow, tracking, profiles, surface traces,
  materials, and the viewer's limits.
- [Streaming from C++](docs/streaming.md): starting a viewer, the client API,
  frame schemas and model axes, displacement, and changing the viewer's listener.
- [Python package](docs/python.md): wheels, the data model shared with other
  packages, and the Python tests; the API is in [python/README.md](python/README.md).

## Layout

- `src/libRayfem`: the library (public headers in `include/libRayfem`; the
  MFEM-free streaming core in `lib/core`).
- `src/viewer`: the RayFEM viewer, by role: `pipeline`, `render`,
  `interaction`, `panels`, `figure`, `ui`.
- `examples`: `exampleClient` and the moving-mesh `sandbox`.
- `python`: the `rayfem` package and its nanobind extension.
- `tests`: `library`, `network`, `viewer`, `graphics` and `integration`.
- `build-config`, `subprojects`, `third_party`: dependency setup, wraps and
  vendored rlImGui.
- `experiments/glvis`: the optional GLVis integration experiment.
