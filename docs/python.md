# Python package

`pip install rayfem` installs a wheel that includes the viewer; nothing is
compiled. Wheels use the stable ABI (one per platform for CPython 3.12 and
later) and cover macOS and Linux on x86_64 and arm64. See
[python/README.md](../python/README.md) for the API.

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
