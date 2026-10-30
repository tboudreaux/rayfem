# rayfem

Stream finite element meshes and fields to the RayFEM viewer from Python.
The wheel includes the viewer, so nothing needs to be built:

```python
import numpy as np
import rayfem
from rayfem import mesh

rayfem.init()  # starts the bundled viewer, or reuses one already running

# A unit square of two triangles, with a value at each vertex.
vertices = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
triangles = np.array([[0, 1, 2], [0, 2, 3]])
with rayfem.Client() as client:
    client.send_mesh(1, mesh.mesh_text(vertices, triangles, "triangle"))
    for step in range(10):
        u = np.sin(vertices[:, 0] + 0.3*step)
        client.send_frame(1, [rayfem.Field("u", mesh.vertex_space(2), u)], index=step, time=0.1*step)
```

## The `rayfem` command

Installing the wheel also installs a `rayfem` command (or `python -m rayfem`)
that runs the viewer in the foreground:

```sh
rayfem                          # the first free port pair from 5555, on 127.0.0.1
rayfem --port 6000              # stream port 6000, control port 6001
rayfem --host 0.0.0.0           # every interface, without authentication
rayfem --light run.serif        # light mode, opening an archive
```

It prints the address to stream to and waits. Ctrl-C (or SIGTERM, or closing
the terminal) closes the viewer normally; a second Ctrl-C, or a viewer that
has not closed after ten seconds, ends it. `rayfem --help` lists the window
options; `--verbose` shows the viewer's graphics log.

## Data model

The Python API shares no C++ types with other packages, so it works beside
PyMFEM, SERiF or anything else, whatever MFEM build they use:

- A **mesh** is MFEM mesh text. `rayfem.mesh.mesh_text` builds it from
  vertices and connectivity; `rayfem.adapters.pymfem.mesh_text` from an
  `mfem.Mesh`.
- A **field** is the text MFEM's `FiniteElementSpace::Save` writes, plus a
  float64 array of coefficients in that space's order (`rayfem.Field`).
  `rayfem.mesh.vertex_space` and `element_space` cover values at vertices and
  per element; `rayfem.mesh.space_header` any MFEM collection.
- A **displacement** is a vector field of offsets from the registered mesh.

## Describing frames

Register a schema to send coordinates (such as an iteration count), values
shown with each frame (such as a virial ratio) and tags (such as
`converged`); the viewer labels, filters and plots frames by them:

```python
client.register_schema(rayfem.Schema(
    values=[rayfem.Value("iteration", "int", coordinate=True), rayfem.Value("residual", "float")],
    tags=[rayfem.Tag("converged")], has_time=False, primary="iteration"))
client.send_frame(1, fields, index=n, values={"iteration": n, "residual": r},
                  tags=["converged"] if done else [])
```

Sending is one-way, but a send raises `RayFEMError` when no viewer is
listening at the client's address (it closed, or moved to another host or
port from its info menu), and a frame raises when the viewer's listener
restarted since its mesh was sent; send the mesh again then.
`client.connected` reports whether a viewer is connected now.
`rayfem.stats()` reports how many frames the viewer has decoded and
rejected, to confirm a stream arrived. Using the client leaves Ctrl-C to
your program: RayFEM keeps the process's own SIGINT and SIGTERM handlers. `rayfem.shutdown()`
closes the viewer. Wheels are abi3 (CPython 3.12 and
later) for macOS and Linux on x86_64 and arm64.
