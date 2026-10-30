"""Checks for an installed rayfem package; runs without a display.

    python python/tests/test_wheel.py

With RAYFEM_STREAM_PROBE set to the stream_probe test program (built with
the C++ tests), frames are also sent to it and checked as the server decodes
them. With RAYFEM_TEST_VIEWER=1 (and a display), the bundled viewer is
started, streamed to and shut down.
"""
from __future__ import annotations

import array
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import rayfem
from rayfem import mesh


def free_port() -> int:
    """A free port whose successor (the control port) is free too."""
    for _ in range(50):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        if port < 65535:
            with socket.socket() as s:
                try:
                    s.bind(("127.0.0.1", port + 1))
                    return port
                except OSError:
                    continue
    raise RuntimeError("No free port pair")


def expect_error(function, *args, **kwargs) -> str:
    try:
        function(*args, **kwargs)
    except rayfem.RayFEMError as error:
        return str(error)
    raise AssertionError(f"{function.__name__} should have raised RayFEMError")


def square():
    vertices = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
    return vertices, mesh.mesh_text(vertices, [[0, 1, 2], [0, 2, 3]], "triangle", attributes=[1, 2])


def test_package() -> None:
    import rayfem._rayfem as core
    suffix = Path(core.__file__).name
    assert ".abi3." in suffix, f"extension is not built for the stable ABI: {suffix}"
    viewer = rayfem.viewer_executable()
    if viewer is not None:  # wheels ship it; a source build may not
        result = subprocess.run([viewer, "--help"], capture_output=True, text=True, timeout=30)
        assert result.returncode == 0 and "--port" in result.stdout, result.stdout + result.stderr
    print(f"package ok: {suffix}; viewer {viewer}")


def test_mesh_text() -> None:
    _, text = square()
    assert text.startswith("MFEM mesh v1.0\n") and "\nelements\n2\n1 2 0 1 2\n2 2 0 2 3\n" in text
    assert "\nvertices\n4\n2\n" in text
    assert mesh.vertex_space(2) == "FiniteElementSpace\nFiniteElementCollection: H1_2D_P1\nVDim: 1\nOrdering: 0\n\n"
    assert "VDim: 3\nOrdering: 1" in mesh.vertex_space(3, vdim=3, ordering="vdim")
    assert "L2_3D_P0" in mesh.element_space(3)
    for bad in (lambda: mesh.mesh_text([[0, 0]], [[0, 1, 2]], "triangle"),
                lambda: mesh.mesh_text([[0, 0], [1, 0], [0, 1]], [[0, 1]], "triangle"),
                lambda: mesh.mesh_text([[0, 0]], [[0]], "polygon")):
        try:
            bad()
        except ValueError:
            continue
        raise AssertionError("invalid mesh input was accepted")
    print("mesh text ok")


def test_client_refusals() -> None:
    _, text = square()
    field = rayfem.Field("u", mesh.vertex_space(2), array.array("d", [0, 1, 2, 3]))
    # No viewer listens here: sends are refused rather than queued.
    port = free_port()
    client = rayfem.Client(port=port)
    assert not client.connected and client.address.endswith(f":{port}")
    assert "MFEM mesh" in expect_error(client.send_mesh, 1, "not a mesh")
    assert "No viewer is listening" in expect_error(client.send_mesh, 1, text)
    assert rayfem.init(port, timeout=20) == rayfem.InitStatus.Started
    try:
        deadline = time.monotonic() + 10
        while not client.connected and time.monotonic() < deadline:
            time.sleep(0.05)
        assert client.connected
        assert "sendMesh" in expect_error(client.send_frame, 1, [rayfem.Field("u", mesh.vertex_space(2), [0.0]*4)], index=0)
        client.send_mesh(1, text)
        client.send_frame(1, [field], index=0, time=0.5)
        assert "increase" in expect_error(client.send_frame, 1, [field], index=0)
        assert "non-finite" in expect_error(client.send_frame, 1, [rayfem.Field("u", mesh.vertex_space(2), [0, float("nan"), 0, 0])], index=1)
        assert "header" in expect_error(client.send_frame, 1, [rayfem.Field("u", "nonsense", [1.0])], index=1)
        assert "schema" in expect_error(client.send_frame, 1, [field], index=1, values={"iteration": 1})
        client.register_schema(rayfem.Schema(values=[rayfem.Value("iteration", "int", coordinate=True)], has_time=False,
                                             primary="iteration"))
        assert "bool" in expect_error(client.send_frame, 1, [field], index=1, values={"iteration": True})
        assert "not in the registered schema" in expect_error(client.send_frame, 1, [field], index=1, tags=["converged"])
        assert "coordinate" in expect_error(client.register_schema, rayfem.Schema(values=[rayfem.Value("x", "str", coordinate=True)])).lower()
        assert "nonzero" in expect_error(client.register_schema, rayfem.Schema(axes=[rayfem.Axis("spin", (0, 0, 0))]))
        assert "2 or 3" in expect_error(client.register_schema, rayfem.Schema(axes=[rayfem.Axis("spin", (1,))]))
        client.send_frame(1, [field], index=1, values={"iteration": 3})
        # Axes alone are a valid schema; frames then carry no values.
        client.register_schema(rayfem.Schema(axes=[rayfem.Axis("rotation", (0, 0, 1)), rayfem.Axis("magnetic", (0.2, 1), (0, 0, 0.1))]))
        client.send_frame(1, [field], index=2)
    finally:
        assert rayfem.shutdown(port, timeout=20)
    # The viewer closed: the next send says so.
    deadline = time.monotonic() + 10
    while client.connected and time.monotonic() < deadline:
        time.sleep(0.05)
    assert "No viewer is listening" in expect_error(client.send_frame, 1, [field], index=2, values={"iteration": 4})
    client.close()
    assert "closed" in expect_error(client.send_mesh, 1, text)
    print("client refusals ok")


def test_signals() -> None:
    # The client side must leave Ctrl-C to Python: CZMQ would otherwise
    # replace the SIGINT handler when the first socket is created.
    client = rayfem.Client(port=free_port())
    expect_error(rayfem.stats, free_port(), timeout=0.2)
    assert signal.getsignal(signal.SIGINT) is signal.default_int_handler
    try:
        os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)
        raise AssertionError("SIGINT did not raise KeyboardInterrupt after using the client")
    except KeyboardInterrupt:
        pass
    client.close()
    print("Ctrl-C still reaches Python")


def test_probe(probe: str) -> None:
    """Frames sent from Python arrive at the C++ server decoded exactly."""
    port = free_port()
    process = subprocess.Popen([probe, str(port), "3", "30"], stdout=subprocess.PIPE, text=True)
    assert process.stdout.readline().strip() == "READY"
    vertices, text = square()
    with rayfem.Client(port=port) as client:
        client.register_schema(rayfem.Schema(
            values=[rayfem.Value("iteration", "int", coordinate=True), rayfem.Value("virial ratio", "float"),
                    rayfem.Value("solver", "str")],
            tags=[rayfem.Tag("converged", "done")], has_time=False, primary="iteration"))
        client.send_mesh(7, text)
        for step in range(3):
            u = [float(step + i) for i in range(4)]
            moved = [0.01*step]*8  # one 2D offset per vertex, interleaved
            try:
                import numpy
                u = numpy.asarray(u)  # exercise the NumPy path too
            except ImportError:
                pass
            client.send_frame(7, [rayfem.Field("u", mesh.vertex_space(2), u),
                                  ("cells", mesh.element_space(2), [10.0, 20.0])],
                              index=step, time=0.0, label=f"step {step}",
                              values={"iteration": step + 1, "virial ratio": 1.0 + 0.1/(step + 1), "solver": "Newton"},
                              tags=["converged"] if step == 2 else [],
                              displacement=rayfem.Field("displacement", mesh.vertex_space(2, vdim=2, ordering="vdim"), moved))
    output, _ = process.communicate(timeout=60)
    assert process.returncode == 0, output
    frames = [line for line in output.splitlines() if line.startswith("FRAME")]
    assert len(frames) == 3, output
    last = frames[-1]
    for expected in ("index=2", "label=step 2", "elements=2", "fields=2", "field=u:4:14", "field=cells:2:30",
                     "displacement=8:0.02", "value=iteration:3", "value=solver:Newton", "tag=converged",
                     "has_time=0 primary=iteration"):
        assert expected in last, f"{expected!r} missing from {last}"
    # Doubles arrive bit for bit: the probe prints 17 significant digits.
    virial = float(last.split("value=virial ratio:")[1].split()[0])
    assert virial == 1.0 + 0.1/3, last
    print("end-to-end through the C++ server ok")


def test_viewer() -> None:
    port = free_port()
    assert rayfem.init(port, timeout=20) == rayfem.InitStatus.Started
    try:
        assert rayfem.init(port) == rayfem.InitStatus.AlreadyRunning
        _, text = square()
        with rayfem.Client(port=port) as client:
            client.send_mesh(1, text)
            for step in range(5):
                client.send_frame(1, [rayfem.Field("u", mesh.vertex_space(2), [step, 1, 2, 3])], index=step, time=step)
                time.sleep(0.1)
        deadline = time.monotonic() + 30
        while rayfem.stats(port).received < 5 and time.monotonic() < deadline:
            time.sleep(0.1)
        counters = rayfem.stats(port)
        assert counters.received == 5 and counters.errors == 0, counters
    finally:
        assert rayfem.shutdown(port, timeout=20)
    assert not rayfem.shutdown(port)
    assert "No RayFEM viewer" in expect_error(rayfem.stats, port)
    print("bundled viewer started, streamed to and shut down")


if __name__ == "__main__":
    test_package()
    test_mesh_text()
    test_client_refusals()
    test_signals()
    if os.environ.get("RAYFEM_STREAM_PROBE"):
        test_probe(os.environ["RAYFEM_STREAM_PROBE"])
    if os.environ.get("RAYFEM_TEST_VIEWER") == "1":
        test_viewer()
    print(f"rayfem {rayfem.__version__} on Python {sys.version.split()[0]}: all checks passed")
