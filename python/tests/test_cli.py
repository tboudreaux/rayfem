"""Checks for the ``rayfem`` command.

    python python/tests/test_cli.py

Port selection and the forced close run anywhere. With RAYFEM_TEST_VIEWER=1
(and a display), the bundled viewer is also run through the command: ports
chosen and printed, a busy port refused, frames streamed, and Ctrl-C and
SIGTERM closing it cleanly.
"""
from __future__ import annotations

import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

import rayfem
from rayfem import cli, mesh


def held(port: int, host: str = "127.0.0.1") -> socket.socket:
    listener = socket.socket()
    listener.bind((host, port))
    listener.listen(1)
    return listener


def free_base() -> int:
    """A stream port whose pair, and the next pair, are free."""
    for base in range(47000, 48000, 4):
        if all(cli._free("127.0.0.1", base + i) for i in range(4)):
            return base
    raise AssertionError("no free ports")


def test_ports() -> None:
    base = free_base()
    assert cli.find_ports("127.0.0.1", base, None) == (base, base + 1, [])
    assert cli.find_ports("127.0.0.1", base, base + 3) == (base, base + 3, [])
    with held(base + 1):
        try:
            cli.find_ports("127.0.0.1", base, None)
            raise AssertionError("a busy control port should be refused")
        except rayfem.RayFEMError as error:
            assert f"Port {base + 1}" in str(error)
    old = cli.DEFAULT_PORT
    cli.DEFAULT_PORT = base
    try:
        with held(base):
            port, control, skipped = cli.find_ports("127.0.0.1", None, None)
            assert (port, control) == (base + 1, base + 2) and skipped == [base], (port, control, skipped)
    finally:
        cli.DEFAULT_PORT = old
    assert not cli._free("no-such-host.invalid", base)
    print("port selection ok")


def test_forced_close() -> None:
    # A process that ignores SIGINT is ended once the grace period passes.
    stubborn = subprocess.Popen([sys.executable, "-c",
                                 "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(60)"])
    time.sleep(0.3)
    start = time.monotonic()
    status = cli._stop(stubborn, 1.0)
    assert status == -signal.SIGKILL and time.monotonic() - start < 5, status
    print("forced close ok")


def command() -> list[str]:
    script = shutil.which("rayfem", path=os.path.dirname(sys.executable))
    return [script] if script else [sys.executable, "-m", "rayfem"]


def start(*args: str) -> tuple[subprocess.Popen, str]:
    """Run the command until it reports the viewer ready; return its output so far."""
    process = subprocess.Popen(command() + list(args), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    output = ""
    deadline = time.monotonic() + 60
    while "Press Ctrl-C" not in output:
        line = process.stdout.readline()
        if not line or time.monotonic() > deadline:
            process.kill()
            raise AssertionError("rayfem did not report ready:\n" + output + process.stdout.read())
        output += line
    return process, output


def close(process: subprocess.Popen, signum: int) -> tuple[int, str, float]:
    start = time.monotonic()
    process.send_signal(signum)
    out, _ = process.communicate(timeout=30)
    return process.returncode, out, time.monotonic() - start


def test_command() -> None:
    base = free_base()
    first, output = start("--port", str(base))
    try:
        assert f"listening on 127.0.0.1:{base} (control port {base + 1})" in output, output
        assert f"rayfem.Client(host='127.0.0.1', port={base})" in output, output
        # A busy port is refused; auto selection skips past it.
        refused = subprocess.run(command() + ["--port", str(base)], capture_output=True, text=True, timeout=30)
        assert refused.returncode == 1 and f"Port {base} on 127.0.0.1 is in use" in refused.stderr, refused
        # Stream a frame and confirm it arrived.
        vertices = [[0, 0], [1, 0], [1, 1], [0, 1]]
        with rayfem.Client(port=base) as client:
            client.send_mesh(1, mesh.mesh_text(vertices, [[0, 1, 2], [0, 2, 3]], "triangle"))
            client.send_frame(1, [rayfem.Field("u", mesh.vertex_space(2), [0.0, 1, 2, 3])], index=0)
        deadline = time.monotonic() + 20
        while rayfem.stats(base).received < 1 and time.monotonic() < deadline:
            time.sleep(0.1)
        assert rayfem.stats(base).received == 1
    finally:
        status, out, seconds = close(first, signal.SIGINT)
    assert status == 0 and "Closing RayFEM" in out, (status, out)
    assert cli._free("127.0.0.1", base) and cli._free("127.0.0.1", base + 1), "ports released"
    print(f"Ctrl-C closed the viewer in {seconds:.1f} s and released its ports")

    # Automatic ports, on every interface, closed by SIGTERM.
    second, output = start("--host", "0.0.0.0")
    match = re.search(r"listening on 0\.0\.0\.0:(\d+) \(control port (\d+)\)", output)
    assert match and int(match.group(2)) == int(match.group(1)) + 1, output
    assert "host='127.0.0.1'" in output, output
    status, out, seconds = close(second, signal.SIGTERM)
    assert status == 0, (status, out)
    print(f"automatic ports {match.group(1)}/{match.group(2)} on 0.0.0.0; SIGTERM closed it in {seconds:.1f} s")


if __name__ == "__main__":
    test_ports()
    test_forced_close()
    if os.environ.get("RAYFEM_TEST_VIEWER") == "1":
        test_command()
    print("rayfem command: all checks passed")
