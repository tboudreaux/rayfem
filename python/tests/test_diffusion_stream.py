"""A viewer started from Python receives a 2D diffusion solve streamed from C++.

    python python/tests/test_diffusion_stream.py BUILD/tests/diffusion_stream

rayfem.init() starts the viewer bundled with the installed package; the
MFEM program (tests/diffusion_stream.cpp) solves the heat equation, checks
its error against the exact solution, and streams every step. The viewer's
counters (rayfem.stats) then confirm every frame was decoded and none was
rejected. Needs a display for the viewer window.
"""
from __future__ import annotations

import socket
import subprocess
import sys
import time

import rayfem

STEPS = 40


def free_port() -> int:
    """A free port whose successor (the control port) is free too."""
    for _ in range(50):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", port + 1))
                return port
            except OSError:
                continue
    raise RuntimeError("No free port pair")


def main(solver: str) -> None:
    port = free_port()
    status = rayfem.init(port, timeout=20)
    assert status == rayfem.InitStatus.Started, status
    try:
        before = rayfem.stats(port)
        assert before.received == 0 and before.errors == 0, before
        run = subprocess.run([solver, "--port", str(port), "--steps", str(STEPS), "--pace", "0.05"],
                             capture_output=True, text=True, timeout=300)
        print(run.stdout.strip())
        assert run.returncode == 0, run.stdout + run.stderr
        deadline = time.monotonic() + 60
        while True:
            counters = rayfem.stats(port)
            if counters.received >= STEPS + 1 or time.monotonic() > deadline:
                break
            time.sleep(0.2)
        print(f"viewer decoded {counters.received} frames; {counters.errors} rejected, "
              f"{counters.dropped} dropped, {counters.skipped} skipped")
        assert counters.received == STEPS + 1 and counters.errors == 0 and counters.dropped == 0, counters
    finally:
        assert rayfem.shutdown(port, timeout=20)
    print("diffusion stream to a Python-started viewer ok")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
