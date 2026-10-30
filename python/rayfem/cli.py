"""The ``rayfem`` command: run the bundled viewer in the foreground.

    rayfem                          # first free port pair from 5555, on 127.0.0.1
    rayfem --port 6000              # stream port 6000, control port 6001
    rayfem --host 0.0.0.0 --light   # every interface (no authentication)
    rayfem run.serif                # open an archive

It prints where clients should stream, then waits. Ctrl-C (or SIGTERM or a
closed terminal) asks the viewer to close normally, as its window's close
button does; a second Ctrl-C, or a viewer that does not close within the
grace period, ends it forcibly. The exit status is the viewer's.
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import time

from . import RayFEMError, stats
from ._viewer import viewer_executable

DEFAULT_PORT = 5555
SEARCH = 200  # ports tried from DEFAULT_PORT when none is given
GRACE = 10.0  # seconds a viewer has to close before it is killed


def _free(host: str, port: int) -> bool:
    """Whether a TCP listener could bind host:port now."""
    try:
        infos = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM, flags=socket.AI_PASSIVE)
    except socket.gaierror:
        return False
    for family, kind, proto, _, address in infos:
        with socket.socket(family, kind, proto) as probe:
            # As libzmq binds: a port in TIME_WAIT is still usable.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(address)
                probe.listen(1)
            except OSError:
                return False
    return True


def find_ports(host: str, port: int | None, control: int | None) -> tuple[int, int, list[int]]:
    """The stream and control ports to use, and the stream ports skipped as busy.

    A given port is used as is (its control port defaults to port + 1); with
    none, the first pair from DEFAULT_PORT with both ports free is chosen.
    """
    if port is not None:
        control = control if control is not None else port + 1
        for p in (port, control):
            if not _free(host, p):
                raise RayFEMError(f"Port {p} on {host} is in use; choose another or omit --port to find a free one")
        return port, control, []
    skipped = []
    for candidate in range(DEFAULT_PORT, DEFAULT_PORT + SEARCH):
        pair = control if control is not None else candidate + 1
        if candidate != pair and pair <= 65535 and _free(host, candidate) and _free(host, pair):
            return candidate, pair, skipped
        skipped.append(candidate)
    raise RayFEMError(f"No free port pair on {host} between {DEFAULT_PORT} and {DEFAULT_PORT + SEARCH - 1}")


def _client_host(host: str) -> str:
    """An address to reach a listener bound to host from this machine."""
    return {"0.0.0.0": "127.0.0.1", "*": "127.0.0.1", "::": "::1"}.get(host, host)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rayfem", description="Run the RayFEM viewer and print where to stream to. Ctrl-C closes it.")
    parser.add_argument("archive", nargs="?", help="a .serif archive to open")
    parser.add_argument("--host", "--listen", dest="host", default="127.0.0.1",
                        help="address to listen on (default 127.0.0.1, this machine only; "
                             "0.0.0.0 or :: listen on every interface without authentication)")
    parser.add_argument("--port", type=int, help=f"stream port (default: the first free pair from {DEFAULT_PORT})")
    parser.add_argument("--control-port", type=int,
                        help="port for init, stats and shutdown (default: stream port + 1)")
    parser.add_argument("--light", action="store_true", help="start in light mode")
    parser.add_argument("--history", type=int, help="maximum retained frames")
    parser.add_argument("--fps", type=int, help="maximum display rate")
    parser.add_argument("--width", type=int, help="window width")
    parser.add_argument("--height", type=int, help="window height")
    parser.add_argument("--verbose", action="store_true", help="show the viewer's graphics log")
    parser.add_argument("--executable", help="viewer to run instead of the bundled one")
    return parser


def _stop(viewer: subprocess.Popen, grace: float) -> int:
    """Ask the viewer to close, then end it if it does not."""
    if viewer.poll() is None:
        # The viewer treats SIGINT as its close button: it leaves its main
        # loop and releases the window and its ports.
        viewer.send_signal(signal.SIGINT)
        try:
            return viewer.wait(grace)
        except subprocess.TimeoutExpired:
            print(f"rayfem: the viewer did not close within {grace:g} s; ending it", file=sys.stderr)
            viewer.kill()
    return viewer.wait()


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    for name in ("port", "control_port"):
        value = getattr(args, name)
        if value is not None and not 1 <= value <= 65535:
            print(f"rayfem: --{name.replace('_', '-')} must be between 1 and 65535", file=sys.stderr)
            return 2
    executable = args.executable or os.environ.get("RAYFEM_EXECUTABLE") or viewer_executable()
    if not executable:
        print("rayfem: this installation has no bundled viewer; set RAYFEM_EXECUTABLE or pass --executable",
              file=sys.stderr)
        return 2
    try:
        port, control, skipped = find_ports(args.host, args.port, args.control_port)
    except RayFEMError as error:
        print(f"rayfem: {error}", file=sys.stderr)
        return 1
    command = [executable, "--listen", args.host, "--port", str(port), "--control-port", str(control)]
    for option in ("history", "fps", "width", "height"):
        if getattr(args, option) is not None:
            command += [f"--{option}", str(getattr(args, option))]
    if args.light:
        command.append("--light")
    if args.archive:
        command.append(args.archive)

    if skipped:
        busy = f"Port {skipped[0]} is" if len(skipped) == 1 else f"Ports {skipped[0]}-{skipped[-1]} are"
        print(f"{busy} in use; using the next free pair.")
    print(f"Starting RayFEM on {args.host}:{port} (control port {control})")
    # Its own session: Ctrl-C reaches only this process, which closes the
    # viewer once instead of both receiving the signal.
    viewer = subprocess.Popen(command, start_new_session=True,
                              stdout=None if args.verbose else subprocess.DEVNULL)
    requests = []
    def request(signum, _frame):
        requests.append(signum)
        if len(requests) > 1 and viewer.poll() is None:
            viewer.kill()  # a second Ctrl-C does not wait
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request)

    try:
        client = _client_host(args.host)
        deadline = time.monotonic() + 30
        while viewer.poll() is None:
            try:
                stats(port, client, control_port=control, timeout=0.5)
                break
            except RayFEMError:
                if time.monotonic() > deadline:
                    break
                time.sleep(0.1)
        if viewer.poll() is not None:
            print(f"rayfem: the viewer exited during startup (status {viewer.returncode})", file=sys.stderr)
            return viewer.returncode or 1
        print(f"RayFEM is listening on {args.host}:{port} (control port {control}).")
        print(f"  Python: rayfem.Client(host={client!r}, port={port})")
        print(f"  C++:    RayFEM::Client client({client!r}, {port});".replace("'", '"'))
        print("Press Ctrl-C to close it.")
        sys.stdout.flush()
        return viewer.wait()
    except KeyboardInterrupt:
        print("\nClosing RayFEM...")
        try:
            status = _stop(viewer, GRACE)
        except KeyboardInterrupt:
            status = _stop(viewer, 0)
        # SIGINT's normal close exits with 0.
        return status if status >= 0 else 128 - status


if __name__ == "__main__":
    sys.exit(main())
