"""Stream finite element meshes and fields to the RayFEM viewer.

RayFEM's Python API shares no C++ types with other packages. A mesh is MFEM
mesh text, and a field is the text of its finite element space plus a float64
array of coefficients. Any producer (PyMFEM, SERiF, or plain NumPy through
:mod:`rayfem.mesh`) can send to the viewer, whatever MFEM build it uses.

    import rayfem
    from rayfem import mesh

    rayfem.init()                              # start the bundled viewer
    client = rayfem.Client()
    client.send_mesh(1, mesh.mesh_text(vertices, triangles, "triangle"))
    client.send_frame(1, [rayfem.Field("u", mesh.vertex_space(2), u)], index=0, time=0.0)
"""
from __future__ import annotations

import array
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Union

from . import _rayfem
from ._rayfem import InitStatus, ShutdownStatus
from ._viewer import viewer_executable

__version__ = "0.1.0"
__all__ = [
    "Client", "Field", "InitStatus", "RayFEMError", "Schema", "ShutdownStatus", "Tag", "Value",
    "ViewerStats", "init", "shutdown", "stats", "viewer_executable",
]


class RayFEMError(RuntimeError):
    """A viewer could not be started or stopped, or a send was refused."""


def init(port: int = 5555, host: str = "127.0.0.1", *, control_port: int = 0, width: int = 1440,
         height: int = 900, fps: int = 60, history: int = 120, timeout: float = 5.0,
         executable: str | None = None) -> InitStatus:
    """Start a viewer, or verify one already listening on ``host:port``.

    The viewer bundled in the wheel is used unless ``executable`` or the
    ``RAYFEM_EXECUTABLE`` environment variable names another. Window options
    apply only when a viewer is started. Returns ``InitStatus.Started`` or
    ``InitStatus.AlreadyRunning``; raises :class:`RayFEMError` otherwise.
    """
    path = executable or viewer_executable() or ""
    status, _, error = _rayfem.init(host, port, control_port, width, height, fps, history,
                                    int(timeout * 1000), path)
    if status == InitStatus.Failed:
        raise RayFEMError(error)
    return status


def shutdown(port: int = 5555, host: str = "127.0.0.1", *, control_port: int = 0,
             timeout: float = 5.0) -> bool:
    """Ask the viewer at ``host:port`` to close. True if it stopped, False if none was running."""
    status, error = _rayfem.shutdown(host, port, control_port, int(timeout * 1000))
    if status == ShutdownStatus.Failed:
        raise RayFEMError(error)
    return status == ShutdownStatus.Stopped


@dataclass(frozen=True)
class ViewerStats:
    """A viewer's counters since it started (see :func:`stats`)."""
    received: int  # frames fully decoded
    errors: int    # packets or frames rejected
    dropped: int   # frames dropped from a full queue
    skipped: int   # frames skipped while receiving was paused


def stats(port: int = 5555, host: str = "127.0.0.1", *, control_port: int = 0, timeout: float = 2.0) -> ViewerStats:
    """Ask the viewer at ``host:port`` how many frames it has decoded and rejected.

    Sending is one-way, so this is how to confirm frames arrived.
    """
    received, errors, dropped, skipped, error = _rayfem.stats(host, port, control_port, int(timeout * 1000))
    if error:
        raise RayFEMError(error)
    return ViewerStats(received, errors, dropped, skipped)


@dataclass(frozen=True)
class Value:
    """A value frames carry. ``type`` is ``"int"``, ``"float"`` or ``"str"``.

    A coordinate (int or float) is a progress axis, such as an iteration
    count, that the viewer can label frames by, filter on and plot against.
    """
    name: str
    type: str = "float"
    coordinate: bool = False
    unit: str = ""
    description: str = ""


@dataclass(frozen=True)
class Tag:
    """A kind of frame, such as ``"converged"``."""
    name: str
    description: str = ""


@dataclass
class Schema:
    """What frames will carry, registered once with :meth:`Client.register_schema`.

    ``has_time=False`` says simulation time means nothing (a static solve);
    ``primary`` names the coordinate frames are labelled by.
    """
    values: Sequence[Value] = field(default_factory=list)
    tags: Sequence[Tag] = field(default_factory=list)
    has_time: bool = True
    primary: str = ""


def _float64(values: Any) -> Any:
    """A contiguous float64 vector the extension can read in place (copied only if needed)."""
    try:
        import numpy  # noqa: PLC0415 - optional
    except ImportError:
        numpy = None
    if numpy is not None and not isinstance(values, array.array):
        return numpy.ascontiguousarray(values, dtype=numpy.float64).reshape(-1)
    try:
        view = memoryview(values)
    except TypeError:
        return array.array("d", values)
    if view.format == "d" and view.ndim == 1 and view.c_contiguous:
        return values
    return array.array("d", view.tolist() if view.ndim == 1 else [x for row in view.tolist() for x in row])


@dataclass
class Field:
    """A named field: the text of its finite element space and its coefficients.

    ``space`` is what MFEM's ``FiniteElementSpace::Save`` writes (see
    :func:`rayfem.mesh.vertex_space` and :func:`rayfem.mesh.space_header`).
    ``values`` is any sequence or array of floats, in that space's DOF order.
    """
    name: str
    space: str
    values: Any

    def _wire(self) -> tuple[str, str, Any]:
        return (self.name, self.space, _float64(self.values))


FieldLike = Union[Field, tuple[str, str, Any]]


def _field(item: FieldLike) -> tuple[str, str, Any]:
    if isinstance(item, Field):
        return item._wire()
    name, space, values = item
    return (name, space, _float64(values))


class Client:
    """Sends meshes and frames to a viewer at ``host:port``.

    Sends are queued for the viewer; a refused send (invalid input, or a
    viewer that stopped receiving) raises :class:`RayFEMError`. The viewer
    validates everything it decodes and reports problems in its window.
    """

    def __init__(self, host: str = "localhost", port: int = 5555, *, binary: bool = True) -> None:
        self._core: _rayfem.StreamClient | None = _rayfem.StreamClient(host, port)
        self._core.binary_fields = binary

    def _client(self) -> _rayfem.StreamClient:
        if self._core is None:
            raise RayFEMError("The client is closed")
        return self._core

    def register_schema(self, schema: Schema) -> None:
        """Declare the values and tags frames will carry; registering again replaces it."""
        values = [(v.name, v.type, v.coordinate, v.unit, v.description) for v in schema.values]
        tags = [(t.name, t.description) for t in schema.tags]
        try:
            self._client().register_schema(values, tags, schema.has_time, schema.primary)
        except (RuntimeError, ValueError, TypeError) as error:
            raise RayFEMError(str(error)) from error

    def send_mesh(self, mesh_id: int, mesh: str | bytes) -> None:
        """Register MFEM mesh text under ``mesh_id``; frames then refer to it."""
        text = mesh.encode() if isinstance(mesh, str) else bytes(mesh)
        try:
            self._client().send_mesh(mesh_id, text)
        except RuntimeError as error:
            raise RayFEMError(str(error)) from error

    def send_frame(self, mesh_id: int, fields: Iterable[FieldLike], *, index: int, time: float = 0.0,
                   label: str = "", values: Mapping[str, int | float | str] | None = None,
                   tags: Iterable[str] = (), displacement: FieldLike | None = None) -> None:
        """Send one frame: every field of one state, with increasing ``index`` per mesh.

        ``values`` and ``tags`` must be declared in the registered schema.
        ``displacement`` is a vector field of offsets from the registered mesh.
        """
        try:
            self._client().send_frame(mesh_id, [_field(f) for f in fields], index, float(time), label,
                                      dict(values or {}), list(tags),
                                      _field(displacement) if displacement is not None else None)
        except (RuntimeError, ValueError, TypeError) as error:
            raise RayFEMError(str(error)) from error

    def close(self) -> None:
        """Release the connection; queued packets are flushed for up to a second."""
        self._core = None

    def __enter__(self) -> Client:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
