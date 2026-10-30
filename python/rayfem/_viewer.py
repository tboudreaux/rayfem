"""Locate the viewer executable shipped inside the wheel."""
from __future__ import annotations

import os
from pathlib import Path


def viewer_executable() -> str | None:
    """The bundled viewer, if this installation has one.

    ``rayfem.init`` falls back to ``RAYFEM_EXECUTABLE`` and then ``PATH``
    when this returns None (for example in a source build without a viewer).
    """
    if os.environ.get("RAYFEM_EXECUTABLE"):
        return None  # the launcher honours the environment variable itself
    candidate = Path(__file__).resolve().parent / "bin" / "RayFEM"
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return str(candidate)
    return None
