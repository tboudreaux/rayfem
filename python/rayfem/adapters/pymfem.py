"""Send PyMFEM objects to RayFEM.

PyMFEM may link a different MFEM build than any other package in the
process, so nothing here passes C++ objects across: meshes are converted to
MFEM text and grid functions to their space header and coefficient array,
using only PyMFEM's public methods.

    from rayfem.adapters import pymfem as adapter
    client.send_mesh(1, adapter.mesh_text(mesh))
    client.send_frame(1, [adapter.field("u", gf)], index=step, time=t)
"""
from __future__ import annotations

import os
import tempfile
from typing import Any

from .. import Field
from ..mesh import space_header


def mesh_text(mesh: Any) -> str:
    """MFEM mesh text for an ``mfem.Mesh`` (curved and NURBS meshes included)."""
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "mesh.mesh")
        mesh.Print(path, 17)
        with open(path, encoding="utf-8") as stream:
            return stream.read()


def space(fespace: Any) -> str:
    """The space header of an ``mfem.FiniteElementSpace`` (not NURBS or variable order)."""
    if fespace.GetNURBSext() is not None:
        raise ValueError("NURBS spaces need MFEM's full space text; save it with fespace.Save")
    return space_header(fespace.FEColl().Name(), fespace.GetVDim(),
                        "vdim" if fespace.GetOrdering() == 1 else "nodes")


def field(name: str, gf: Any) -> Field:
    """A :class:`rayfem.Field` for an ``mfem.GridFunction`` (its coefficients, without copying)."""
    return Field(name, space(gf.FESpace()), gf.GetDataArray())
