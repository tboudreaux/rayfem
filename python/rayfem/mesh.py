"""Build MFEM mesh text and finite element space headers from plain arrays.

This lets code without MFEM visualize data in RayFEM: describe the mesh by its
vertices and element connectivity, then send values at vertices
(:func:`vertex_space`) or per element (:func:`element_space`).

Element vertices follow MFEM's ordering: counter-clockwise for triangles and
quadrilaterals; for a hexahedron the bottom face counter-clockwise, then the
top face in the same order.
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import Any

# MFEM's Geometry::Type numbers and dimensions.
GEOMETRIES: dict[str, tuple[int, int, int]] = {
    # name: (geometry id, dimension, vertices)
    "segment": (1, 1, 2),
    "triangle": (2, 2, 3),
    "quadrilateral": (3, 2, 4),
    "tetrahedron": (4, 3, 4),
    "hexahedron": (5, 3, 8),
    "prism": (6, 3, 6),
    "pyramid": (7, 3, 5),
}
ALIASES = {"quad": "quadrilateral", "square": "quadrilateral", "tet": "tetrahedron", "hex": "hexahedron",
           "cube": "hexahedron", "wedge": "prism", "line": "segment"}


def _geometry(name: str) -> tuple[int, int, int]:
    key = ALIASES.get(name.lower(), name.lower())
    if key not in GEOMETRIES:
        raise ValueError(f"Unknown geometry {name!r}; use one of {sorted(GEOMETRIES)}")
    return GEOMETRIES[key]


def _rows(data: Any) -> list[list[float]]:
    rows = data.tolist() if hasattr(data, "tolist") else [list(row) for row in data]
    return [list(row) if isinstance(row, (list, tuple)) else [row] for row in rows]


def mesh_text(vertices: Any, elements: Any, geometry: str, *, attributes: Sequence[int] | None = None,
              boundary: Any = None, boundary_geometry: str | None = None,
              boundary_attributes: Sequence[int] | None = None) -> str:
    """MFEM mesh text ("MFEM mesh v1.0") for one element type.

    ``vertices`` is (vertex count, space dimension); ``elements`` is (element
    count, vertices per element) of zero-based vertex indices. ``attributes``
    are element material IDs (default 1). The boundary is optional: RayFEM
    finds the outer surface from the elements themselves.
    """
    geometry_id, dimension, corners = _geometry(geometry)
    points = _rows(vertices)
    cells = [[int(v) for v in row] for row in _rows(elements)]
    if not points or not cells:
        raise ValueError("A mesh needs at least one vertex and one element")
    space_dimension = len(points[0])
    if not dimension <= space_dimension <= 3 or any(len(p) != space_dimension for p in points):
        raise ValueError(f"Vertices must all have {dimension} to 3 coordinates")
    for cell in cells:
        if len(cell) != corners:
            raise ValueError(f"A {geometry} has {corners} vertices, not {len(cell)}")
        if any(v < 0 or v >= len(points) for v in cell):
            raise ValueError("Element vertex indices must refer to the vertex list")
    attributes = list(attributes) if attributes is not None else [1]*len(cells)
    if len(attributes) != len(cells) or any(int(a) < 1 for a in attributes):
        raise ValueError("One positive attribute per element")
    lines = ["MFEM mesh v1.0", "", "dimension", str(dimension), "", "elements", str(len(cells))]
    lines += [f"{int(a)} {geometry_id} " + " ".join(map(str, cell)) for a, cell in zip(attributes, cells)]
    faces = [] if boundary is None else [[int(v) for v in row] for row in _rows(boundary)]
    lines += ["", "boundary", str(len(faces))]
    if faces:
        if boundary_geometry is None:
            raise ValueError("Give boundary_geometry with boundary elements")
        face_id = _geometry(boundary_geometry)[0]
        face_attributes = list(boundary_attributes) if boundary_attributes is not None else [1]*len(faces)
        lines += [f"{int(a)} {face_id} " + " ".join(map(str, face)) for a, face in zip(face_attributes, faces)]
    lines += ["", "vertices", str(len(points)), str(space_dimension)]
    lines += [" ".join(repr(float(x)) for x in point) for point in points]
    return "\n".join(lines) + "\n"


def space_header(collection: str, vdim: int = 1, ordering: str = "nodes") -> str:
    """What MFEM's ``FiniteElementSpace::Save`` writes for a collection such as ``"H1_3D_P2"``.

    ``ordering`` is ``"nodes"`` (all of component 0, then 1, ...) or
    ``"vdim"`` (components interleaved per DOF). NURBS and variable-order
    spaces need the full MFEM text instead.
    """
    if ordering not in ("nodes", "vdim"):
        raise ValueError("ordering is 'nodes' or 'vdim'")
    if vdim < 1:
        raise ValueError("vdim must be positive")
    return (f"FiniteElementSpace\nFiniteElementCollection: {collection}\nVDim: {vdim}\n"
            f"Ordering: {0 if ordering == 'nodes' else 1}\n\n")


def vertex_space(dimension: int, vdim: int = 1, ordering: str = "nodes") -> str:
    """Values at the vertices of a mesh from :func:`mesh_text`, in vertex order.

    These are linear (H1, order 1) fields. For a vector (``vdim`` > 1), such as a
    displacement with one component per coordinate, ``ordering="vdim"`` takes
    values as (vertex count, vdim) flattened row by row.
    """
    return space_header(f"H1_{dimension}D_P1", vdim, ordering)


def element_space(dimension: int, vdim: int = 1, ordering: str = "nodes") -> str:
    """One constant value per element (L2, order 0), in element order."""
    return space_header(f"L2_{dimension}D_P0", vdim, ordering)
