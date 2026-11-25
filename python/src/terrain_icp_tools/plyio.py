# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
"""Minimal PLY point I/O on top of plyfile: vertex x, y, z plus named scalar fields."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement


def read_vertices(path: Path) -> dict[str, np.ndarray]:
    """All scalar vertex properties as float64 arrays, keyed by name (faces and other elements are ignored)."""
    vertex = PlyData.read(str(path))["vertex"].data
    return {name: np.asarray(vertex[name], dtype=np.float64) for name in vertex.dtype.names}


def read_points(path: Path) -> np.ndarray:
    v = read_vertices(path)
    return np.column_stack([v["x"], v["y"], v["z"]])


def write_points(path: Path, xyz: np.ndarray, fields: dict[str, np.ndarray] | None = None,
                 double: bool = False) -> None:
    """Binary little-endian PLY. Coordinates as float32, or float64 with `double` (georeferenced coordinates)."""
    fields = fields or {}
    coord = "f8" if double else "f4"
    dtype = [("x", coord), ("y", coord), ("z", coord)] + [(name, "f4") for name in fields]
    data = np.empty(len(xyz), dtype=dtype)
    data["x"], data["y"], data["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    for name, values in fields.items():
        data[name] = values
    PlyData([PlyElement.describe(data, "vertex")], byte_order="<").write(str(path))
