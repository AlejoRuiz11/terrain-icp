# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
"""DEM GeoTIFF -> PLY point cloud in local metres around a chosen origin."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rasterio

from terrain_icp_tools import plyio


def dem_points(path: Path, origin: tuple[float, float, float], step: int = 1) -> np.ndarray:
    """Pixel centres of band 1 as (E - E0, N - N0, h - h0), keeping every `step`-th row and column; nodata dropped.

    The DEM must be in a projected CRS with metre units, so that E, N and h share the same unit."""
    if step < 1:
        raise ValueError("step must be >= 1")
    with rasterio.open(path) as src:
        if src.crs is None or not src.crs.is_projected:
            raise ValueError(f"{Path(path).name}: the DEM needs a projected CRS in metres (reproject it first)")
        unit, factor = src.crs.linear_units_factor
        if abs(factor - 1.0) > 1e-12:
            raise ValueError(f"{Path(path).name}: CRS unit is '{unit}', metres expected")
        band = src.read(1, masked=True)[::step, ::step]
        rows = np.arange(0, src.height, step)
        cols = np.arange(0, src.width, step)
        c, r = np.meshgrid(cols + 0.5, rows + 0.5)  # +0.5: pixel centre
        a = src.transform
        e = a.a * c + a.b * r + a.c
        n = a.d * c + a.e * r + a.f

    valid = ~np.ma.getmaskarray(band) & np.isfinite(band.filled(np.nan))
    points = np.column_stack([e[valid], n[valid], band.data[valid].astype(np.float64)])
    return points - np.asarray(origin, dtype=np.float64)


def dem_to_ply(dem: Path, out: Path, origin: tuple[float, float, float], step: int = 1) -> int:
    points = dem_points(dem, origin, step)
    plyio.write_points(out, points)
    return len(points)
