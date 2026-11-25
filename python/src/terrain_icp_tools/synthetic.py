"""Synthetic terrain (sum of Gaussian hills) for tests and demos. No real site is modelled anywhere."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rasterio
from rasterio.transform import from_origin

from terrain_icp_tools import plyio

# Same hills as tests/synthetic_terrain.hpp: (x, y, height, sigma) in metres.
HILLS = [
    (-40.0, 30.0, 18.0, 25.0),
    (35.0, 45.0, 12.0, 18.0),
    (10.0, -35.0, 22.0, 30.0),
    (-55.0, -50.0, 9.0, 15.0),
    (60.0, -10.0, -10.0, 20.0),
    (-5.0, 5.0, 6.0, 10.0),
]

# A generic transverse Mercator on the WGS84 ellipsoid (not any real projection zone): projected, in metres.
LOCAL_CRS = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"


def height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    z = np.zeros(np.broadcast(x, y).shape)
    for hx, hy, h, sigma in HILLS:
        z += h * np.exp(-((x - hx) ** 2 + (y - hy) ** 2) / (2.0 * sigma**2))
    return z


def write_dem(path: Path, origin: tuple[float, float, float], half: float = 100.0, spacing: float = 1.0) -> None:
    """GeoTIFF of the terrain over [-half, half]^2 around `origin` (E, N, h), float32, one NaN-free nodata corner."""
    e0, n0, h0 = origin
    n = int(round(2.0 * half / spacing))
    centres = -half + spacing * (np.arange(n) + 0.5)  # pixel centres, local coordinates
    x, y = np.meshgrid(centres, centres[::-1])  # row 0 is the northern edge
    z = (h0 + height(x, y)).astype(np.float32)
    z[:3, :3] = -9999.0  # a nodata patch, to exercise the masking
    transform = from_origin(e0 - half, n0 + half, spacing, spacing)
    with rasterio.open(path, "w", driver="GTiff", height=n, width=n, count=1, dtype="float32", crs=LOCAL_CRS,
                       transform=transform, nodata=-9999.0) as dst:
        dst.write(z, 1)


def rotation_z(yaw_deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def mesh_points(count: int = 6000, half: float = 70.0, noise: float = 0.02, scale: float = 1.0, yaw_deg: float = 0.0,
                translation=(0.0, 0.0, 0.0), vertical_scale: float = 1.0, seed: int = 42) -> np.ndarray:
    """Random surface samples in the moving frame: x_moving = truth^-1(x_surface), x_ref = s R x_moving + t.

    `vertical_scale` stretches the sampled heights first (a mesh whose heights are exaggerated by that factor)."""
    rng = np.random.default_rng(seed)
    x, y = rng.uniform(-half, half, (2, count))
    z = vertical_scale * height(x, y) + rng.normal(0.0, noise, count)
    surface = np.column_stack([x, y, z])
    r = rotation_z(yaw_deg)
    return (surface - np.asarray(translation)) @ r / scale  # row-vector form of R^T (q - t) / s


def write_scene(out: Path, origin: tuple[float, float, float], scale: float = 1.005, yaw_deg: float = 2.0,
                translation=(2.0, -2.0, 1.0), vertical_scale: float = 1.0, noise: float = 0.02) -> dict:
    """dem.tif and mesh.ply (both georeferenced around `origin`) plus truth.json in `out`."""
    out.mkdir(parents=True, exist_ok=True)
    write_dem(out / "dem.tif", origin)
    local = mesh_points(scale=scale, yaw_deg=yaw_deg, translation=translation, vertical_scale=vertical_scale,
                        noise=noise)
    plyio.write_points(out / "mesh.ply", local + np.asarray(origin), double=True)
    truth = {"scale": scale, "rotation_deg": {"yaw": yaw_deg, "pitch": 0.0, "roll": 0.0},
             "translation_m": list(translation), "vertical_scale": vertical_scale, "noise_m": noise,
             "origin": list(origin)}
    (out / "truth.json").write_text(json.dumps(truth, indent=2) + "\n")
    return truth
