"""Cross-check terrain-icp against CloudCompare's ICP on synthetic scenes (no real data involved).

    uv run --project python python scripts/compare_cloudcompare.py --exe build/msvc-release/terrain-icp.exe \
        [--cloudcompare path/to/CloudCompare.exe] [--out demo/compare]

For every scene the same local PLY files go to:
  - terrain-icp align --scale                      (point-to-plane, 10 % trimming)
  - terrain-icp-tools run --scale --vertical-scale (only in the height-exaggeration scene)
  - CloudCompare -ICP -ADJUST_SCALE on the DEM points  (point-to-point)
  - CloudCompare -ICP -ADJUST_SCALE -USE_C2M_DIST on the DEM triangulated (point-to-mesh)
CloudCompare runs with -OVERLAP 90 (same 10 % trimming as ours; the outlier scene adds 15 % / 85 % for both) and -MIN_ERROR_DIFF 1e-8 (run to convergence).

Tool-independent error: RMS over all moving points of |T_estimated(m) - x_true(m)|, where x_true is where the point
really belongs in the reference frame. Writes <out>/comparison.md and prints it.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import rasterio
from plyfile import PlyData, PlyElement

from terrain_icp_tools import cli, dem, plyio, synthetic

ORIGIN = (1000.0, 2000.0, 100.0)
TRUTH = {"scale": 1.005, "yaw_deg": 2.0, "translation": (2.0, -2.0, 1.0)}
SCENES = {  # name: (description, write_scene keyword arguments)
    "base": ("base: yaw 2°, t (2, -2, 1) m, scale 1.005, noise 2 cm, DEM 1 m", {}),
    "outliers": ("10 % of the mesh vertices lifted 3-20 m", {"outliers": 0.1}),
    "vertical": ("mesh heights exaggerated 3 %", {"vertical_scale": 1.03}),
    "dense_dem": ("base scene, DEM at 0.25 m instead of 1 m", {"dem_spacing": 0.25}),
}
CC_COMMON = ["-ADJUST_SCALE", "-MIN_ERROR_DIFF", "1e-8", "-RANDOM_SAMPLING_LIMIT", "1000000"]


def default_cloudcompare() -> str:
    for c in (Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/CloudCompare/CloudCompare.exe",
              Path("C:/Program Files/CloudCompare/CloudCompare.exe")):
        if c.is_file():
            return str(c)
    return "CloudCompare"


def write_dem_mesh(dem_path: Path, out: Path) -> None:
    """The DEM as a triangle mesh in local metres: two triangles per square of four valid pixel centres."""
    with rasterio.open(dem_path) as src:
        band = src.read(1, masked=True)
        a = src.transform
        rows, cols = band.shape
        c, r = np.meshgrid(np.arange(cols) + 0.5, np.arange(rows) + 0.5)
        x = a.a * c + a.b * r + a.c - ORIGIN[0]
        y = a.d * c + a.e * r + a.f - ORIGIN[1]
    valid = ~np.ma.getmaskarray(band)
    index = np.full(band.shape, -1, np.int64)
    index[valid] = np.arange(valid.sum())
    vertex = np.empty(valid.sum(), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])
    vertex["x"], vertex["y"], vertex["z"] = x[valid], y[valid], band.data[valid] - ORIGIN[2]

    i00, i01, i10, i11 = index[:-1, :-1], index[:-1, 1:], index[1:, :-1], index[1:, 1:]
    ok = (i00 >= 0) & (i01 >= 0) & (i10 >= 0) & (i11 >= 0)
    tris = np.concatenate([np.stack([i00[ok], i10[ok], i11[ok]], 1), np.stack([i00[ok], i11[ok], i01[ok]], 1)])
    face = np.empty(len(tris), dtype=[("vertex_indices", "i4", (3,))])
    face["vertex_indices"] = tris
    PlyData([PlyElement.describe(vertex, "vertex"), PlyElement.describe(face, "face")]).write(str(out))


def similarity_from_ypr(scale: float, yaw: float, pitch: float, roll: float, t) -> np.ndarray:
    cy, sy, cp, sp, cr, sr = (f(math.radians(v)) for v in (yaw, pitch, roll) for f in (math.cos, math.sin))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    m = np.eye(4)
    m[:3, :3] = scale * rz @ ry @ rx
    m[:3, 3] = t
    return m


def describe(m: np.ndarray) -> dict:
    """scale, yaw/pitch/roll (deg, ZYX) and translation of a 4x4 similarity."""
    a = m[:3, :3]
    s = float(np.cbrt(np.linalg.det(a)))
    r = a / s
    return {"scale": s, "yaw": math.degrees(math.atan2(r[1, 0], r[0, 0])),
            "pitch": math.degrees(math.asin(max(-1.0, min(1.0, -r[2, 0])))),
            "roll": math.degrees(math.atan2(r[2, 1], r[2, 2])), "t": m[:3, 3]}


def apply(m: np.ndarray, p: np.ndarray) -> np.ndarray:
    return p @ m[:3, :3].T + m[:3, 3]


def ours_matrix(results: Path) -> np.ndarray:
    t = json.loads(results.read_text())["transform"]
    m = np.eye(4)
    m[:3, :3] = t["scale"] * np.asarray(t["rotation_matrix"])
    m[:3, 3] = t["translation_m"]
    return m


def cloudcompare(cc: str, work: Path, reference: Path, extra: list[str],
                 overlap: int = 90) -> tuple[np.ndarray, float]:
    for f in work.glob("*_REGISTRATION_MATRIX_*.txt"):
        f.unlink()
    start = time.perf_counter()
    subprocess.run([cc, "-SILENT", "-AUTO_SAVE", "OFF", "-O", str(work / "moving.ply"), "-O", str(reference),
                    "-ICP", *CC_COMMON, "-OVERLAP", str(overlap), *extra], check=True, capture_output=True)
    elapsed = time.perf_counter() - start
    matrix = next(work.glob("*_REGISTRATION_MATRIX_*.txt"))
    return np.loadtxt(matrix), elapsed


def run_scene(name: str, kwargs: dict, root: Path, exe: str, cc: str) -> list[dict]:
    work = root / name
    scene = synthetic.write_scene(work / "scene", ORIGIN, **kwargs)
    k_true = scene["vertical_scale"]

    dem.dem_to_ply(work / "scene" / "dem.tif", work / "reference.ply", ORIGIN)
    write_dem_mesh(work / "scene" / "dem.tif", work / "reference_mesh.ply")
    moving = plyio.read_points(work / "scene" / "mesh.ply") - ORIGIN
    plyio.write_points(work / "moving.ply", moving)
    moving = plyio.read_points(work / "moving.ply")  # exactly what every tool reads (float32)

    truth = similarity_from_ypr(TRUTH["scale"], TRUTH["yaw_deg"], 0.0, 0.0, TRUTH["translation"])
    x_true = apply(truth, moving)
    x_true[:, 2] /= k_true  # undo the height exaggeration (applied about z = 0 of the local terrain)

    def row(method: str, m: np.ndarray, seconds: float, points: np.ndarray = moving,
            vertical: float | None = None) -> dict:
        err = np.sqrt(np.mean(np.sum((apply(m, points) - x_true) ** 2, axis=1)))
        return {"scene": name, "method": method, **describe(m), "vertical": vertical, "error": float(err),
                "seconds": seconds}

    rows = [row("truth", truth, 0.0, vertical=TRUTH["scale"] / k_true)]
    rows[0]["error"] = float("nan") if k_true != 1.0 else 0.0

    def ours(trim: float) -> tuple[np.ndarray, float]:
        out = work / f"ours_trim{trim:g}"
        start = time.perf_counter()
        subprocess.run([exe, "align", str(work / "reference.ply"), str(work / "moving.ply"), "--out", str(out),
                        "--scale", "--trim", str(trim)], check=True, capture_output=True)
        return ours_matrix(out / "results.json"), time.perf_counter() - start

    rows.append(row("terrain-icp (point-to-plane)", *ours(0.1)))

    if k_true != 1.0:
        start = time.perf_counter()
        cli.main(["run", str(work / "scene" / "dem.tif"), str(work / "scene" / "mesh.ply"), "--origin",
                  *map(str, ORIGIN), "--out", str(work / "ours_vertical"), "--shift-moving", "--scale",
                  "--vertical-scale", "--exe", exe])
        seconds = time.perf_counter() - start
        v = json.loads((work / "ours_vertical" / "vertical_scale.json").read_text())
        corrected = plyio.read_points(work / "ours_vertical" / "moving.ply")  # heights already divided by k
        rows.append(row("terrain-icp + vertical step", ours_matrix(work / "ours_vertical" / "results.json"),
                        seconds, corrected, v["vertical_scale"]))

    rows.append(row("CloudCompare point-to-point", *cloudcompare(cc, work, work / "reference.ply", [])))
    mesh = work / "reference_mesh.ply"
    rows.append(row("CloudCompare point-to-mesh (C2M)", *cloudcompare(cc, work, mesh, ["-USE_C2M_DIST"])))

    if scene["outliers"] > 0.0:
        # Sensitivity, both tools alike: trimming only works when it exceeds the outlier fraction (10.2 % drawn).
        rows.append(row("terrain-icp --trim 0.15", *ours(0.15)))
        rows.append(row("CloudCompare C2M -OVERLAP 85", *cloudcompare(cc, work, mesh, ["-USE_C2M_DIST"], 85)))
    return rows


def table(rows: list[dict]) -> str:
    lines = ["| Scene | Method | Scale | Vertical scale | Yaw [°] | Pitch [°] | Roll [°] | t E / N / U [m] "
             "| RMS error vs truth [m] | Time [s] |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        vertical = "" if r["vertical"] is None else f"{r['vertical']:.5f}"
        error = "—" if r["method"] == "truth" else f"{r['error']:.4f}"
        seconds = "" if r["method"] == "truth" else f"{r['seconds']:.1f}"
        t = " / ".join(f"{v:+.3f}" for v in r["t"])
        lines.append(f"| {r['scene']} | {r['method']} | {r['scale']:.5f} | {vertical} | {r['yaw']:+.4f} | "
                     f"{r['pitch']:+.4f} | {r['roll']:+.4f} | {t} | {error} | {seconds} |")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="terrain-icp executable (a Release build)")
    parser.add_argument("--cloudcompare", default=default_cloudcompare())
    parser.add_argument("--out", default="demo/compare")
    parser.add_argument("--scenes", nargs="*", default=list(SCENES), choices=list(SCENES))
    args = parser.parse_args()

    root = Path(args.out)
    exe = str(Path(args.exe).resolve())
    rows = []
    for name in args.scenes:
        print(f"scene {name}: {SCENES[name][0]}", flush=True)
        rows += run_scene(name, SCENES[name][1], root, exe, args.cloudcompare)
    text = "Scenes:\n\n" + "\n".join(f"- `{n}`: {SCENES[n][0]}" for n in args.scenes) + "\n\n" + table(rows) + "\n"
    (root / "comparison.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
