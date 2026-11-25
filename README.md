# terrain-icp

**Author:** Fabian Alejandro Ruiz Osorio ([@AlejoRuiz11](https://github.com/AlejoRuiz11)) · MIT License

Compare a 3D terrain reconstruction (for example the vertices of a photogrammetry mesh) against a georeferenced
elevation model (DEM) and measure how they differ: **scale, rotation, translation and the residual deformation**.

The core is a small C++20 point-to-plane ICP:

- reference normals by PCA of the k nearest neighbours (nanoflann k-d tree), oriented up;
- rigid (6 parameters) or similarity (7 parameters, uniform scale, `--scale`) estimation with Eigen;
- pair rejection by distance (`--max-dist`) and trimming of the worst residuals every iteration (`--trim`);
- an explicit **degeneracy check**: the eigenvalues of the normal matrix tell which parameters the terrain actually
  constrains. Flat terrain cannot fix yaw or horizontal position; instead of returning a number that only reflects
  noise, those directions are held at the initial guess and named in a warning.

Everything in this repository (tests, examples, figures) uses synthetic terrain. Real data stays outside the
repository and is passed by path.

> Status: MVP 1 (C++ core and CLI), MVP 2 (Python tools) and MVP 3 (cross-check with CloudCompare) done. See [Roadmap](#roadmap).

## Usage

```bash
terrain-icp align <reference.ply> <moving.ply> --out <dir> [--init init.json] [--scale] [--max-dist m]
                  [--trim ratio] [--max-iter n] [--max-cond c]
```

Inputs are PLY point clouds or meshes (ASCII or binary little-endian; `x y z` and optionally `nx ny nz` are read,
other vertex properties and faces are ignored) in a local metric frame: x East, y North, z Up, metres. Large
projected coordinates should be shifted to a local origin first (single-precision PLY loses millimetres at
10^6 m).

Outputs in `<dir>`:

- `results.json`: the transform (`scale`, `rotation_deg` as yaw/pitch/roll, `rotation_matrix`, `translation_m`,
  and the displacement of the moving cloud's centroid), fit statistics (RMS, mean, median and 95th percentile of
  |residual|, inliers, iterations, convergence), conditioning (eigenvalues, condition number, weak parameters) and
  warnings.
- `residuals.ply`: every moving point after alignment with `distance` (signed point-to-plane distance to the
  reference, positive above it) and `inlier` (0/1). It opens in CloudCompare or any PLY viewer as a scalar field.

The transform maps the moving cloud onto the reference: `x_ref = scale * R * x_moving + t`, with
`R = Rz(yaw) * Ry(pitch) * Rx(roll)` and yaw counter-clockwise seen from above. `--init` takes the same
`transform` object (any key may be omitted), so a previous `results.json` can be reused as the starting point.

## Python tools (DEM conversion, report, whole chain)

`python/` is a [uv](https://docs.astral.sh/uv/) project (rasterio, numpy, matplotlib, plyfile). From the repository
root:

```bash
uv run --project python terrain-icp-tools dem-to-ply dem.tif reference.ply --origin E N H [--step 2]
uv run --project python terrain-icp-tools report out/ [--vertical-scale]
uv run --project python terrain-icp-tools run dem.tif mesh.ply --origin E N H --out out/ \
    [--shift-moving] [--scale] [--max-dist m] [--max-iter n] [--max-cond c] [--vertical-scale] [--trim-sweep 0.1,0.2,0.3] \
    [--exe path/to/terrain-icp]
```

- `dem-to-ply` turns band 1 of a GeoTIFF into pixel-centre points `(E − E0, N − N0, h − H0)`: a local metric frame
  around a chosen origin (for example the landing zone), so the PLY keeps full precision. The DEM must be in a
  projected CRS in metres; nodata is dropped; `--step n` keeps every n-th row and column.
- `report` reads `results.json` and `residuals.ply` and writes `residual_map.png` (median residual per hexagonal
  cell, diverging colour scale), `residual_histogram.png` and `summary.md` (scale, heading change, pitch/roll,
  translation, RMS and percentiles, inliers, conditioning, warnings).
- `run` chains `dem-to-ply`, `terrain-icp align` and `report`. With `--shift-moving` the mesh is taken to be in the
  DEM's CRS and is shifted by the same origin.
- `run --trim-sweep 0.1,0.2,0.3` repeats the alignment (and the vertical step, if asked) with each trim ratio in
  `out/trim_sweep/` and adds to `summary.md` a table of scale, vertical scale, yaw and RMS per ratio, with a verdict:
  *stable* when the spread stays under 2e-4 in scale, 5e-4 in vertical scale and 0.01° in yaw, and the ratio it is
  stable from (the longest run of highest ratios that agree, e.g. "stable from trim 0.2"). An unstable result
  usually means the outliers exceed the smaller ratios (trimming must exceed the outlier fraction).

**Vertical scale** (`--vertical-scale`): a mesh can be stretched in height only (for example a camera calibration
error), which a uniform scale cannot represent. After the alignment, a moving point whose heights are stretched by
`k` about a level `z0` sits `dz = (z − z0)(1 − 1/k)` above the reference, seen by the point-to-plane residual as
`n_z · dz`; a least-squares fit of `distance = n_z (a + b (z − z̄))` gives `k = 1 / (1 − b)`. Because the uniform
ICP absorbs part of such a stretch (in its scale and tilt), `run` alternates: it divides the stretch found so far
out of the moving heights and re-aligns from the previous result, until `|k − 1| < 1e-4` (at most 10 rounds). The
report gives the vertical scale moving → reference as `scale / k`, also written to `vertical_scale.json`. On the synthetic test (heights exaggerated
3 %) one fit alone is off by 0.6 %; the alternation recovers it to 0.05 %.

`terrain-icp-tools synth --out demo/` writes a synthetic scene (`dem.tif` on a generic transverse Mercator,
`mesh.ply` in the same CRS and `truth.json`), so the whole chain can be tried without real data:

```bash
uv run --project python terrain-icp-tools synth --out demo
uv run --project python terrain-icp-tools run demo/dem.tif demo/mesh.ply --origin 1000 2000 100 --out demo/out \
    --shift-moving --scale --vertical-scale --exe build/msvc-release/terrain-icp.exe
```

## Local UI

An optional [Streamlit](https://streamlit.io) interface runs the same chain as `run` from the browser:

```bash
uv sync --project python --extra ui
uv run --project python terrain-icp-tools ui        # --port to change the port (default 8501)
```

It listens on localhost only, with telemetry off; uploads and the default output folder go to a temporary
directory. Step by step, with the synthetic scene:

1. Build the Release executable (`cmake --preset msvc-release` and `cmake --build --preset msvc-release`, or the
   `gcc-release` preset). The UI finds it in `build/*-release/`; otherwise give its path in *terrain-icp executable*.
2. Write a scene: `uv run --project python terrain-icp-tools synth --out demo/scene`. It prints the origin's
   latitude and longitude (`0.01808739`, `0.00898315` for the default origin E 1000, N 2000, H 100).
3. Start the UI, drop `demo/scene/dem.tif` on *Reference DEM* and `demo/scene/mesh.ply` on *Moving mesh*. Leave
   *The mesh is in the DEM's CRS* ticked: the synthetic mesh is georeferenced.
4. Enter the origin as latitude / longitude (converted to the DEM's CRS with pyproj; a DEM that is not projected in
   metres is rejected with a clear error) or switch to easting / northing. Elevation 100.
5. Optionally load an initial alignment: a previous `results.json`, or `{"east_m", "north_m", "up_m", "yaw_deg"}`.
   Options default to scale and vertical scale on and a trim sweep of 0.1, 0.2, 0.3; *Advanced* has the iteration
   and condition-number limits and the DEM step.
6. Press **Align**. The progress bar follows the steps (DEM, alignment, each trim, report).

![UI inputs with the synthetic scene](docs/img/ui_inputs.png)

The result shows the known transform of the scene (scale 1.005, yaw 2°, translation 2 / −2 / 1 m), the vertical
scale, RMS, warnings, the trim sweep with its verdict, the residual map and histogram, and a button that opens the
output folder (`results.json`, `residuals.ply`, `summary.md`, figures).

![UI result with the synthetic scene](docs/img/ui_results.png)

## Method

For each moving point `p` (already transformed) the nearest reference point `q` with normal `n` gives the residual
`r = n · (p − q)`. Linearising a small update about the centroid `c` of the inliers (rotation vector `ω`,
translation `δt`, relative scale `δs`):

```
r(ω, δt, δs) ≈ r + [(p − c) × n] · ω + n · δt + [n · (p − c)] δs
```

The rows are stacked into `J`, and `JᵀJ x = −Jᵀr` is solved; the increment is composed as
`p → c + (1 + δs) R(ω) (p − c) + δt`. The rotation and scale columns are divided by the RMS radius `L` of the cloud,
so every unknown is expressed as the displacement (m) it causes on a typical point. That makes the eigenvalues of
`JᵀJ / N` dimensionless and comparable, and their ratio `λmax / λmin` a meaningful condition number.

The system is solved through its eigen-decomposition. Directions with `λ < λmax / max_cond` (default 1000) are not
updated, and every parameter that lies mostly in that weak subspace is reported (`weak_parameters`). On flat ground
the normals are all vertical, so the yaw, horizontal translation and scale columns vanish; the result is a
`"flat terrain: ..."` warning and those parameters keep their initial value.

Robustness: pairs farther than `--max-dist` are rejected; of the rest, the fraction `--trim` (default 10 %) with the
largest |r| is dropped every iteration (trimmed ICP). Iteration stops when an update changes the rotation by less
than 1e-7 rad, the translation by less than 10 µm and the scale by less than 1e-7.

## Build and test

Requirements: CMake ≥ 3.25, Ninja, and MSVC (Visual Studio 2022 or later) or GCC ≥ 13. Eigen 3.4.1, nanoflann
1.7.1, nlohmann/json 3.12.0 and GoogleTest 1.17.0 are downloaded at configure time (pinned archives, SHA256
checked).

```bash
cmake --preset msvc            # or gcc; *-release presets for real data (Debug Eigen is slow)
cmake --build --preset msvc
ctest --preset msvc
```

On Windows, Ninja needs the MSVC environment; from Git Bash, `scripts/msvc.cmd` loads it:

```bash
cmd //c "scripts\msvc.cmd cmake --preset msvc"
cmd //c "scripts\msvc.cmd cmake --build --preset msvc"
cmd //c "scripts\msvc.cmd ctest --preset msvc"
```

The tests build a terrain from a sum of Gaussian hills (200 m × 200 m DEM at 1 m spacing, 6000 random "mesh"
vertices over the central 140 m × 140 m with 2 cm of noise) and check that:

- a known similarity (yaw 2°, translation 3 m, scale 1.005) is recovered to 0.01°, 2 cm and 1e-4;
- the same holds with 10 % of the vertices lifted 3–20 m (outliers), and none of them is kept as inlier;
- on flat terrain yaw, horizontal translation and scale are reported as weak and held, while the vertical fit
  still reaches the noise level;
- the PLY reader and writer round-trip ASCII and binary files, and skip faces and extra properties.

The Python tests (`uv run --project python pytest python/tests`) check the DEM conversion against the analytic
terrain, the report, and the whole `run` chain on a synthetic GeoTIFF and a georeferenced mesh, including a 3 %
height exaggeration recovered by the vertical-scale step. The end-to-end tests use `build/<preset>/terrain-icp` or
`$TERRAIN_ICP_EXE`, and are skipped when neither exists.

## Cross-check with CloudCompare

The same synthetic inputs (local PLY files) were registered with CloudCompare 2.13.2's ICP from the command line
(`-ICP -ADJUST_SCALE`, run to convergence with `-MIN_ERROR_DIFF 1e-8`, `-OVERLAP 90` = the same 10 % trimming as
`terrain-icp`'s default), against the DEM points (point-to-point) and against the DEM triangulated into a mesh
(`-USE_C2M_DIST`, point-to-surface). The error column is tool-independent: RMS over all moving points of the
distance between where the estimated transform puts each point and where it really belongs. Reproduce with:

```bash
uv run --project python python scripts/compare_cloudcompare.py --exe build/msvc-release/terrain-icp.exe \
    [--cloudcompare path/to/CloudCompare.exe] --out demo/compare
```

Scenes (all with yaw 2°, translation (2, −2, 1) m, scale 1.005, 2 cm noise, 6000 moving points):
`base` (DEM at 1 m); `outliers` (10.2 % of the vertices lifted 3–20 m); `vertical` (mesh heights exaggerated 3 %);
`dense_dem` (as `base`, DEM at 0.25 m).

| Scene | Method | Scale | Vertical scale | Yaw [°] | Pitch [°] | Roll [°] | t E / N / U [m] | RMS error vs truth [m] | Time [s] |
|---|---|---|---|---|---|---|---|---|---|
| base | truth | 1.00500 | 1.00500 | +2.0000 | -0.0000 | +0.0000 | +2.000 / -2.000 / +1.000 | — |  |
| base | terrain-icp (point-to-plane) | 1.00506 |  | +2.0001 | +0.0011 | +0.0002 | +2.003 / -1.999 / +0.999 | 0.0049 | 0.1 |
| base | CloudCompare point-to-point | 1.00859 |  | +1.7505 | -0.0615 | -0.0453 | +1.846 / -1.805 / +0.933 | 0.4304 | 1.9 |
| base | CloudCompare point-to-mesh (C2M) | 1.00495 |  | +2.0001 | +0.0001 | +0.0003 | +2.000 / -1.999 / +1.004 | 0.0048 | 11.7 |
| outliers | truth | 1.00500 | 1.00500 | +2.0000 | -0.0000 | +0.0000 | +2.000 / -2.000 / +1.000 | — |  |
| outliers | terrain-icp (point-to-plane) | 1.00509 |  | +1.9877 | -0.0030 | -0.0010 | +1.983 / -1.997 / +0.992 | 0.0231 | 0.1 |
| outliers | CloudCompare point-to-point | 1.00888 |  | +1.7449 | -0.0511 | -0.0467 | +1.881 / -1.795 / +0.920 | 0.4363 | 2.0 |
| outliers | CloudCompare point-to-mesh (C2M) | 1.00504 |  | +1.9884 | -0.0036 | -0.0011 | +1.983 / -1.996 / +0.993 | 0.0230 | 13.2 |
| outliers | terrain-icp --trim 0.15 | 1.00504 |  | +1.9991 | +0.0010 | +0.0006 | +2.004 / -1.999 / +0.999 | 0.0047 | 0.1 |
| outliers | CloudCompare C2M -OVERLAP 85 | 1.00494 |  | +2.0001 | +0.0002 | +0.0004 | +2.002 / -1.999 / +1.002 | 0.0045 | 17.9 |
| vertical | truth | 1.00500 | 0.97573 | +2.0000 | -0.0000 | +0.0000 | +2.000 / -2.000 / +1.000 | — |  |
| vertical | terrain-icp (point-to-plane) | 1.00154 |  | +1.9976 | -0.1151 | +0.0017 | +1.907 / -1.912 / +0.808 | 0.2801 | 0.1 |
| vertical | terrain-icp + vertical step | 1.00502 | 0.97596 | +2.0001 | +0.0001 | +0.0005 | +2.002 / -1.998 / +0.715 | 0.0034 | 1.0 |
| vertical | CloudCompare point-to-point | 1.00696 |  | +1.8605 | -0.1459 | +0.0044 | +1.867 / -1.819 / +0.683 | 0.3542 | 2.7 |
| vertical | CloudCompare point-to-mesh (C2M) | 1.00143 |  | +1.9888 | -0.1203 | -0.0124 | +1.871 / -1.902 / +0.831 | 0.3091 | 12.4 |
| dense_dem | truth | 1.00500 | 1.00500 | +2.0000 | -0.0000 | +0.0000 | +2.000 / -2.000 / +1.000 | — |  |
| dense_dem | terrain-icp (point-to-plane) | 1.00504 |  | +2.0004 | +0.0007 | +0.0005 | +2.003 / -1.999 / +0.999 | 0.0038 | 0.8 |
| dense_dem | CloudCompare point-to-point | 1.00586 |  | +1.9454 | -0.0199 | -0.0114 | +1.946 / -1.941 / +0.985 | 0.1153 | 8.4 |
| dense_dem | CloudCompare point-to-mesh (C2M) | 1.00499 |  | +2.0002 | +0.0007 | +0.0002 | +2.002 / -2.000 / +1.004 | 0.0040 | 187.2 |

What the differences mean (no parameter was tuned to make the tools agree):

- **Point-to-point vs point-to-plane.** CloudCompare's default ICP pairs each moving point with the nearest DEM
  *node*. With 1 m nodes that distance is ~0.4 m whatever the alignment, so the cost is nearly flat below the node
  spacing and the optimum drifts: 0.25° of yaw and 0.36 % of scale, ~0.4 m at the edge of the area. With a DEM 4×
  denser the error drops about 4× (0.43 → 0.12 m), which confirms it is a sampling effect, not a convergence one
  (1000 iterations give the same result). Against the triangulated DEM (C2M, point-to-surface) CloudCompare agrees
  with `terrain-icp` to a few millimetres: both measure the distance to the surface, not to its samples.
- **Trimming must exceed the outlier fraction.** The random draw lifted 10.2 % of the vertices; trimming 10 % (or
  overlap 90 %) leaves ~11 of them in, which costs 0.012° of yaw and 2 cm of error in *both* tools alike. With 15 %
  / overlap 85 % both return to the 5 mm of the clean scene.
- **Height exaggeration is not a similarity.** A uniform scale cannot absorb a 3 % height stretch; every
  uniform-scale method compensates with tilt (pitch −0.12°) and a wrong scale, leaving ~0.3 m of error.
  `run --vertical-scale` estimates the stretch apart and recovers the vertical scale (0.97596 vs 0.97573) and the
  horizontal transform. Its vertical translation differs because the heights are corrected about their mean, not
  about the terrain's zero; the error column accounts for that.
- **Time** includes process start-up and file loading; it is one run on one machine, not a benchmark.
  CloudCompare's C2M variant computes point-to-triangle distances, hence its cost on the 640 k-vertex dense DEM.

## Roadmap

1. **C++ core** — ICP, conditioning, CLI, results.json and residuals.ply. *Done.*
2. **Python tools** — DEM GeoTIFF to PLY around a chosen origin, residual heat map and summary report, vertical
   scale, one `run` command for the whole chain. *Done.*
3. **Cross-check with CloudCompare** — same synthetic inputs, table of differences. *Done*, see
   [above](#cross-check-with-cloudcompare).
4. **Local UI** — Streamlit front end over `run`, origin by latitude / longitude. *Done*, see
   [Local UI](#local-ui).

## Author and license

Designed and written by **Fabian Alejandro Ruiz Osorio** ([@AlejoRuiz11](https://github.com/AlejoRuiz11)): the method, the code, the tests
and the documentation. Copyright (c) 2026 Fabian Alejandro Ruiz Osorio.

Released under the MIT License, see [LICENSE](LICENSE) and [NOTICE](NOTICE). You may use, modify and redistribute
it, including inside other projects, **as long as the copyright notice with the author's name is kept**. If you use it
in published work, please cite it ([CITATION.cff](CITATION.cff)).
