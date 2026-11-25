# terrain-icp

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

> Status: MVP 1 (C++ core and CLI) done. See [Roadmap](#roadmap).

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

## Roadmap

1. **C++ core** — ICP, conditioning, CLI, results.json and residuals.ply. *Done.*
2. **Python tools** — DEM GeoTIFF to PLY around a chosen origin, residual heat map and summary report, one `run`
   command for the whole chain.
3. **Cross-check with CloudCompare** — same synthetic inputs through its fine registration with scale; table of
   differences.

## License

MIT, see [LICENSE](LICENSE).
