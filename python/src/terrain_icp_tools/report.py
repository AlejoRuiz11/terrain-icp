"""Report of an alignment: residual heat map, histogram and summary.md from results.json + residuals.ply."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only, no display
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from terrain_icp_tools import plyio  # noqa: E402


def vertical_scale(z: np.ndarray, distance: np.ndarray, normal_z: np.ndarray) -> dict:
    """Height stretch left in the residuals after the alignment (anisotropic, z only), as a separate step.

    If the moving heights are stretched by k about a level z0, a moving point sits dz = (z - z0)(1 - 1/k) above the
    reference, which the point-to-plane residual sees as n_z * dz. Least squares on
        distance = n_z * (a + b (z - mean(z)))
    gives the stretch b = 1 - 1/k (a absorbs any remaining vertical offset), so k = 1 / (1 - b)."""
    dz = z - z.mean()
    design = np.column_stack([normal_z, normal_z * dz])
    (a, b), *_ = np.linalg.lstsq(design, distance, rcond=None)
    return {"vertical_scale": float(1.0 / (1.0 - b)), "offset_m": float(a)}


def residual_stretch(results_dir: Path) -> float:
    """vertical_scale() on the inliers of an align output directory."""
    v = plyio.read_vertices(Path(results_dir) / "residuals.ply")
    inlier = v["inlier"] > 0.5
    return vertical_scale(v["z"][inlier], v["distance"][inlier], v["normal_z"][inlier])["vertical_scale"]


# The result is called stable under trimming when, across the swept ratios, the spread (max - min) stays below these:
# 3-14 times the spread of the noise-only synthetic scene over trims 0.1-0.3 (yaw 0.0035°, scale 1.4e-5, vertical
# 6.5e-5), and far below the effect of outliers left in the fit (yaw 0.2°, scale 0.04 with 20 % outliers, trim 0.1).
STABLE_SPREAD = {"scale": 2e-4, "vertical_scale": 5e-4, "yaw_deg": 0.01}


def sweep_row(results_dir: Path, trim: float) -> dict:
    """One row of the trimming sensitivity table from an align output directory (vertical scale if present)."""
    res = json.loads((Path(results_dir) / "results.json").read_text())
    vertical_file = Path(results_dir) / "vertical_scale.json"
    vertical = json.loads(vertical_file.read_text())["vertical_scale"] if vertical_file.is_file() else None
    return {"trim": trim, "scale": res["transform"]["scale"], "vertical_scale": vertical,
            "yaw_deg": res["transform"]["rotation_deg"]["yaw"], "rms_m": res["fit"]["rms_m"]}


def _spreads(rows: list[dict]) -> dict[str, float]:
    """max - min of each tested quantity over `rows` (vertical scale only when every row has it)."""
    return {k: max(r[k] for r in rows) - min(r[k] for r in rows)
            for k in STABLE_SPREAD if all(r[k] is not None for r in rows)}


def _unstable(rows: list[dict]) -> list[str]:
    return [k for k, v in _spreads(rows).items() if v > STABLE_SPREAD[k]]


def stable_from(rows: list[dict]) -> float | None:
    """Smallest trim of the longest run of consecutive highest ratios (at least two) that is stable; None if none."""
    rows = sorted(rows, key=lambda r: r["trim"])
    for i in range(len(rows) - 1):
        if not _unstable(rows[i:]):
            return rows[i]["trim"]
    return None


def sweep_section(rows: list[dict]) -> list[str]:
    rows = sorted(rows, key=lambda r: r["trim"])
    lines = ["## Trimming sensitivity", "",
             "The same alignment with each trim ratio (fraction of the worst pairs dropped every iteration).", "",
             "| Trim | Scale | Vertical scale | Yaw [°] | RMS [m] |", "|---|---|---|---|---|"]
    for r in rows:
        vertical = "—" if r["vertical_scale"] is None else f"{r['vertical_scale']:.5f}"
        lines.append(f"| {r['trim']:g} | {r['scale']:.5f} | {vertical} | {r['yaw_deg']:+.4f} | {r['rms_m']:.4f} |")
    if len(rows) < 2:
        return lines + ["", "A single trim ratio: no sensitivity to report.", ""]

    unstable = _unstable(rows)
    detail = ", ".join(f"{k} spread {v:.2g} (limit {STABLE_SPREAD[k]:g})" for k, v in _spreads(rows).items())
    verdict = (f"**NOT stable**: {', '.join(unstable)} change with the trim ratio; outliers probably exceed the "
               "smaller ratios, so trust the larger ones" if unstable else "**Stable**: the result does not depend on "
               "the trim ratio")
    start = stable_from(rows)
    since = (f"**Stable from trim {start:g}**: the ratios from {start:g} up agree within the limits."
             if start is not None else "**Not stable from any trim**: even the two highest ratios disagree.")
    lines += ["", f"{verdict} ({detail}). RMS naturally falls as more pairs are dropped; it is not part of the test.",
              "", since, ""]
    return lines


def _signed(value: float, digits: int) -> str:
    return f"{value:+.{digits}f}"


def _plot_map(xyz: np.ndarray, distance: np.ndarray, inlier: np.ndarray, path: Path) -> float:
    """Median residual per hexagonal cell (any number of points), symmetric colour scale clipped at the 98th
    percentile of |residual| of the inliers. Returns that limit."""
    limit = float(np.percentile(np.abs(distance[inlier]), 98)) or 1e-3
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    ax.scatter(xyz[~inlier, 0], xyz[~inlier, 1], s=2, c="0.6", label="rejected pairs")
    hb = ax.hexbin(xyz[inlier, 0], xyz[inlier, 1], C=distance[inlier], reduce_C_function=np.median, gridsize=120,
                   cmap="RdBu_r", vmin=-limit, vmax=limit)
    fig.colorbar(hb, ax=ax, label="signed distance to reference [m] (> 0: above)")
    ax.set_aspect("equal")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Residuals after alignment (median per cell)")
    if (~inlier).any():
        ax.legend(loc="upper right", markerscale=4)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return limit


def _plot_histogram(distance: np.ndarray, rms: float, path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4), constrained_layout=True)
    ax.hist(distance, bins=80, color="0.35")
    for v, style in ((distance.mean(), "-"), (-rms, ":"), (rms, ":")):
        ax.axvline(v, color="tab:red", linestyle=style)
    ax.set_xlabel("signed distance to reference [m] (inliers)")
    ax.set_ylabel("points")
    ax.set_title(f"Residual histogram: mean {distance.mean():+.3f} m, RMS {rms:.3f} m (dotted: ±RMS)")
    fig.savefig(path, dpi=120)
    plt.close(fig)


def make_report(results_dir: Path, out_dir: Path | None = None, vertical: bool = False,
                removed_stretch: float = 1.0, sweep: list[dict] | None = None) -> Path:
    """`removed_stretch`: height stretch already divided out of the moving cloud before the last alignment (`run
    --vertical-scale` alternates); it is part of the vertical scale reported."""
    results_dir = Path(results_dir)
    out_dir = Path(out_dir) if out_dir else results_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    res = json.loads((results_dir / "results.json").read_text())
    v = plyio.read_vertices(results_dir / "residuals.ply")
    xyz = np.column_stack([v["x"], v["y"], v["z"]])
    distance = v["distance"]
    inlier = v["inlier"] > 0.5

    _plot_map(xyz, distance, inlier, out_dir / "residual_map.png")
    _plot_histogram(distance[inlier], res["fit"]["rms_m"], out_dir / "residual_histogram.png")

    t, fit, cond = res["transform"], res["fit"], res["conditioning"]
    rot = t["rotation_deg"]
    scale = t["scale"]
    tr, shift = t["translation_m"], t["moving_centroid_shift_m"]
    estimated = res["options"]["estimate_scale"]
    cn = cond["condition_number"]

    # Scales follow results.json: factor that maps the moving cloud onto the reference (x_ref = s * x_moving).
    rows = [
        ("Scale moving → reference (uniform, ICP)", f"{scale:.5f} ({_signed(100 * (scale - 1), 3)} %)" if estimated
         else "not estimated (rigid alignment; use --scale)"),
    ]
    if vertical:
        k = removed_stretch * vertical_scale(xyz[inlier, 2], distance[inlier], v["normal_z"][inlier])["vertical_scale"]
        rows.append(("Vertical scale moving → reference (z-only fit)",
                     f"{scale / k:.5f} (ICP scale {scale:.5f} / height stretch {k:.5f})"))
    rows += [
        ("Heading change (rotation about Up)",
         f"{_signed(rot['yaw'], 4)}° counter-clockwise (bearings change by {_signed(-rot['yaw'], 4)}°)"),
        ("Pitch / roll", f"{_signed(rot['pitch'], 4)}° / {_signed(rot['roll'], 4)}°"),
        ("Translation E / N / U", " / ".join(f"{_signed(c, 3)} m" for c in tr)),
        ("Shift of the moving centroid E / N / U", " / ".join(f"{_signed(c, 3)} m" for c in shift)),
        ("RMS residual (inliers)", f"{fit['rms_m']:.4f} m"),
        ("Mean / median abs / 95 % abs", f"{fit['mean_m']:+.4f} / {fit['abs_p50_m']:.4f} / {fit['abs_p95_m']:.4f} m"),
        ("Inliers", f"{fit['inliers']} of {res['inputs']['moving_points']} ({100 * fit['inlier_ratio']:.1f} %)"),
        ("Iterations", f"{fit['iterations']} ({'converged' if fit['converged'] else 'NOT converged'})"),
        ("Condition number", "inf" if cn is None else f"{cn:.1f} (limit {res['options']['max_condition']:g})"),
    ]

    lines = [
        "# Terrain comparison",
        "",
        f"Reference `{res['inputs']['reference']}` ({res['inputs']['reference_points']} points), "
        f"moving `{res['inputs']['moving']}` ({res['inputs']['moving_points']} points).",
        "",
        "| Quantity | Value |",
        "|---|---|",
        *[f"| {k} | {val} |" for k, val in rows],
        "",
        "## Warnings",
        "",
        *([f"- {w}" for w in res["warnings"]] or ["None."]),
        "",
        *(sweep_section(sweep) if sweep else []),
        "## Residuals",
        "",
        "Signed point-to-plane distance of every aligned moving point to the reference (positive: above). "
        "Spatially coherent patches are deformation; salt-and-pepper is noise.",
        "",
        "![Residual map](residual_map.png)",
        "",
        "![Residual histogram](residual_histogram.png)",
        "",
        f"Convention: {res['convention']}.",
        "",
    ]
    summary = out_dir / "summary.md"
    summary.write_text("\n".join(lines), encoding="utf-8")
    return summary
