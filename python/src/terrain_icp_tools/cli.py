# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
"""terrain-icp-tools: dem-to-ply, report, run (the whole chain), synth (a synthetic scene for demos) and ui."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

import terrain_icp_tools
from terrain_icp_tools import dem, plyio, report, synthetic

# Release builds of this repository (python/src/terrain_icp_tools/cli.py -> repository root), used as a last resort.
REPO = Path(__file__).resolve().parents[3]
REPO_BUILDS = [REPO / "build" / preset / name for preset in ("msvc-release", "gcc-release")
               for name in ("terrain-icp.exe", "terrain-icp")]

Progress = Callable[[int, int, str], None]  # (step done, total steps, label of the next step)


def find_executable(explicit: str | None) -> str:
    """--exe, then $TERRAIN_ICP_EXE, then `terrain-icp` on PATH, then a Release build of this repository."""
    candidate = explicit or os.environ.get("TERRAIN_ICP_EXE") or shutil.which("terrain-icp")
    if not candidate:
        candidate = next((str(p) for p in REPO_BUILDS if p.is_file()), None)
    if not candidate or not Path(candidate).is_file():
        raise SystemExit("terrain-icp executable not found: pass --exe, set TERRAIN_ICP_EXE, add it to PATH or "
                         "build the Release preset")
    return str(Path(candidate).resolve())  # Windows cannot start a relative path written with '/'


def run(args: argparse.Namespace, progress: Progress | None = None) -> Path:
    """The whole chain; returns the path of summary.md. `progress` is told before each step."""
    sweep_trims = args.trim_sweep or []
    total = 3 + len(sweep_trims)

    def step(done: int, label: str) -> None:
        if progress:
            progress(done, total, label)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    origin = tuple(args.origin)

    step(0, "Converting the DEM")
    reference = out / "reference.ply"
    count = dem.dem_to_ply(Path(args.dem), reference, origin, args.step)
    print(f"DEM -> {reference.name}: {count} points")

    points = plyio.read_points(Path(args.mesh))
    if args.shift_moving:  # the mesh is in the DEM's CRS: bring it to the same local origin
        points = points - origin
    exe = find_executable(args.exe)

    step(1, "Aligning")
    stretch = align_and_fit(exe, args, reference, points, out, args.trim)
    sweep = []
    for i, trim in enumerate(sweep_trims):
        step(2 + i, f"Trim sweep: {trim:g}")
        sub = out / "trim_sweep" / f"{trim:g}"
        print(f"trim sweep: {trim:g}")
        sub.mkdir(parents=True, exist_ok=True)
        align_and_fit(exe, args, reference, points, sub, trim)
        sweep.append(report.sweep_row(sub, trim))

    step(total - 1, "Writing the report")
    summary = report.make_report(out, vertical=args.vertical_scale, removed_stretch=stretch, sweep=sweep)
    print(f"report: {summary}")
    step(total, "Done")
    return summary


def align_and_fit(exe: str, args: argparse.Namespace, reference: Path, points, out: Path,
                  trim: float | None) -> float:
    """terrain-icp align into `out` (moving.ply, results.json, residuals.ply), plus the vertical step when asked.
    Returns the height stretch divided out of moving.ply (1 without --vertical-scale)."""
    moving = out / "moving.ply"
    plyio.write_points(moving, points)

    def align(init: str | None) -> None:
        command = [exe, "align", str(reference), str(moving), "--out", str(out)]
        if args.scale:
            command.append("--scale")
        for flag, value in (("--max-dist", args.max_dist), ("--trim", trim), ("--init", init),
                            ("--max-iter", args.max_iter), ("--max-cond", args.max_cond)):
            if value is not None:
                command += [flag, str(value)]
        subprocess.run(command, check=True)

    align(args.init)
    stretch = 1.0
    if args.vertical_scale:
        # Alternate: a uniform-scale ICP absorbs part of a height stretch (scale, tilt), so one fit on its residuals
        # is biased. Remove the stretch found so far from the moving heights (about their mean, in the moving frame;
        # fine while pitch and roll are small) and re-align from the previous result until nothing is left.
        z_mean = points[:, 2].mean()
        for _ in range(args.vertical_iterations):
            k = report.residual_stretch(out)
            stretch *= k
            corrected = points.copy()
            corrected[:, 2] = z_mean + (points[:, 2] - z_mean) / stretch
            plyio.write_points(moving, corrected)
            align(str(out / "results.json"))
            if abs(k - 1.0) < 1e-4:
                break
        # Machine-readable result of the vertical step; moving.ply holds the height-corrected cloud it applies to.
        stretch_total = stretch * report.residual_stretch(out)
        scale = json.loads((out / "results.json").read_text())["transform"]["scale"]
        (out / "vertical_scale.json").write_text(json.dumps(
            {"height_stretch": stretch_total, "vertical_scale": scale / stretch_total,
             "heights_corrected_about_z": float(z_mean)}, indent=2) + "\n")
    return stretch


def trim_list(text: str) -> list[float]:
    values = [float(v) for v in text.split(",") if v.strip()]
    if not values or any(not 0.0 <= v < 1.0 for v in values):
        raise argparse.ArgumentTypeError("--trim-sweep needs comma-separated ratios in [0, 1)")
    return values


def run_ui(port: int) -> int:
    """Launch the Streamlit app bound to localhost, without telemetry, with large uploads allowed."""
    if importlib.util.find_spec("streamlit") is None:
        print("error: the UI needs the optional 'ui' extra; install it with: uv sync --project python --extra ui",
              file=sys.stderr)
        return 1
    command = [sys.executable, "-m", "streamlit", "run", str(Path(__file__).with_name("ui.py")),
               "--server.address", "localhost", "--server.port", str(port),
               "--browser.gatherUsageStats", "false", "--server.maxUploadSize", "4096",
               "--server.showEmailPrompt", "false", "--client.toolbarMode", "minimal"]
    return subprocess.run(command, check=False).returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="terrain-icp-tools", description=__doc__)
    parser.add_argument("--version", action="version",
                        version=f"%(prog)s {terrain_icp_tools.__version__} - {terrain_icp_tools.__copyright__}")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_origin(p: argparse.ArgumentParser) -> None:
        p.add_argument("--origin", nargs=3, type=float, required=True, metavar=("E", "N", "H"),
                       help="local origin in the DEM's CRS (metres), e.g. the landing zone")

    p = sub.add_parser("dem-to-ply", help="GeoTIFF DEM -> PLY in local metres around --origin")
    p.add_argument("dem")
    p.add_argument("out")
    add_origin(p)
    p.add_argument("--step", type=int, default=1, help="keep every n-th row and column (default 1)")

    p = sub.add_parser("report", help="heat map, histogram and summary.md from an align output directory")
    p.add_argument("results_dir")
    p.add_argument("--out", help="output directory (default: results_dir)")
    p.add_argument("--vertical-scale", action="store_true", help="also fit a z-only scale on the residuals")

    p = sub.add_parser("run", help="dem-to-ply + terrain-icp align + report")
    p.add_argument("dem")
    p.add_argument("mesh", help="moving PLY (mesh or points), in local metres unless --shift-moving")
    add_origin(p)
    p.add_argument("--out", required=True)
    p.add_argument("--step", type=int, default=1)
    p.add_argument("--shift-moving", action="store_true", help="the mesh is in the DEM's CRS: subtract --origin")
    p.add_argument("--scale", action="store_true")
    p.add_argument("--max-dist", type=float)
    p.add_argument("--trim", type=float)
    p.add_argument("--max-iter", type=int)
    p.add_argument("--max-cond", type=float)
    p.add_argument("--init")
    p.add_argument("--vertical-scale", action="store_true",
                   help="estimate a z-only scale, alternating with the ICP (see --vertical-iterations)")
    p.add_argument("--vertical-iterations", type=int, default=10)
    p.add_argument("--trim-sweep", type=trim_list, metavar="R1,R2,...",
                   help="also align with each trim ratio (in out/trim_sweep/) and tabulate the sensitivity")
    p.add_argument("--exe", help="path of terrain-icp (default: $TERRAIN_ICP_EXE or PATH)")

    p = sub.add_parser("synth", help="synthetic scene: dem.tif, mesh.ply (same CRS) and truth.json")
    p.add_argument("--out", required=True)
    p.add_argument("--origin", nargs=3, type=float, default=(1000.0, 2000.0, 100.0), metavar=("E", "N", "H"))
    p.add_argument("--scale", type=float, default=1.005)
    p.add_argument("--yaw", type=float, default=2.0, help="degrees")
    p.add_argument("--translation", nargs=3, type=float, default=(2.0, -2.0, 1.0))
    p.add_argument("--vertical-scale", type=float, default=1.0)
    p.add_argument("--outliers", type=float, default=0.0, help="fraction of mesh vertices lifted 3-20 m")

    p = sub.add_parser("ui", help="local web interface (needs the 'ui' extra)")
    p.add_argument("--port", type=int, default=8501)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.command == "dem-to-ply":
        n = dem.dem_to_ply(Path(args.dem), Path(args.out), tuple(args.origin), args.step)
        print(f"{n} points -> {args.out}")
    elif args.command == "report":
        print(report.make_report(Path(args.results_dir), args.out and Path(args.out), args.vertical_scale))
    elif args.command == "run":
        run(args)
    elif args.command == "ui":
        sys.exit(run_ui(args.port))
    elif args.command == "synth":
        synthetic.write_scene(Path(args.out), tuple(args.origin), args.scale, args.yaw, tuple(args.translation),
                              args.vertical_scale, outliers=args.outliers)
        lat, lon = synthetic.origin_latlon(tuple(args.origin))
        print(f"dem.tif, mesh.ply and truth.json -> {args.out}")
        print(f"origin E/N/H {' '.join(f'{v:g}' for v in args.origin)} = latitude {lat:.8f}, longitude {lon:.8f}")


if __name__ == "__main__":
    main(sys.argv[1:])
