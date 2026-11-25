"""terrain-icp-tools: dem-to-ply, report, run (the whole chain) and synth (a synthetic scene for demos)."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

from terrain_icp_tools import dem, plyio, report, synthetic


def find_executable(explicit: str | None) -> str:
    """--exe, then $TERRAIN_ICP_EXE, then `terrain-icp` on PATH."""
    candidate = explicit or os.environ.get("TERRAIN_ICP_EXE") or shutil.which("terrain-icp")
    if not candidate or not Path(candidate).is_file():
        raise SystemExit("terrain-icp executable not found: pass --exe, set TERRAIN_ICP_EXE or add it to PATH")
    return str(Path(candidate).resolve())  # Windows cannot start a relative path written with '/'


def run(args: argparse.Namespace) -> None:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    origin = tuple(args.origin)

    reference = out / "reference.ply"
    count = dem.dem_to_ply(Path(args.dem), reference, origin, args.step)
    print(f"DEM -> {reference.name}: {count} points")

    points = plyio.read_points(Path(args.mesh))
    if args.shift_moving:  # the mesh is in the DEM's CRS: bring it to the same local origin
        points = points - origin
    moving = out / "moving.ply"
    plyio.write_points(moving, points)

    exe = find_executable(args.exe)

    def align(init: str | None) -> None:
        command = [exe, "align", str(reference), str(moving), "--out", str(out)]
        if args.scale:
            command.append("--scale")
        for flag, value in (("--max-dist", args.max_dist), ("--trim", args.trim), ("--init", init)):
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

    summary = report.make_report(out, vertical=args.vertical_scale, removed_stretch=stretch)
    print(f"report: {summary}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="terrain-icp-tools", description=__doc__)
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
    p.add_argument("--init")
    p.add_argument("--vertical-scale", action="store_true",
                   help="estimate a z-only scale, alternating with the ICP (see --vertical-iterations)")
    p.add_argument("--vertical-iterations", type=int, default=10)
    p.add_argument("--exe", help="path of terrain-icp (default: $TERRAIN_ICP_EXE or PATH)")

    p = sub.add_parser("synth", help="synthetic scene: dem.tif, mesh.ply (same CRS) and truth.json")
    p.add_argument("--out", required=True)
    p.add_argument("--origin", nargs=3, type=float, default=(1000.0, 2000.0, 100.0), metavar=("E", "N", "H"))
    p.add_argument("--scale", type=float, default=1.005)
    p.add_argument("--yaw", type=float, default=2.0, help="degrees")
    p.add_argument("--translation", nargs=3, type=float, default=(2.0, -2.0, 1.0))
    p.add_argument("--vertical-scale", type=float, default=1.0)

    args = parser.parse_args(argv)
    if args.command == "dem-to-ply":
        n = dem.dem_to_ply(Path(args.dem), Path(args.out), tuple(args.origin), args.step)
        print(f"{n} points -> {args.out}")
    elif args.command == "report":
        print(report.make_report(Path(args.results_dir), args.out and Path(args.out), args.vertical_scale))
    elif args.command == "run":
        run(args)
    elif args.command == "synth":
        synthetic.write_scene(Path(args.out), tuple(args.origin), args.scale, args.yaw, tuple(args.translation),
                              args.vertical_scale)
        print(f"dem.tif, mesh.ply and truth.json -> {args.out}")


if __name__ == "__main__":
    main(sys.argv[1:])
