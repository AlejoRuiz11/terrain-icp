"""Everything the Streamlit app does except drawing widgets, so it can be tested without a browser.

The app (ui.py) only collects inputs, calls `run_alignment` and shows the `Outcome`."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pyproj
import rasterio

from terrain_icp_tools import cli, report

WGS84_LONLAT = pyproj.CRS.from_proj4("+proj=longlat +datum=WGS84 +no_defs")
DEFAULT_SWEEP = (0.1, 0.2, 0.3)


def dem_crs(dem: Path) -> pyproj.CRS:
    """CRS of the DEM; it must be projected with metre axes (E, N and h in one unit)."""
    with rasterio.open(dem) as src:
        if src.crs is None:
            raise ValueError(f"{Path(dem).name} has no CRS")
        crs = pyproj.CRS.from_wkt(src.crs.to_wkt())
    if not crs.is_projected or any(abs(a.unit_conversion_factor - 1.0) > 1e-12 for a in crs.axis_info[:2]):
        raise ValueError(f"{Path(dem).name}: the DEM's CRS is not projected in metres ({crs.name}); reproject it to a "
                         "metric projection first")
    return crs


def origin_from_latlon(dem: Path, lat: float, lon: float, height: float) -> tuple[float, float, float]:
    """Latitude / longitude (decimal degrees, WGS84) and height -> (E, N, H) in the DEM's CRS. The height is used as
    given: it must be in the DEM's vertical reference."""
    to_dem = pyproj.Transformer.from_crs(WGS84_LONLAT, dem_crs(dem), always_xy=True)
    e, n = to_dem.transform(lon, lat)
    return float(e), float(n), float(height)


def write_init(src: Path, dst: Path) -> Path:
    """Initial alignment for `terrain-icp align --init`, from either a terrain-icp results.json (or its `transform`
    object), used as is, or a simple JSON {"east_m", "north_m", "up_m", "yaw_deg"} (any key may be omitted)."""
    data = json.loads(Path(src).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("initial alignment: a JSON object is expected")
    if {"transform", "translation_m", "rotation_deg", "scale"} & data.keys():
        transform = data.get("transform", data)
    else:
        simple = {"east_m", "north_m", "up_m", "yaw_deg"}
        if not data.keys() <= simple:
            raise ValueError(f"initial alignment: unknown keys {sorted(data.keys() - simple)}; expected a "
                             "results.json or {east_m, north_m, up_m, yaw_deg}")
        transform = {"translation_m": [float(data.get(k, 0.0)) for k in ("east_m", "north_m", "up_m")],
                     "rotation_deg": {"yaw": float(data.get("yaw_deg", 0.0))}}
    dst.write_text(json.dumps({"transform": transform}, indent=2) + "\n", encoding="utf-8")
    return dst


def default_executable() -> str | None:
    """terrain-icp as the CLI would find it ($TERRAIN_ICP_EXE, PATH, the repository's Release build), or None."""
    try:
        return cli.find_executable(None)
    except SystemExit:
        return None


@dataclass(frozen=True)
class Options:
    scale: bool = True
    vertical_scale: bool = True
    trim_sweep: tuple[float, ...] = DEFAULT_SWEEP
    shift_moving: bool = True  # the mesh is in the DEM's CRS
    max_dist: float | None = None
    trim: float | None = None
    max_iter: int | None = None
    max_cond: float | None = None
    step: int = 1


@dataclass
class Outcome:
    out: Path
    origin: tuple[float, float, float]
    results: dict
    vertical_scale: float | None
    sweep: list[dict] = field(default_factory=list)
    verdict: tuple[str, str] | None = None  # report.sweep_verdict, with two trims or more

    @property
    def residual_map(self) -> Path:
        return self.out / "residual_map.png"

    @property
    def histogram(self) -> Path:
        return self.out / "residual_histogram.png"

    @property
    def summary(self) -> Path:
        return self.out / "summary.md"


def run_arguments(dem: Path, mesh: Path, origin: tuple[float, float, float], out: Path, options: Options,
                  exe: str | None, init: Path | None) -> list[str]:
    """The `terrain-icp-tools run` command line equivalent to the UI inputs."""
    argv = ["run", str(dem), str(mesh), "--origin", *(repr(v) for v in origin), "--out", str(out),
            "--step", str(options.step)]
    argv += [flag for flag, on in (("--shift-moving", options.shift_moving), ("--scale", options.scale),
                                   ("--vertical-scale", options.vertical_scale)) if on]
    for flag, value in (("--max-dist", options.max_dist), ("--trim", options.trim), ("--max-iter", options.max_iter),
                        ("--max-cond", options.max_cond), ("--init", init), ("--exe", exe)):
        if value is not None:
            argv += [flag, str(value)]
    if options.trim_sweep:
        argv += ["--trim-sweep", ",".join(f"{t:g}" for t in options.trim_sweep)]
    return argv


def run_alignment(dem: Path, mesh: Path, origin: tuple[float, float, float], out: Path, options: Options = Options(),
                  exe: str | None = None, init: Path | None = None, progress: cli.Progress | None = None) -> Outcome:
    """Same chain as `terrain-icp-tools run` (it is called with the equivalent arguments), then the outcome."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if init is not None:
        init = write_init(init, out / "init.json")
    args = cli.build_parser().parse_args(run_arguments(dem, mesh, origin, out, options, exe, init))
    cli.run(args, progress)
    return load_outcome(out, origin, options.trim_sweep)


def load_outcome(out: Path, origin: tuple[float, float, float], trims: tuple[float, ...] = ()) -> Outcome:
    out = Path(out)
    results = json.loads((out / "results.json").read_text())
    vertical_file = out / "vertical_scale.json"
    vertical = json.loads(vertical_file.read_text())["vertical_scale"] if vertical_file.is_file() else None
    sweep = sorted((report.sweep_row(out / "trim_sweep" / f"{t:g}", t) for t in trims), key=lambda r: r["trim"])
    verdict = report.sweep_verdict(sweep) if len(sweep) >= 2 else None
    return Outcome(out, origin, results, vertical, sweep, verdict)


def open_folder(path: Path) -> None:
    """Show `path` in the system file manager."""
    if sys.platform == "win32":
        os.startfile(path)  # noqa: S606 (a local folder chosen by the user)
    else:
        subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(path)])
