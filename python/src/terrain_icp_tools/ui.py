"""Local Streamlit app (optional `ui` extra): a thin layer over `terrain-icp-tools run`.

All the work is in ui_logic.py, which never touches `st` and is tested without a browser; this file only draws
widgets. Uploads and the default output folder go to a temporary directory, never to the repository."""

from __future__ import annotations

import tempfile
from pathlib import Path

import streamlit as st

from terrain_icp_tools import ui_logic
from terrain_icp_tools.ui_logic import Options, Outcome


def _save_upload(directory: Path, label: str, upload) -> Path:
    """Write an uploaded file into `directory`, keeping its extension; return the path."""
    path = directory / f"{label}{Path(upload.name).suffix.lower()}"
    path.write_bytes(upload.getvalue())
    return path


def _optional(value: float, off: float = 0.0) -> float | None:
    return None if value == off else value


def _trims(text: str) -> tuple[float, ...]:
    return tuple(ui_logic.cli.trim_list(text)) if text.strip() else ()


def _app() -> None:
    st.set_page_config(page_title="terrain-icp", layout="wide")
    st.title("Terrain comparison (ICP)")
    st.caption("Aligns a terrain mesh to a georeferenced DEM. Runs locally: nothing is uploaded anywhere.")

    col_a, col_b = st.columns(2)
    dem_up = col_a.file_uploader("Reference DEM (GeoTIFF)", type=["tif", "tiff"], key="dem")
    mesh_up = col_b.file_uploader("Moving mesh or points (PLY)", type=["ply"], key="mesh")
    shift = col_b.checkbox("The mesh is in the DEM's CRS (shift it by the origin)", value=True)

    st.subheader("Origin")
    st.caption("Local frame for the comparison (for example the landing zone); converted to the DEM's CRS.")
    mode = st.radio("Given as", ["Latitude / longitude", "Easting / northing"], horizontal=True)
    cols = st.columns(3)
    if mode == "Latitude / longitude":
        lat = cols[0].number_input("Latitude [°]", min_value=-90.0, max_value=90.0, value=0.0, format="%.8f")
        lon = cols[1].number_input("Longitude [°]", min_value=-180.0, max_value=180.0, value=0.0, format="%.8f")
    else:
        east = cols[0].number_input("Easting [m]", value=0.0, format="%.3f")
        north = cols[1].number_input("Northing [m]", value=0.0, format="%.3f")
    height = cols[2].number_input("Elevation [m]", value=0.0, format="%.3f",
                                  help="In the DEM's vertical reference.")

    init_up = st.file_uploader("Initial alignment (optional): a results.json of terrain-icp, or "
                               '{"east_m", "north_m", "up_m", "yaw_deg"}', type=["json"], key="init")

    st.subheader("Options")
    cols = st.columns(4)
    scale = cols[0].checkbox("Estimate scale", value=True)
    vertical = cols[1].checkbox("Estimate vertical scale", value=True)
    sweep_text = cols[2].text_input("Trim sweep", value=",".join(f"{t:g}" for t in ui_logic.DEFAULT_SWEEP),
                                    help="Comma-separated trim ratios; empty for none.")
    max_dist = cols[3].number_input("Max pair distance [m] (0: none)", min_value=0.0, value=0.0)
    with st.expander("Advanced"):
        cols = st.columns(3)
        max_iter = cols[0].number_input("Max iterations", min_value=1, value=100)
        max_cond = cols[1].number_input("Max condition number", min_value=1.0, value=1000.0)
        step = cols[2].number_input("DEM step (keep every n-th pixel)", min_value=1, value=1)

    if "workdir" not in st.session_state:
        st.session_state["workdir"] = tempfile.mkdtemp(prefix="terrain-icp-")
    workdir = Path(st.session_state["workdir"])
    exe = st.text_input("terrain-icp executable", value=ui_logic.default_executable() or "",
                        help="Found automatically in the repository's Release build, $TERRAIN_ICP_EXE or PATH.")
    out_text = st.text_input("Output folder", value=str(workdir / "out"))

    if st.button("Align", type="primary", disabled=dem_up is None or mesh_up is None):
        bar = st.progress(0.0, text="Starting")
        try:
            dem = _save_upload(workdir, "dem", dem_up)
            mesh = _save_upload(workdir, "mesh", mesh_up)
            init = _save_upload(workdir, "init_upload", init_up) if init_up else None
            origin = (ui_logic.origin_from_latlon(dem, lat, lon, height) if mode == "Latitude / longitude"
                      else (east, north, height))
            options = Options(scale=scale, vertical_scale=vertical, trim_sweep=_trims(sweep_text),
                              shift_moving=shift, max_dist=_optional(max_dist), max_iter=int(max_iter),
                              max_cond=float(max_cond), step=int(step))
            st.session_state["outcome"] = ui_logic.run_alignment(
                dem, mesh, origin, Path(out_text), options, exe or None, init,
                progress=lambda done, total, label: bar.progress(done / total, text=label))
        except (ValueError, OSError, RuntimeError, SystemExit) as exc:
            st.error(f"Error: {exc}")
        except Exception as exc:  # the executable failed: its message is in the console
            st.error(f"Alignment failed: {exc}")
        finally:
            bar.empty()

    outcome: Outcome | None = st.session_state.get("outcome")
    if outcome is not None:
        _results(outcome)


def _results(o: Outcome) -> None:
    t, fit = o.results["transform"], o.results["fit"]
    rot = t["rotation_deg"]
    st.header("Result")
    st.caption("Transform moving → reference: x_ref = scale · R · x_moving + t; yaw counter-clockwise seen from "
               f"above. Origin E/N/H {o.origin[0]:.3f} / {o.origin[1]:.3f} / {o.origin[2]:.3f} m.")
    cols = st.columns(4)
    cols[0].metric("Scale", f"{t['scale']:.5f}" if o.results["options"]["estimate_scale"] else "not estimated")
    cols[1].metric("Vertical scale", "not estimated" if o.vertical_scale is None else f"{o.vertical_scale:.5f}")
    cols[2].metric("RMS [m]", f"{fit['rms_m']:.4f}")
    cols[3].metric("Inliers", f"{100 * fit['inlier_ratio']:.1f} %")
    cols = st.columns(4)
    for col, name in zip(cols, ("yaw", "pitch", "roll")):
        col.metric(f"{name.capitalize()} [°]", f"{rot[name]:+.4f}")
    cols = st.columns(4)
    for col, axis, value in zip(cols, "ENU", t["translation_m"]):
        col.metric(f"Translation {axis} [m]", f"{value:+.3f}")
    for warning in o.results["warnings"]:
        st.warning(warning)
    if not o.results["warnings"]:
        st.success("No warnings.")

    if o.sweep:
        st.subheader("Trimming sensitivity")
        st.table([{"Trim": f"{r['trim']:g}", "Scale": f"{r['scale']:.5f}",
                   "Vertical scale": "—" if r["vertical_scale"] is None else f"{r['vertical_scale']:.5f}",
                   "Yaw [°]": f"{r['yaw_deg']:+.4f}", "RMS [m]": f"{r['rms_m']:.4f}"} for r in o.sweep])
        if o.verdict:
            verdict, since = o.verdict
            (st.success if verdict.startswith("**Stable") else st.warning)(verdict)
            st.markdown(since)

    st.subheader("Residuals")
    cols = st.columns(2)
    cols[0].image(str(o.residual_map))
    cols[1].image(str(o.histogram))

    st.caption(f"Files written to {o.out} (results.json, residuals.ply, summary.md, figures).")
    if st.button("Open output folder"):
        ui_logic.open_folder(o.out)


if __name__ == "__main__":
    _app()
