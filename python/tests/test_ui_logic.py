"""The UI's logic (ui_logic.py) without Streamlit: origin from lat/lon, initial alignment files, the whole run."""

import json

import numpy as np
import pytest
import rasterio
from rasterio.transform import from_origin

from terrain_icp_tools import synthetic, ui_logic

ORIGIN = (1000.0, 2000.0, 100.0)


def test_latlon_origin_round_trip(tmp_path):
    synthetic.write_dem(tmp_path / "dem.tif", ORIGIN, half=10)
    lat, lon = synthetic.origin_latlon(ORIGIN)
    e, n, h = ui_logic.origin_from_latlon(tmp_path / "dem.tif", lat, lon, 100.0)
    assert (e, n, h) == pytest.approx(ORIGIN, abs=1e-3)  # 8 printed decimals of a degree are ~1 mm


def test_latlon_origin_needs_a_metric_projection(tmp_path):
    path = tmp_path / "geographic.tif"
    with rasterio.open(path, "w", driver="GTiff", width=4, height=4, count=1, dtype="float32",
                       crs="+proj=longlat +datum=WGS84 +no_defs", transform=from_origin(0, 0, 1e-4, 1e-4)) as dst:
        dst.write(np.zeros((1, 4, 4), np.float32))
    with pytest.raises(ValueError, match="not projected in metres"):
        ui_logic.origin_from_latlon(path, 0.0, 0.0, 0.0)


def test_initial_alignment_files(tmp_path):
    simple = tmp_path / "simple.json"
    simple.write_text(json.dumps({"east_m": 2.0, "north_m": -2.0, "yaw_deg": 1.5}))
    init = json.loads(ui_logic.write_init(simple, tmp_path / "a.json").read_text())
    assert init == {"transform": {"translation_m": [2.0, -2.0, 0.0], "rotation_deg": {"yaw": 1.5}}}

    results = tmp_path / "results.json"  # a previous terrain-icp output is passed on as is
    transform = {"scale": 1.01, "rotation_deg": {"yaw": 2.0, "pitch": 0.0, "roll": 0.0}, "translation_m": [1, 2, 3]}
    results.write_text(json.dumps({"fit": {}, "transform": transform}))
    assert json.loads(ui_logic.write_init(results, tmp_path / "b.json").read_text()) == {"transform": transform}

    simple.write_text(json.dumps({"dx": 1.0}))
    with pytest.raises(ValueError, match="unknown keys"):
        ui_logic.write_init(simple, tmp_path / "c.json")


def test_run_alignment_from_latlon_recovers_the_synthetic_scene(tmp_path, exe):
    synthetic.write_scene(tmp_path / "scene", ORIGIN)  # scale 1.005, yaw 2°, translation (2, -2, 1)
    dem = tmp_path / "scene" / "dem.tif"
    origin = ui_logic.origin_from_latlon(dem, *synthetic.origin_latlon(ORIGIN), ORIGIN[2])
    steps = []
    o = ui_logic.run_alignment(dem, tmp_path / "scene" / "mesh.ply", origin, tmp_path / "out", exe=exe,
                               progress=lambda done, total, label: steps.append((done, total)))

    t = o.results["transform"]
    assert t["scale"] == pytest.approx(1.005, abs=1e-4)
    assert t["rotation_deg"]["yaw"] == pytest.approx(2.0, abs=0.01)
    assert t["translation_m"] == pytest.approx([2.0, -2.0, 1.0], abs=0.02)
    assert o.vertical_scale == pytest.approx(1.005, abs=5e-4)
    assert [r["trim"] for r in o.sweep] == [0.1, 0.2, 0.3]
    assert o.verdict[0].startswith("**Stable**") and o.verdict[1].startswith("**Stable from trim 0.1**")
    assert o.residual_map.is_file() and o.histogram.is_file() and o.summary.is_file()
    assert steps[0] == (0, 6) and steps[-1] == (6, 6)  # DEM, align, 3 trims, report
