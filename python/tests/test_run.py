"""End to end: synthetic dem.tif + georeferenced mesh.ply -> `run` (dem-to-ply, terrain-icp align, report)."""

import json

import pytest

from terrain_icp_tools import cli, synthetic

ORIGIN = (1000.0, 2000.0, 100.0)


def run_scene(tmp_path, exe, **truth):
    synthetic.write_scene(tmp_path / "scene", ORIGIN, **truth)
    out = tmp_path / "out"
    cli.main(["run", str(tmp_path / "scene" / "dem.tif"), str(tmp_path / "scene" / "mesh.ply"),
              "--origin", *map(str, ORIGIN), "--out", str(out), "--shift-moving", "--scale", "--vertical-scale",
              "--exe", exe])
    return out, json.loads((out / "results.json").read_text())


def reported_vertical_scale(out) -> float:
    text = (out / "summary.md").read_text(encoding="utf-8")
    row = next(line for line in text.splitlines() if line.startswith("| Vertical scale"))
    return float(row.split("|")[2].split()[0])


def test_run_recovers_the_known_transform(tmp_path, exe):
    out, res = run_scene(tmp_path, exe)
    t = res["transform"]
    assert t["rotation_deg"]["yaw"] == pytest.approx(2.0, abs=0.01)
    assert t["scale"] == pytest.approx(1.005, abs=1e-4)
    assert t["translation_m"] == pytest.approx([2.0, -2.0, 1.0], abs=0.02)
    assert res["fit"]["converged"] and res["warnings"] == []
    assert reported_vertical_scale(out) == pytest.approx(1.005, abs=5e-4)  # no height stretch in this scene
    assert (out / "residual_map.png").is_file()


def test_trim_sweep_flags_outliers_above_the_smallest_ratio(tmp_path, exe):
    # 20 % of the vertices lifted 3-20 m: trimming 10 % keeps many of them, 30 % drops them all.
    synthetic.write_scene(tmp_path / "scene", ORIGIN, outliers=0.2)
    out = tmp_path / "out"
    cli.main(["run", str(tmp_path / "scene" / "dem.tif"), str(tmp_path / "scene" / "mesh.ply"),
              "--origin", *map(str, ORIGIN), "--out", str(out), "--shift-moving", "--scale", "--vertical-scale",
              "--trim-sweep", "0.1,0.2,0.3", "--exe", exe])

    text = (out / "summary.md").read_text(encoding="utf-8")
    rows = [line.split("|")[1:-1] for line in text.splitlines() if line.startswith("| 0.")]
    assert [r[0].strip() for r in rows] == ["0.1", "0.2", "0.3"]
    trim_03 = [float(c) for c in rows[2][1:]]  # scale, vertical scale, yaw, rms
    assert trim_03[0] == pytest.approx(1.005, abs=2e-4)
    assert trim_03[1] == pytest.approx(1.005, abs=5e-4)
    assert trim_03[2] == pytest.approx(2.0, abs=0.01)
    assert "**NOT stable**" in text


def test_run_measures_a_vertical_stretch(tmp_path, exe):
    # Mesh heights 3 % exaggerated: x_ref = s R x_moving + t horizontally, but heights map with s / 1.03.
    out, res = run_scene(tmp_path, exe, vertical_scale=1.03)
    assert reported_vertical_scale(out) == pytest.approx(1.005 / 1.03, abs=5e-4)
    # Once the stretch is removed, the ICP sees a pure similarity again.
    assert res["transform"]["scale"] == pytest.approx(1.005, abs=2e-4)
    assert res["transform"]["rotation_deg"]["pitch"] == pytest.approx(0.0, abs=0.01)
