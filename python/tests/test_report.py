import json

import numpy as np
import pytest

from terrain_icp_tools import plyio, report


def test_vertical_scale_recovers_a_height_stretch():
    rng = np.random.default_rng(1)
    z = rng.uniform(-10.0, 30.0, 5000)
    nz = rng.uniform(0.7, 1.0, 5000)
    k = 1.02  # moving heights 2 % taller than the reference
    distance = nz * ((1.0 - 1.0 / k) * (z - z.mean()) + 0.05) + rng.normal(0.0, 0.02, 5000)

    fit = report.vertical_scale(z, distance, nz)
    assert fit["vertical_scale"] == pytest.approx(k, abs=5e-4)
    assert fit["offset_m"] == pytest.approx(0.05, abs=2e-3)


def fake_results(tmp_path):
    """An align output directory written by hand: 2000 points, 10 % rejected, 2 cm residuals."""
    rng = np.random.default_rng(2)
    xyz = np.column_stack([rng.uniform(-50, 50, (2000, 2)), rng.uniform(0, 20, 2000)])
    distance = rng.normal(0.0, 0.02, 2000)
    inlier = (np.arange(2000) % 10 != 0).astype(float)
    plyio.write_points(tmp_path / "residuals.ply", xyz,
                       {"distance": distance, "inlier": inlier, "normal_z": np.ones(2000)})
    results = {
        "convention": "x_ref = scale * R * x_moving + translation",
        "inputs": {"reference": "reference.ply", "moving": "moving.ply", "reference_points": 40000,
                   "moving_points": 2000, "reference_normals": "pca"},
        "options": {"estimate_scale": True, "max_distance_m": None, "trim_ratio": 0.1, "max_iterations": 100,
                    "max_condition": 1000.0, "normal_neighbours": 10},
        "transform": {"scale": 1.005, "rotation_deg": {"yaw": 2.0, "pitch": 0.001, "roll": -0.002},
                      "rotation_matrix": np.eye(3).tolist(), "translation_m": [2.0, -2.0, 1.0],
                      "moving_centroid_shift_m": [1.9, -2.1, 1.0]},
        "fit": {"converged": True, "iterations": 11, "inliers": 1800, "inlier_ratio": 0.9, "rms_m": 0.02,
                "mean_m": 0.0, "abs_p50_m": 0.013, "abs_p95_m": 0.04},
        "conditioning": {"condition_number": None, "eigenvalues": [0.0] * 7, "weak_parameters": ["rotation_z"]},
        "warnings": ["flat terrain: yaw and horizontal translation are poorly constrained."],
    }
    (tmp_path / "results.json").write_text(json.dumps(results))


def test_report_writes_figures_and_summary(tmp_path):
    fake_results(tmp_path)
    summary = report.make_report(tmp_path, tmp_path / "report", vertical=True)

    assert (tmp_path / "report" / "residual_map.png").stat().st_size > 10_000
    assert (tmp_path / "report" / "residual_histogram.png").stat().st_size > 5_000
    text = summary.read_text(encoding="utf-8")
    assert "| Scale moving → reference (uniform, ICP) | 1.00500 (+0.500 %) |" in text
    assert "+2.0000° counter-clockwise (bearings change by -2.0000°)" in text
    assert "Vertical scale moving → reference" in text
    assert "| Condition number | inf |" in text
    assert "- flat terrain: yaw and horizontal translation are poorly constrained." in text
