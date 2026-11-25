import numpy as np
import pytest
import rasterio
from rasterio.transform import from_origin

from terrain_icp_tools import dem, plyio, synthetic

ORIGIN = (1000.0, 2000.0, 100.0)


def test_dem_points_are_local_pixel_centres(tmp_path):
    synthetic.write_dem(tmp_path / "dem.tif", ORIGIN)
    p = dem.dem_points(tmp_path / "dem.tif", ORIGIN)

    assert len(p) == 200 * 200 - 9  # the 3 x 3 nodata corner is dropped
    assert p[:, :2].min() == pytest.approx(-99.5) and p[:, :2].max() == pytest.approx(99.5)
    # Heights relative to h0 match the analytic terrain (float32 storage).
    np.testing.assert_allclose(p[:, 2], synthetic.height(p[:, 0], p[:, 1]), atol=1e-4)


def test_step_subsamples_rows_and_columns(tmp_path):
    synthetic.write_dem(tmp_path / "dem.tif", ORIGIN)
    out = tmp_path / "dem.ply"
    n = dem.dem_to_ply(tmp_path / "dem.tif", out, ORIGIN, step=4)

    assert n == 50 * 50 - 1  # of the nodata corner only pixel (0, 0) is sampled, and it is dropped
    assert len(plyio.read_points(out)) == n


def test_geographic_dem_is_rejected(tmp_path):
    path = tmp_path / "geo.tif"
    with rasterio.open(path, "w", driver="GTiff", height=2, width=2, count=1, dtype="float32",
                       crs="+proj=longlat +datum=WGS84 +no_defs", transform=from_origin(0, 1, 0.001, 0.001)) as d:
        d.write(np.zeros((2, 2), np.float32), 1)
    with pytest.raises(ValueError, match="projected"):
        dem.dem_points(path, (0.0, 0.0, 0.0))
