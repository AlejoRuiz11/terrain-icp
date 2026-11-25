import os
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def exe() -> str:
    """The C++ terrain-icp executable: $TERRAIN_ICP_EXE, else any local build tree; skip when there is none."""
    explicit = os.environ.get("TERRAIN_ICP_EXE")
    if explicit:  # an explicit choice must exist: never fall back silently to another build
        assert Path(explicit).is_file(), f"TERRAIN_ICP_EXE does not exist: {explicit}"
        return explicit
    candidates = []
    for preset in ("msvc-release", "gcc-release", "msvc", "gcc"):
        candidates += [REPO / "build" / preset / "terrain-icp.exe", REPO / "build" / preset / "terrain-icp"]
    for c in candidates:
        if c and Path(c).is_file():
            return str(c)
    pytest.skip("terrain-icp not built (set TERRAIN_ICP_EXE)")
