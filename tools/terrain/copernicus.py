"""Copernicus DEM source cells on AWS Open Data (GLO-30, with GLO-90 filling its gaps)."""

from __future__ import annotations

import math
import re
from typing import TYPE_CHECKING, NamedTuple

from common.net import download_file

if TYPE_CHECKING:
    from collections.abc import Mapping
    from pathlib import Path

NODATA = -32767.0
# Below any land surface on Earth; also catches NODATA after resampling blends it.
MIN_VALID_HEIGHT = -1000.0
# Covers the job mosaic padding: four of the coarsest (10 arc-second, above 85 degrees) pixels.
CELL_BUFFER_DEG = 0.02


class Dataset(NamedTuple):
    base_url: str
    arcsec_code: str


GLO30 = Dataset("https://copernicus-dem-30m.s3.eu-central-1.amazonaws.com", "10")
# GLO-30 omits some cells (Armenia, Azerbaijan and their borders) that GLO-90 covers.
GLO90 = Dataset("https://copernicus-dem-90m.s3.eu-central-1.amazonaws.com", "30")
DATASETS = (GLO30, GLO90)

_CELL_PATTERN = re.compile(r"Copernicus_DSM_COG_(\d+)_([NS])(\d{2})_00_([EW])(\d{3})_00_DEM")


def cell_name(lat: int, lon: int, dataset: Dataset = GLO30) -> str:
    """Name of the 1x1 degree cell whose south-west corner is (lat, lon)."""
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    return (
        f"Copernicus_DSM_COG_{dataset.arcsec_code}_{ns}{abs(lat):02d}_00_{ew}{abs(lon):03d}_00_DEM"
    )


def parse_cell_name(name: str) -> tuple[int, int]:
    match = _CELL_PATTERN.fullmatch(name)
    if not match:
        raise ValueError(f"Not a Copernicus cell name: {name!r}")
    _, ns, lat, ew, lon = match.groups()
    return (int(lat) if ns == "N" else -int(lat)), (int(lon) if ew == "E" else -int(lon))


def cell_url(name: str, dataset: Dataset = GLO30) -> str:
    return f"{dataset.base_url}/{name}/{name}.tif"


def cells_for_bounds(
    west: float,
    south: float,
    east: float,
    north: float,
    cells: Mapping[tuple[int, int], str],
    buffer_deg: float = CELL_BUFFER_DEG,
) -> dict[tuple[int, int], str]:
    """The cells intersecting the buffered lon/lat box; absent cells are ocean."""
    lats = range(
        max(math.floor(south - buffer_deg), -90), min(math.floor(north + buffer_deg), 89) + 1
    )
    lons = range(
        max(math.floor(west - buffer_deg), -180), min(math.floor(east + buffer_deg), 179) + 1
    )
    return {(lat, lon): cells[(lat, lon)] for lat in lats for lon in lons if (lat, lon) in cells}


def _tile_list(cache_dir: Path, dataset: Dataset) -> list[str]:
    path = cache_dir / f"tileList_{dataset.arcsec_code}.txt"
    if not path.is_file():
        cache_dir.mkdir(parents=True, exist_ok=True)
        download_file(f"{dataset.base_url}/tileList.txt", path)
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def load_cells(cache_dir: Path) -> dict[tuple[int, int], str]:
    """Source URL per (lat, lon) cell, preferring the first dataset that has it."""
    cells: dict[tuple[int, int], str] = {}
    for dataset in DATASETS:
        for name in _tile_list(cache_dir, dataset):
            cells.setdefault(parse_cell_name(name), cell_url(name, dataset))
    return cells
