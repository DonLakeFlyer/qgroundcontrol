"""Warp Copernicus cells into job tiles (JOB_ZOOM) and write their MAX_ZOOM..JOB_ZOOM pyramid."""

from __future__ import annotations

import math
import os
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import TYPE_CHECKING, Any, NamedTuple

import numpy as np
import rasterio
from rasterio.fill import fillnodata
from rasterio.merge import merge
from rasterio.transform import from_bounds, from_origin
from rasterio.warp import Resampling, reproject
from rasterio.windows import from_bounds as from_bounds_window

from . import terrarium
from .copernicus import MIN_VALID_HEIGHT, cells_for_bounds
from .jobs import OCEAN_STATUS, done_marker, mark_done
from .tilemath import (
    JOB_ZOOM,
    MAX_ZOOM,
    TILE_SIZE,
    TileKey,
    children,
    lonlat_bounds,
    mercator_bounds,
)

if TYPE_CHECKING:
    from collections.abc import Mapping
    from pathlib import Path

    from affine import Affine
    from rasterio.io import DatasetReader

JOB_TILES = 1 << (MAX_ZOOM - JOB_ZOOM)
JOB_PIXELS = JOB_TILES * TILE_SIZE
# About 3 km at 1 arc-second: bridges genuine source voids without inventing large areas.
VOID_FILL_PIXELS = 100

GDAL_ENV: dict[str, Any] = {
    "GDAL_DISABLE_READDIR_ON_OPEN": "EMPTY_DIR",
    "CPL_VSIL_CURL_ALLOWED_EXTENSIONS": ".tif",
    "GDAL_HTTP_MULTIRANGE": "YES",
    "GDAL_HTTP_MERGE_CONSECUTIVE_RANGES": "YES",
    "GDAL_HTTP_MAX_RETRY": "5",
    "GDAL_HTTP_RETRY_DELAY": "2",
    "VSI_CACHE": "TRUE",
    # MB per process; GDAL's default (5% of physical RAM) multiplies across pool workers.
    "GDAL_CACHEMAX": 512,
}


def tile_path(out: Path, key: TileKey) -> Path:
    return out / "terrarium" / str(key.z) / str(key.x) / f"{key.y}.png"


def job_cells(job: TileKey, cells: Mapping[tuple[int, int], str]) -> dict[tuple[int, int], str]:
    return cells_for_bounds(*lonlat_bounds(job), cells)


def lattice_bounds(
    west: float, south: float, east: float, north: float, res_x: float, res_y: float
) -> tuple[float, float, float, float]:
    """Expand a box to whole source pixels; their centers sit on whole multiples of the resolution."""

    def lower(value: float, res: float) -> float:
        return (math.floor(value / res + 0.5) - 0.5) * res

    def upper(value: float, res: float) -> float:
        return (math.ceil(value / res + 0.5) - 0.5) * res

    return lower(west, res_x), lower(south, res_y), upper(east, res_x), upper(north, res_y)


class Mosaic(NamedTuple):
    heights: np.ndarray
    transform: Affine
    voids_filled: int


def _merge_same_resolution(
    datasets: list[DatasetReader],
    bounds: tuple[float, float, float, float],
    res: tuple[float, float],
) -> tuple[np.ndarray, Affine]:
    """Exact pixel copy: cells sharing a resolution share one pixel lattice.

    Never merge mixed resolutions directly: rasterio rounds the coarser source windows to
    whole pixels, shifting that data by up to a pixel (~23 m at 60 degrees north).
    """
    merged, transform = merge(
        datasets,
        bounds=lattice_bounds(*bounds, *res),
        res=res,
        nodata=np.nan,
        dtype="float32",
    )
    heights = merged[0]
    heights[heights < MIN_VALID_HEIGHT] = np.nan
    return heights, transform


def mosaic_job(job: TileKey, cells: Mapping[tuple[int, int], str]) -> Mosaic:
    """Merge the job's source cells into one geographic grid, filling interior voids.

    Interpolating from a single mosaic, rather than warping each cell on its own, keeps
    bilinear sampling continuous across 1-degree cell edges. Cells on a coarser lattice (a
    longitude-spacing band or GLO-90 gap fill) are warped onto the finest one.
    """
    datasets = [rasterio.open(url) for url in cells.values()]
    try:
        res_x = min(ds.res[0] for ds in datasets)
        res_y = min(ds.res[1] for ds in datasets)
        # The bilinear kernel reaches two of the coarsest source pixels past the job edge;
        # the mosaic is padded twice that so its own ragged outer edge stays clear of it.
        margin_x = 2 * max(ds.res[0] for ds in datasets)
        margin_y = 2 * max(ds.res[1] for ds in datasets)
        west, south, east, north = lonlat_bounds(job)
        padded = (
            west - 2 * margin_x,
            south - 2 * margin_y,
            east + 2 * margin_x,
            north + 2 * margin_y,
        )
        left, bottom, right, top = lattice_bounds(*padded, res_x, res_y)
        transform = from_origin(left, top, res_x, res_y)
        shape = (round((top - bottom) / res_y), round((right - left) / res_x))
        heights = np.full(shape, np.nan, dtype=np.float32)
        groups: dict[tuple[float, float], list[DatasetReader]] = defaultdict(list)
        for dataset in datasets:
            groups[dataset.res].append(dataset)
        # Finest first, so where groups overlap at a band edge the finer data wins.
        for res in sorted(groups, key=lambda res: res[0] * res[1]):
            group_heights, group_transform = _merge_same_resolution(groups[res], padded, res)
            if group_transform != transform or group_heights.shape != shape:
                warped = np.full(shape, np.nan, dtype=np.float32)
                reproject(
                    source=group_heights,
                    destination=warped,
                    src_transform=group_transform,
                    src_crs="EPSG:4326",
                    src_nodata=np.nan,
                    dst_transform=transform,
                    dst_crs="EPSG:4326",
                    dst_nodata=np.nan,
                    resampling=Resampling.bilinear,
                    num_threads=1,
                )
                group_heights = warped
            heights = np.where(np.isnan(heights), group_heights, heights)
        # Mixed-resolution merges can leave a NaN column at the mosaic's outer edge; only
        # voids inside the job footprint plus kernel margin can reach the output.
        footprint = from_bounds_window(
            west - margin_x,
            south - margin_y,
            east + margin_x,
            north + margin_y,
            transform=transform,
        )
        (row0, row1), (col0, col1) = footprint.round_offsets().round_lengths().toranges()
        in_footprint = np.zeros(heights.shape, dtype=bool)
        in_footprint[max(row0, 0) : max(row1, 0), max(col0, 0) : max(col1, 0)] = True
        # Coverage is whole 1-degree cells, not file extents: pixel areas on different lattices
        # (GLO-90 beside GLO-30) leave thin gaps between files that must be filled, not sea.
        # A pixel centered on a whole degree is the top row (or first column) of the cell
        # south (or east) of it; the half-pixel nudge also absorbs float noise.
        lat_cells = np.floor(transform.f + (np.arange(shape[0]) + 1) * transform.e).astype(int)
        lon_cells = np.floor(transform.c + (np.arange(shape[1]) + 1) * transform.a).astype(int)
        covered = np.zeros(shape, dtype=bool)
        for lat, lon in cells:
            covered[np.ix_(lat_cells == lat, lon_cells == lon)] = True
        covered &= in_footprint
    finally:
        for dataset in datasets:
            dataset.close()

    void_mask = np.isnan(heights) & covered
    voids = int(void_mask.sum())
    if voids:
        # fillnodata writes into its input and fills ocean too; only voids may take its values.
        filled = fillnodata(
            heights.copy(), mask=~np.isnan(heights), max_search_distance=VOID_FILL_PIXELS
        )
        heights = np.where(void_mask, filled, heights).astype(np.float32)
        unfilled = int((np.isnan(heights) & covered).sum())
        if unfilled:
            # Writing these as sea level would put land at 0 m, the failure terraingen shipped.
            raise RuntimeError(f"{unfilled} void pixels inside source cells could not be filled")
    return Mosaic(heights, transform, voids)


def warp_job(job: TileKey, cells: Mapping[tuple[int, int], str]) -> tuple[np.ndarray, int]:
    """Bilinear-warp the job mosaic into its MAX_ZOOM-resolution mercator grid; returns voids filled."""
    mosaic = mosaic_job(job, cells)
    heights = np.full((JOB_PIXELS, JOB_PIXELS), np.nan, dtype=np.float32)
    reproject(
        source=mosaic.heights,
        destination=heights,
        src_transform=mosaic.transform,
        src_crs="EPSG:4326",
        src_nodata=np.nan,
        dst_transform=from_bounds(*mercator_bounds(job), JOB_PIXELS, JOB_PIXELS),
        dst_crs="EPSG:3857",
        dst_nodata=np.nan,
        resampling=Resampling.bilinear,
        num_threads=1,
    )
    return np.nan_to_num(heights, nan=terrarium.SEA_LEVEL), mosaic.voids_filled


def write_tile(out: Path, key: TileKey, heights: np.ndarray) -> None:
    path = tile_path(out, key)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(terrarium.tile_png_bytes(heights))


def write_job_pyramid(out: Path, job: TileKey, heights: np.ndarray) -> None:
    level = heights
    for z in range(MAX_ZOOM, JOB_ZOOM - 1, -1):
        depth = z - JOB_ZOOM
        for ty in range(1 << depth):
            for tx in range(1 << depth):
                block = level[
                    ty * TILE_SIZE : (ty + 1) * TILE_SIZE, tx * TILE_SIZE : (tx + 1) * TILE_SIZE
                ]
                write_tile(out, TileKey(z, (job.x << depth) + tx, (job.y << depth) + ty), block)
        if z > JOB_ZOOM:
            level = terrarium.downsample(level)


def run_job(out: Path, job: TileKey, cells: Mapping[tuple[int, int], str], force: bool) -> str:
    if done_marker(out, job).exists() and not force:
        return "skipped"
    if cells:
        with rasterio.Env(**GDAL_ENV):
            heights, voids_filled = warp_job(job, cells)
        status = f"{len(cells)} cells, {voids_filled} void px filled"
    else:
        heights = np.zeros((JOB_PIXELS, JOB_PIXELS), dtype=np.float32)
        status = OCEAN_STATUS
    write_job_pyramid(out, job, heights)
    mark_done(out, job, status)
    return status


def generate(
    out: Path,
    jobs: list[TileKey],
    cells: Mapping[tuple[int, int], str],
    workers: int,
    force: bool,
) -> int:
    """Run all jobs in a process pool; returns the number of failed jobs."""
    failures = 0
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(run_job, out, job, job_cells(job, cells), force): job for job in jobs
        }
        for done, future in enumerate(as_completed(futures), start=1):
            job = futures[future]
            try:
                status = future.result()
            except Exception as error:
                failures += 1
                status = f"FAILED: {error}"
            elapsed = time.monotonic() - start
            print(f"[{done}/{len(jobs)} {elapsed:7.1f}s] job {job}: {status}", flush=True)
    return failures


def build_overviews(out: Path) -> None:
    """Build zooms below JOB_ZOOM from existing tiles; missing children are sea level."""
    for z in range(JOB_ZOOM - 1, -1, -1):
        child_dir = out / "terrarium" / str(z + 1)
        parents = {
            TileKey(z, int(path.parent.name) >> 1, int(path.stem) >> 1)
            for path in child_dir.glob("*/*.png")
        }
        missing = 0
        for key in sorted(parents):
            mosaic = np.zeros((2 * TILE_SIZE, 2 * TILE_SIZE), dtype=np.float32)
            for index, child in enumerate(children(key)):
                path = tile_path(out, child)
                if not path.is_file():
                    missing += 1
                    continue
                row, col = divmod(index, 2)
                mosaic[
                    row * TILE_SIZE : (row + 1) * TILE_SIZE, col * TILE_SIZE : (col + 1) * TILE_SIZE
                ] = terrarium.read_heights(path)
            write_tile(out, key, terrarium.downsample(mosaic))
        print(f"z{z}: {len(parents)} tiles ({missing} missing children filled with sea level)")


def default_workers() -> int:
    return os.cpu_count() or 1
