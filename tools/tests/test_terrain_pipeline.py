"""Tests for terrarium encoding, generation and validation (requires the terrain dependency group)."""

from __future__ import annotations

import io
import zipfile
from typing import TYPE_CHECKING

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")
pytest.importorskip("rasterio")

import rasterio  # noqa: E402
from rasterio.transform import from_origin  # noqa: E402

from terrain import (  # noqa: E402
    ardupilot,
    copernicus,
    generate,
    jobs,
    metadata,
    terrarium,
    tilemath,
    validate,
)

TILE_SIZE = tilemath.TILE_SIZE
TileKey = tilemath.TileKey
# Each straddles a 1-degree corner: Alps 8E/46N, Oslo 10E and the 60N spacing band.
ALPS_JOB = TileKey(generate.JOB_ZOOM, 267, 182)
OSLO_JOB = TileKey(generate.JOB_ZOOM, 270, 148)

if TYPE_CHECKING:
    from pathlib import Path


def ramp(lon, lat):
    """Linear in lon/lat, so bilinear interpolation must reproduce it exactly."""
    return 100.0 * lon + 10.0 * lat


def write_cell(
    directory: Path,
    lat: int,
    lon: int,
    res_x: float,
    res_y: float,
    void: tuple[slice, slice] | None = None,
) -> str:
    """A Copernicus-style cell: pixel centers on whole multiples of the resolution, area-referenced."""
    width, height = round(1 / res_x), round(1 / res_y)
    lons = lon + np.arange(width) * res_x
    lats = lat + 1 - np.arange(height) * res_y
    data = ramp(lons[None, :], lats[:, None]).astype(np.float32)
    if void is not None:
        data[void] = copernicus.NODATA
    path = directory / f"cell_{lat}_{lon}.tif"
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        width=width,
        height=height,
        count=1,
        dtype="float32",
        crs="EPSG:4326",
        transform=from_origin(lon - res_x / 2, lat + 1 + res_y / 2, res_x, res_y),
    ) as dataset:
        dataset.write(data, 1)
    return str(path)


def expected_job_ramp(job: TileKey):
    lons, lats = validate.pixel_center_lonlats(job)
    return ramp(lons[None, :], lats[:, None])


def alps_cells(
    directory: Path,
    skip: tuple[int, int] | None = None,
    void_in_45_7: tuple[slice, slice] | None = None,
) -> dict[tuple[int, int], str]:
    """The four 0.01-degree cells under ALPS_JOB, optionally one missing or one with a void."""
    return {
        (lat, lon): write_cell(
            directory, lat, lon, 0.01, 0.01, void_in_45_7 if (lat, lon) == (45, 7) else None
        )
        for lat in (45, 46)
        for lon in (7, 8)
        if (lat, lon) != skip
    }


def test_encode_decode_round_trip_within_half_a_quantum() -> None:
    heights = np.array([[-432.7, 0.0, 0.001953], [8848.86, 1234.5678, -32768.0]], dtype=np.float32)
    decoded = terrarium.decode(terrarium.encode(heights))
    assert np.abs(decoded - heights).max() <= 0.5 / 256 + 1e-4


def test_encode_matches_the_terrarium_formula() -> None:
    rgb = terrarium.encode(np.array([[0.0, 1.5]], dtype=np.float32))
    assert rgb[0, 0].tolist() == [128, 0, 0]
    assert rgb[0, 1].tolist() == [128, 1, 128]


def test_encode_clamps_out_of_range_heights() -> None:
    rgb = terrarium.encode(np.array([[-40000.0, 40000.0]], dtype=np.float32))
    assert rgb[0, 0].tolist() == [0, 0, 0]
    assert rgb[0, 1].tolist() == [255, 255, 255]


def test_downsample_averages_heights_not_rgb() -> None:
    heights = np.array([[255.0, 257.0], [255.0, 257.0]], dtype=np.float32)
    assert terrarium.downsample(heights).tolist() == [[256.0]]


def test_png_round_trip_and_sea_tile(tmp_path: Path) -> None:
    rng = np.random.default_rng(0)
    heights = rng.uniform(-100, 5000, size=(TILE_SIZE, TILE_SIZE)).astype(np.float32)
    path = tmp_path / "tile.png"
    path.write_bytes(terrarium.png_bytes(heights))
    decoded = terrarium.read_heights(path)
    assert np.abs(decoded - heights).max() <= terrarium.HEIGHT_STEP / 2 + 1e-3
    steps = decoded / terrarium.HEIGHT_STEP
    assert np.array_equal(steps, np.rint(steps))

    sea = np.zeros((TILE_SIZE, TILE_SIZE), dtype=np.float32)
    assert terrarium.tile_png_bytes(sea) is terrarium.sea_png_bytes()
    assert not terrarium.heights_from_bytes(terrarium.sea_png_bytes()).any()
    near_sea = np.full((TILE_SIZE, TILE_SIZE), terrarium.HEIGHT_STEP / 4, dtype=np.float32)
    assert terrarium.tile_png_bytes(near_sea) is terrarium.sea_png_bytes()


def test_quantize_rounds_to_nearest_step_without_bias() -> None:
    step = terrarium.HEIGHT_STEP
    heights = np.array([0.1, 0.13, -0.13, 1234.374, 1234.376, -0.0], dtype=np.float32)
    assert terrarium.quantize(heights).tolist() == [0.0, 0.25, -0.25, 1234.25, 1234.5, 0.0]
    rng = np.random.default_rng(0)
    samples = rng.uniform(-500, 9000, 100_000)
    error = terrarium.quantize(samples).astype(np.float64) - samples
    assert np.abs(error).max() <= step / 2 + 1e-3
    assert abs(error.mean()) < 1e-3


def test_tilejson_describes_the_published_tiles() -> None:
    doc = metadata.tilejson("https://example.test", "v1")
    assert doc["tiles"] == ["https://example.test/v1/terrarium/{z}/{x}/{y}.png"]
    assert (doc["minzoom"], doc["maxzoom"]) == (0, tilemath.MAX_ZOOM)
    assert doc["height_step_m"] == terrarium.HEIGHT_STEP
    assert doc["vertical_datum"] == "EGM2008"
    assert "WorldDEM-30" in str(doc["attribution"])
    assert "WorldDEM-90" in str(doc["attribution"])


def test_png_bytes_rejects_wrong_tile_size() -> None:
    with pytest.raises(ValueError):
        terrarium.png_bytes(np.zeros((128, 128), dtype=np.float32))


def test_ocean_job_writes_full_pyramid_and_marker(tmp_path: Path) -> None:
    job = TileKey(generate.JOB_ZOOM, 10, 20)
    assert generate.run_job(tmp_path, job, {}, force=False) == "ocean"
    tiles = sorted((tmp_path / "terrarium").glob("*/*/*.png"))
    assert len(tiles) == 256 + 64 + 16 + 4 + 1
    assert generate.tile_path(
        tmp_path, TileKey(tilemath.MAX_ZOOM, 10 * 16 + 15, 20 * 16 + 15)
    ).is_file()
    assert jobs.completed_jobs(tmp_path) == {job: "ocean"}
    assert generate.run_job(tmp_path, job, {}, force=False) == "skipped"


def test_job_pyramid_places_tiles_by_position(tmp_path: Path) -> None:
    job = TileKey(generate.JOB_ZOOM, 1, 2)
    heights = np.zeros((generate.JOB_PIXELS, generate.JOB_PIXELS), dtype=np.float32)
    heights[:TILE_SIZE, -TILE_SIZE:] = 100.0
    generate.write_job_pyramid(tmp_path, job, heights)

    north_east = terrarium.read_heights(
        generate.tile_path(tmp_path, TileKey(tilemath.MAX_ZOOM, 16 + 15, 32))
    )
    assert np.allclose(north_east, 100.0)
    assert not terrarium.read_heights(
        generate.tile_path(tmp_path, TileKey(tilemath.MAX_ZOOM, 16, 32))
    ).any()
    job_tile = terrarium.read_heights(generate.tile_path(tmp_path, job))
    assert np.allclose(job_tile[:16, -16:], 100.0)
    assert not job_tile[16:, :].any()


def test_build_overviews_fills_missing_children_with_sea(tmp_path: Path) -> None:
    generate.write_tile(
        tmp_path,
        TileKey(generate.JOB_ZOOM, 0, 0),
        np.full((TILE_SIZE, TILE_SIZE), 400.0, np.float32),
    )
    generate.build_overviews(tmp_path)

    parent = TileKey(generate.JOB_ZOOM - 1, 0, 0)
    overview = terrarium.read_heights(generate.tile_path(tmp_path, parent))
    assert np.allclose(overview[: TILE_SIZE // 2, : TILE_SIZE // 2], 400.0)
    assert not overview[TILE_SIZE // 2 :, :].any()
    assert generate.tile_path(tmp_path, TileKey(0, 0, 0)).is_file()


def test_lattice_bounds_land_on_pixel_edges() -> None:
    res = 1 / 3600
    west, south, east, north = generate.lattice_bounds(7.0001, 46.0001, 7.9999, 46.9999, res, res)
    for edge in (west, south, east, north):
        assert (edge / res - 0.5) == pytest.approx(round(edge / res - 0.5), abs=1e-6)
    assert west <= 7.0001 and south <= 46.0001 and east >= 7.9999 and north >= 46.9999
    assert 7.0001 - west < res and east - 7.9999 < res


def test_job_matches_source_exactly_across_cell_borders(tmp_path: Path) -> None:
    cells = alps_cells(tmp_path)
    heights, voids = generate.warp_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))
    assert voids == 0
    assert np.abs(heights - expected_job_ramp(ALPS_JOB)).max() < 0.01


def test_job_spanning_a_longitude_spacing_band_has_no_gaps(tmp_path: Path) -> None:
    res_y = 1 / 120
    cells = {
        (lat, lon): write_cell(tmp_path, lat, lon, (1.5 if lat == 59 else 2.0) * res_y, res_y)
        for lat in (59, 60)
        for lon in (9, 10, 11)
    }
    heights, voids = generate.warp_job(OSLO_JOB, generate.job_cells(OSLO_JOB, cells))
    assert voids == 0
    assert np.abs(heights - expected_job_ramp(OSLO_JOB)).max() < 0.01


def test_missing_cells_are_sea_level_and_land_is_untouched(tmp_path: Path) -> None:
    cells = alps_cells(tmp_path, skip=(45, 8))
    heights, _ = generate.warp_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))
    lons, lats = validate.pixel_center_lonlats(ALPS_JOB)
    in_missing = (lats[:, None] < 45.95) & (lons[None, :] > 8.05)
    in_land = (lats[:, None] > 46.05) | (lons[None, :] < 7.95)
    assert in_missing.any() and not heights[in_missing].any()
    assert np.abs(heights - expected_job_ramp(ALPS_JOB))[in_land].max() < 0.01


def test_void_filling_leaves_neighboring_ocean_at_sea_level(tmp_path: Path) -> None:
    cells = alps_cells(tmp_path, skip=(45, 8), void_in_45_7=(slice(18, 21), slice(85, 88)))
    mosaic = generate.mosaic_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))
    assert mosaic.voids_filled == 9
    lats = mosaic.transform.f + (np.arange(mosaic.heights.shape[0]) + 0.5) * mosaic.transform.e
    lons = mosaic.transform.c + (np.arange(mosaic.heights.shape[1]) + 0.5) * mosaic.transform.a
    ocean = (lats[:, None] < 46) & (lons[None, :] >= 8)
    assert np.isnan(mosaic.heights[ocean]).all()


def test_small_voids_are_interpolated_not_sea_level(tmp_path: Path) -> None:
    void = (slice(18, 21), slice(85, 88))
    cells = alps_cells(tmp_path, void_in_45_7=void)
    heights, voids = generate.warp_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))
    assert voids == 9
    assert np.abs(heights - expected_job_ramp(ALPS_JOB)).max() < 2.0


def test_gap_between_cells_on_different_lattices_is_filled_not_sea(tmp_path: Path) -> None:
    # GLO-90 (3 arc-second) north of GLO-30 (1 arc-second): their pixel areas leave a gap.
    cells = {
        (45, 7): write_cell(tmp_path, 45, 7, 0.01, 0.01),
        (45, 8): write_cell(tmp_path, 45, 8, 0.01, 0.01),
        (46, 7): write_cell(tmp_path, 46, 7, 0.04, 0.04),
        (46, 8): write_cell(tmp_path, 46, 8, 0.04, 0.04),
    }
    heights, voids = generate.warp_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))
    assert voids > 0
    assert heights.min() > 1000.0
    assert np.abs(heights - expected_job_ramp(ALPS_JOB)).max() < 0.5


def test_unfillable_voids_fail_the_job(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(generate, "VOID_FILL_PIXELS", 1)
    cells = alps_cells(tmp_path, void_in_45_7=(slice(5, 35), slice(78, 98)))
    with pytest.raises(RuntimeError, match="could not be filled"):
        generate.warp_job(ALPS_JOB, generate.job_cells(ALPS_JOB, cells))


def test_source_sampler_interpolates_across_east_and_south_edges(tmp_path: Path) -> None:
    sampler = validate.SourceSampler(alps_cells(tmp_path))
    try:
        for lon, lat in ((7.9951, 46.5), (7.5, 46.0049), (7.9951, 46.0049), (7.123, 45.456)):
            assert sampler.height(lon, lat) == pytest.approx(ramp(lon, lat), abs=1e-3)
        assert sampler.height(20.5, 46.5) == terrarium.SEA_LEVEL
    finally:
        sampler.close()


def test_fit_offset_recovers_a_known_shift() -> None:
    rows, cols = np.mgrid[0:200, 0:200].astype(np.float64)

    def field(x, y):
        return 300 * np.sin(x / 17) * np.cos(y / 23)

    ours = field(cols, rows)
    reference = field(cols + 0.3, rows - 0.2) + 5.0
    offset_x, offset_y, bias = validate.fit_offset(ours, reference)
    assert offset_x == pytest.approx(0.3, abs=0.02)
    assert offset_y == pytest.approx(-0.2, abs=0.02)
    assert bias == pytest.approx(5.0, abs=0.5)


def test_parse_hgt_marks_voids_and_rejects_non_square() -> None:
    data = np.array([[1, 2], [ardupilot.HGT_VOID, 4]], dtype=">i2").tobytes()
    grid = ardupilot.parse_hgt(data)
    assert grid[0].tolist() == [1.0, 2.0]
    assert np.isnan(grid[1, 0])
    with pytest.raises(ValueError):
        ardupilot.parse_hgt(b"\x00" * 6)


def test_hgt_sampling_is_pixel_is_point_with_shared_edges(tmp_path: Path) -> None:
    cells = ardupilot.HgtCells(tmp_path)
    spacing = 4
    for lat, lon in ((46, 7), (46, 8)):
        offsets = np.arange(spacing + 1) / spacing
        lons = lon + offsets
        lats = lat + 1 - offsets
        cells._cells[(lat, lon)] = ramp(lons[None, :], lats[:, None]).astype(np.float32)
    cells._cells[(45, 7)] = None
    lons = np.array([7.0, 7.3, 7.999, 8.0, 8.6])
    lats = np.array([46.999, 46.55, 46.0])
    assert np.allclose(cells.sample_grid(lons, lats), ramp(lons[None, :], lats[:, None]), atol=1e-3)
    assert np.isnan(cells.sample_grid(np.array([7.5]), np.array([45.5]))).all()
    assert ardupilot.hgt_name(-33, -70) == "S33W070"


def test_fit_offset_strides_large_grids_without_losing_accuracy() -> None:
    side = 2100
    assert side * side // validate.MAX_FIT_POINTS >= 4
    rows, cols = np.mgrid[0:side, 0:side].astype(np.float32)

    def field(x, y):
        return 300 * np.sin(x / 17) * np.cos(y / 23)

    offset_x, offset_y, bias = validate.fit_offset(
        field(cols, rows), field(cols - 0.25, rows + 0.4) - 3.0
    )
    assert offset_x == pytest.approx(-0.25, abs=0.02)
    assert offset_y == pytest.approx(0.4, abs=0.02)
    assert bias == pytest.approx(-3.0, abs=0.5)


def test_hgt_sampling_is_exact_across_row_chunks(tmp_path: Path) -> None:
    cells = ardupilot.HgtCells(tmp_path)
    spacing = 1200
    offsets = np.arange(spacing + 1) / spacing
    cells._cells[(46, 7)] = ramp(7 + offsets[None, :], 47 - offsets[:, None]).astype(np.float32)
    lats = 47 - (np.arange(1500) + 0.5) / 1500
    lons = np.array([7.001, 7.5, 7.999])
    assert np.allclose(cells.sample_grid(lons, lats), ramp(lons[None, :], lats[:, None]), atol=1e-2)


def test_hgt_cells_keep_a_bounded_lru(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def archive(name: str) -> bytes:
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w") as bundle:
            bundle.writestr(f"{name}.hgt", np.zeros((3, 3), dtype=">i2").tobytes())
        return buffer.getvalue()

    cells = ardupilot.HgtCells(tmp_path)
    monkeypatch.setattr(cells, "_download", archive)
    keys = [(46, lon) for lon in range(ardupilot.MAX_CACHED_CELLS + 2)]
    for lat, lon in keys[: ardupilot.MAX_CACHED_CELLS]:
        cells.cell(lat, lon)
    cells.cell(*keys[0])
    for lat, lon in keys[ardupilot.MAX_CACHED_CELLS :]:
        cells.cell(lat, lon)
    assert len(cells._cells) == ardupilot.MAX_CACHED_CELLS
    assert keys[0] in cells._cells
    assert keys[1] not in cells._cells and keys[2] not in cells._cells
