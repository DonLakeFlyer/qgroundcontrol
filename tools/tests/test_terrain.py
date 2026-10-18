"""Tests for terrain tile math, Copernicus cell lookup and upload policy."""

from __future__ import annotations

from pathlib import Path

import pytest

from terrain import copernicus, jobs, tilemath, upload

TileKey = tilemath.TileKey


def test_mercator_bounds_cover_world_at_zoom_zero() -> None:
    half = tilemath.MERCATOR_HALF_EXTENT
    assert tilemath.mercator_bounds(TileKey(0, 0, 0)) == pytest.approx((-half, -half, half, half))


def test_lonlat_bounds_put_y_zero_at_the_north_edge() -> None:
    west, south, east, north = tilemath.lonlat_bounds(TileKey(1, 0, 0))
    assert (west, east) == pytest.approx((-180.0, 0.0))
    assert south == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(tilemath.MAX_LATITUDE)


@pytest.mark.parametrize(
    ("lon", "lat"), [(7.65, 45.97), (-122.5, 37.75), (-70.0, -32.65), (0.001, -0.001)]
)
def test_tile_for_lonlat_contains_the_point(lon: float, lat: float) -> None:
    key = tilemath.tile_for_lonlat(lon, lat, tilemath.MAX_ZOOM)
    west, south, east, north = tilemath.lonlat_bounds(key)
    assert west <= lon < east
    assert south <= lat < north


def test_tile_for_lonlat_clamps_to_the_grid() -> None:
    n = 1 << 4
    assert tilemath.tile_for_lonlat(180.0, -90.0, 4) == TileKey(4, n - 1, n - 1)
    assert tilemath.tile_for_lonlat(-180.0, 90.0, 4) == TileKey(4, 0, 0)


def test_mercator_lonlat_round_trip() -> None:
    mx, my = tilemath.lonlat_to_mercator(86.925, 27.988)
    assert tilemath.mercator_to_lonlat(mx, my) == pytest.approx((86.925, 27.988))


def test_tiles_in_bbox_spans_rows_and_columns() -> None:
    tiles = tilemath.tiles_in_bbox(-1.0, -1.0, 1.0, 1.0, 1)
    assert tiles == [TileKey(1, 0, 0), TileKey(1, 1, 0), TileKey(1, 0, 1), TileKey(1, 1, 1)]
    with pytest.raises(ValueError):
        tilemath.tiles_in_bbox(1.0, 0.0, -1.0, 1.0, 1)


def test_parent_and_children_are_inverse() -> None:
    key = TileKey(8, 133, 91)
    assert [tilemath.parent(child) for child in tilemath.children(key)] == [key] * 4
    assert tilemath.children(key)[0] == TileKey(9, 266, 182)
    with pytest.raises(ValueError):
        tilemath.parent(TileKey(0, 0, 0))


def test_all_tiles_count() -> None:
    assert len(tilemath.all_tiles(3)) == 64


@pytest.mark.parametrize(
    ("lat", "lon", "name"),
    [
        (46, 7, "Copernicus_DSM_COG_10_N46_00_E007_00_DEM"),
        (-33, -70, "Copernicus_DSM_COG_10_S33_00_W070_00_DEM"),
        (0, -1, "Copernicus_DSM_COG_10_N00_00_W001_00_DEM"),
    ],
)
def test_cell_name_uses_south_west_corner_and_round_trips(lat: int, lon: int, name: str) -> None:
    assert copernicus.cell_name(lat, lon) == name
    assert copernicus.parse_cell_name(name) == (lat, lon)


def test_glo90_cells_use_their_own_name_and_bucket() -> None:
    name = copernicus.cell_name(40, 44, copernicus.GLO90)
    assert name == "Copernicus_DSM_COG_30_N40_00_E044_00_DEM"
    assert copernicus.parse_cell_name(name) == (40, 44)
    assert copernicus.cell_url(name, copernicus.GLO90).startswith(copernicus.GLO90.base_url)
    with pytest.raises(ValueError):
        copernicus.parse_cell_name("N40E044")


def test_cells_for_bounds_includes_buffered_neighbors_that_exist() -> None:
    cells = {(46, 7): "n46e7", (45, 7): "n45e7", (46, 8): "n46e8"}
    assert copernicus.cells_for_bounds(7.2, 46.005, 7.8, 46.5, cells) == {
        (45, 7): "n45e7",
        (46, 7): "n46e7",
    }


def test_cells_for_bounds_stays_inside_the_globe() -> None:
    cells = {(-90, -180): "south-west", (89, 179): "north-east"}
    assert copernicus.cells_for_bounds(-180.0, -90.0, 180.0, 90.0, cells) == cells


def test_load_cells_prefers_glo30_and_fills_gaps_from_glo90(tmp_path: Path) -> None:
    glo30 = [copernicus.cell_name(46, 7)]
    glo90 = [
        copernicus.cell_name(46, 7, copernicus.GLO90),
        copernicus.cell_name(40, 44, copernicus.GLO90),
    ]
    (tmp_path / "tileList_10.txt").write_text("\n".join(glo30) + "\n", encoding="utf-8")
    (tmp_path / "tileList_30.txt").write_text("\n".join(glo90) + "\n", encoding="utf-8")
    cells = copernicus.load_cells(tmp_path)
    assert cells == {
        (46, 7): copernicus.cell_url(glo30[0]),
        (40, 44): copernicus.cell_url(glo90[1], copernicus.GLO90),
    }


def test_upload_cache_control_is_immutable_only_for_release_prefixes() -> None:
    assert upload.cache_control_for("v1") == upload.IMMUTABLE_CACHE_CONTROL
    assert upload.cache_control_for("v12") == upload.IMMUTABLE_CACHE_CONTROL
    assert upload.cache_control_for("validation") == upload.SCRATCH_CACHE_CONTROL
    assert upload.cache_control_for("v1-test") == upload.SCRATCH_CACHE_CONTROL


def test_sync_command_targets_private_bucket_without_acl() -> None:
    command = upload.sync_command(Path("out"), "v1", profile="qgc-terrain", dryrun=True)
    assert command[:5] == [
        "aws",
        "s3",
        "sync",
        str(Path("out") / "terrarium"),
        "s3://qgc-terrain-assets/v1/terrarium",
    ]
    assert "--acl" not in command
    assert command[command.index("--content-type") + 1] == "image/png"
    assert command[command.index("--profile") + 1] == "qgc-terrain"
    assert command[-1] == "--dryrun"


def test_scratch_prefixes_mirror_and_release_prefixes_never_delete() -> None:
    assert "--delete" in upload.sync_command(Path("out"), "validation", profile=None, dryrun=False)
    assert "--delete" not in upload.sync_command(Path("out"), "v1", profile=None, dryrun=False)


def test_metadata_commands_use_short_caching_and_content_types() -> None:
    paths = [Path("out/tilejson.json"), Path("out/ATTRIBUTION.txt")]
    commands = upload.metadata_commands(paths, "v1", profile="qgc-terrain", dryrun=True)
    assert [command[4] for command in commands] == [
        "s3://qgc-terrain-assets/v1/tilejson.json",
        "s3://qgc-terrain-assets/v1/ATTRIBUTION.txt",
    ]
    for command, content_type in zip(
        commands, ["application/json", "text/plain; charset=utf-8"], strict=True
    ):
        assert command[command.index("--content-type") + 1] == content_type
        assert command[command.index("--cache-control") + 1] == upload.SCRATCH_CACHE_CONTROL
        assert "--acl" not in command
        assert command[-1] == "--dryrun"


@pytest.mark.parametrize("prefix", ["", "v1/terrarium", "../x", "V1", "-rf"])
def test_sync_command_rejects_unsafe_prefixes(prefix: str) -> None:
    with pytest.raises(ValueError):
        upload.sync_command(Path("out"), prefix, profile=None, dryrun=False)


def job_key(x: int, y: int) -> TileKey:
    return TileKey(tilemath.JOB_ZOOM, x, y)


def test_completed_jobs_reads_marker_statuses(tmp_path: Path) -> None:
    jobs.mark_done(tmp_path, job_key(1, 2), "4 cells, 0 void px filled")
    jobs.mark_done(tmp_path, job_key(3, 4), jobs.OCEAN_STATUS)
    assert jobs.completed_jobs(tmp_path) == {
        job_key(1, 2): "4 cells, 0 void px filled",
        job_key(3, 4): jobs.OCEAN_STATUS,
    }
    assert jobs.completed_jobs(tmp_path / "missing") == {}


def test_parse_job_accepts_only_job_zoom_tiles() -> None:
    z = tilemath.JOB_ZOOM
    assert jobs.parse_job(f"{z}/267/182") == job_key(267, 182)
    for text in ("267/182", f"{z}/a/1", f"{z - 1}/1/1", f"{z}/{1 << z}/0", f"{z}/0/-1"):
        with pytest.raises(ValueError):
            jobs.parse_job(text)


def test_select_jobs_defaults_to_every_completed_job() -> None:
    completed = {job_key(2, 0): "1 cells", job_key(1, 0): jobs.OCEAN_STATUS}
    assert jobs.select_jobs(completed) == [job_key(1, 0), job_key(2, 0)]


def test_select_jobs_explicit_must_be_completed() -> None:
    completed = {job_key(1, 0): "1 cells", job_key(2, 0): "1 cells"}
    assert jobs.select_jobs(completed, explicit=[job_key(2, 0), job_key(2, 0)]) == [job_key(2, 0)]
    with pytest.raises(ValueError, match="Not completed"):
        jobs.select_jobs(completed, explicit=[job_key(9, 9)])


def test_select_jobs_samples_land_jobs_reproducibly() -> None:
    completed = {job_key(x, 0): "2 cells" for x in range(50)}
    completed.update({job_key(x, 1): jobs.OCEAN_STATUS for x in range(50)})
    sample = jobs.select_jobs(completed, sample=10, seed=7)
    assert len(sample) == 10
    assert sample == sorted(sample)
    assert all(completed[job] != jobs.OCEAN_STATUS for job in sample)
    assert jobs.select_jobs(completed, sample=10, seed=7) == sample
    assert jobs.select_jobs(completed, sample=10, seed=8) != sample
    assert len(jobs.select_jobs(completed, sample=500)) == 50
    with pytest.raises(ValueError):
        jobs.select_jobs(completed, sample=0)
