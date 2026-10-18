"""Command-line interface for terrain tile generation, validation and upload."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from .copernicus import load_cells
from .jobs import completed_jobs, parse_job, select_jobs
from .tilemath import JOB_ZOOM, MAX_ZOOM, TileKey, all_tiles, tiles_in_bbox

# Small validation areas: high relief, coastlines, flat plains, each hemisphere, every
# Copernicus longitude-spacing band, a band boundary (60N), and GLO-90 gap filling.
SAMPLE_AREAS: dict[str, tuple[float, float, float, float]] = {
    "alps": (7.6, 45.95, 7.7, 46.0),
    "everest": (86.9, 27.95, 86.95, 28.0),
    "aconcagua": (-70.05, -32.7, -70.0, -32.6),
    "sf-coast": (-122.55, 37.7, -122.4, 37.8),
    "kansas": (-98.5, 38.5, -98.4, 38.6),
    "jotunheimen": (8.25, 61.6, 8.35, 61.65),
    "oslo-60n": (10.7, 59.97, 10.8, 60.03),
    "svalbard": (15.55, 78.2, 15.7, 78.25),
    "aragats": (44.15, 40.5, 44.25, 40.55),
    "armenia-40n": (43.8, 39.97, 43.9, 40.03),
    "caucasus-42n": (44.55, 41.97, 44.65, 42.03),
}


def _source_cache(out: Path) -> Path:
    return out / "_source"


def _selected_jobs(args: argparse.Namespace) -> list[TileKey]:
    if args.all:
        return all_tiles(JOB_ZOOM)
    names = sorted(SAMPLE_AREAS) if "all" in (args.area or []) else args.area or []
    boxes = [SAMPLE_AREAS[name] for name in names] + [tuple(box) for box in args.bbox or []]
    if not boxes:
        raise SystemExit("Select tiles with --area, --bbox or --all")
    return sorted({job for box in boxes for job in tiles_in_bbox(*box, JOB_ZOOM)})


def _cmd_generate(args: argparse.Namespace) -> int:
    from .generate import generate

    jobs = _selected_jobs(args)
    cells = load_cells(_source_cache(args.out))
    print(f"{len(jobs)} z{JOB_ZOOM} jobs, {args.workers} workers -> {args.out}", flush=True)
    failures = generate(args.out, jobs, cells, args.workers, args.force)
    if failures:
        print(f"{failures} job(s) failed; rerun to retry them")
    return 1 if failures else 0


def _cmd_overviews(args: argparse.Namespace) -> int:
    from .generate import build_overviews

    build_overviews(args.out)
    return 0


def _cmd_validate(args: argparse.Namespace) -> int:
    import numpy as np

    from . import validate
    from .ardupilot import HgtCells

    completed = completed_jobs(args.out)
    if not completed:
        raise SystemExit(f"No completed jobs under {args.out}")
    try:
        explicit = [parse_job(text) for text in args.job or []]
        jobs = select_jobs(completed, explicit=explicit, sample=args.sample_jobs, seed=args.seed)
    except ValueError as error:
        raise SystemExit(str(error)) from None
    cells = load_cells(_source_cache(args.out))
    hgt = HgtCells(args.out / "_reference" / "ardupilot")
    rng = np.random.default_rng(args.seed)

    # One decoded job (~64 MB plus check temporaries) at a time; only result rows are kept.
    sources: list[validate.SourceCheck] = []
    references: list[tuple[TileKey, validate.ReferenceCheck | None]] = []
    alignments: list[validate.AlignmentCheck] = []
    for index, job in enumerate(jobs, start=1):
        print(f"[{index}/{len(jobs)}] job {job}", file=sys.stderr, flush=True)
        ours = validate.read_job_heights(args.out, job)
        sources.append(validate.check_source(job, ours, cells, args.samples, rng))
        references.append((job, validate.check_ardupilot(job, ours, hgt)))
        alignments.extend(validate.check_tilezen(job, ours, args.alignment_tiles))
        del ours

    problems: list[str] = []
    print(f"Decoded z{MAX_ZOOM} pixels vs point-bilinear Copernicus source (meters)")
    print(f"{'job':<12} {'n':>5} {'mean':>7} {'p95':>7} {'max':>8} {'border n':>9} {'border':>7}")
    for result in sources:
        print(
            f"{result.job!s:<12} {result.samples:>5} {result.mean_abs:>7.2f} "
            f"{result.p95_abs:>7.2f} {result.max_abs:>8.2f} {result.border_samples:>9} "
            f"{result.border_mean_abs:>7.2f}"
        )
        if result.max_abs > validate.MAX_SOURCE_ERROR:
            problems.append(f"{result.job}: {result.max_abs:.1f} m from the source")

    print("\nArduPilot AW3D30 (terrain.ardupilot.org SRTM1) on land; diff = ours - ArduPilot")
    print(
        f"{'job':<12} {'land px':>8} {'median':>7} {'med|d|':>7} {'p99|d|':>7} "
        f"{'as sea':>7} {'low>50m*':>8} {'east m':>7} {'north m':>8}"
    )
    for job, ref in references:
        if ref is None:
            print(f"{job!s:<12} no ArduPilot land")
            continue
        print(
            f"{job!s:<12} {ref.land_pixels:>8} {ref.median_diff:>7.2f} {ref.median_abs:>7.2f} "
            f"{ref.p99_abs:>7.1f} {ref.land_as_sea:>7} {ref.grossly_low:>8} "
            f"{ref.offset_east_m:>7.1f} {ref.offset_north_m:>8.1f}"
        )
        if math.hypot(ref.offset_east_m, ref.offset_north_m) > validate.MAX_REFERENCE_OFFSET_M:
            problems.append(
                f"{job}: ArduPilot offset ({ref.offset_east_m:.1f}, {ref.offset_north_m:.1f}) m"
            )
        if ref.land_as_sea > validate.MAX_LAND_AS_SEA_FRACTION * ref.land_pixels:
            problems.append(f"{job}: {ref.land_as_sea} land as sea pixels of {ref.land_pixels}")
        if ref.grossly_low > validate.MAX_GROSSLY_LOW_FRACTION * ref.gentle_pixels:
            problems.append(
                f"{job}: {ref.grossly_low} grossly low pixels of {ref.gentle_pixels} on gentle slopes"
            )
    print(f"* on land with slope under {validate.GROSSLY_LOW_MAX_SLOPE}")

    print(f"\nTilezen offset relative to ours on rugged z{MAX_ZOOM} tiles (pixels; informational)")
    print(f"{'tile':<16} {'dx':>6} {'dy':>6} {'bias m':>7}")
    for result in alignments:
        print(
            f"{result.key!s:<16} {result.offset_x:>6.2f} {result.offset_y:>6.2f} "
            f"{result.bias:>7.1f}"
        )

    print(f"\nValidated {len(jobs)} of {len(completed)} completed jobs")
    print("\n".join(problems) if problems else "No problems found")
    return 1 if problems else 0


def _cmd_stats(args: argparse.Namespace) -> int:
    from .validate import size_report

    rows = size_report(args.out)
    if not rows:
        raise SystemExit(f"No tiles under {args.out}")
    print(f"{'zoom':>4} {'tiles':>8} {'sea-sized':>9} {'mean B':>9} {'p95 B':>9} {'total MB':>9}")
    for row in rows:
        print(
            f"{row.zoom:>4} {row.count:>8} {row.sea_count:>9} {row.mean_bytes:>9.0f} "
            f"{row.p95_bytes:>9.0f} {row.total_bytes / 1e6:>9.2f}"
        )
    return 0


def _cmd_upload(args: argparse.Namespace) -> int:
    from .upload import upload

    return upload(args.out, args.prefix, profile=args.profile, dryrun=args.dryrun)


def build_parser() -> argparse.ArgumentParser:
    from .generate import default_workers

    parser = argparse.ArgumentParser(prog="terrain_tiles.py", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    def add_command(name: str, help_text: str, handler) -> argparse.ArgumentParser:
        command = commands.add_parser(name, help=help_text)
        command.add_argument("--out", type=Path, required=True, help="Local tile output directory")
        command.set_defaults(handler=handler)
        return command

    generate = add_command(
        "generate", f"Generate z{MAX_ZOOM}..z{JOB_ZOOM} tiles for z{JOB_ZOOM} jobs", _cmd_generate
    )
    generate.add_argument("--area", action="append", choices=[*sorted(SAMPLE_AREAS), "all"])
    generate.add_argument(
        "--bbox", action="append", nargs=4, type=float, metavar=("W", "S", "E", "N")
    )
    generate.add_argument("--all", action="store_true", help=f"Every z{JOB_ZOOM} job (full globe)")
    generate.add_argument("--workers", type=int, default=default_workers())
    generate.add_argument("--force", action="store_true", help="Regenerate completed jobs")

    add_command(
        "overviews", f"Build z{JOB_ZOOM - 1}..z0 from existing z{JOB_ZOOM} tiles", _cmd_overviews
    )

    validate = add_command(
        "validate", "Check completed jobs against source and Tilezen", _cmd_validate
    )
    selection = validate.add_mutually_exclusive_group()
    selection.add_argument(
        "--job", action="append", metavar="Z/X/Y", help="Validate this completed job (repeatable)"
    )
    selection.add_argument(
        "--sample-jobs",
        type=int,
        metavar="N",
        help="Validate N random completed land jobs (seeded by --seed); default is every job",
    )
    validate.add_argument("--samples", type=int, default=400, help="Source samples per job")
    validate.add_argument("--alignment-tiles", type=int, default=3, help="Tilezen tiles per job")
    validate.add_argument("--seed", type=int, default=0)

    add_command("stats", "Report tile counts and sizes per zoom", _cmd_stats)

    upload = add_command(
        "upload", "Upload tiles, tilejson.json and ATTRIBUTION.txt to <prefix>/", _cmd_upload
    )
    upload.add_argument(
        "--prefix", required=True, help="v<N> for releases (immutable); anything else is scratch"
    )
    upload.add_argument("--profile", help="AWS CLI profile")
    upload.add_argument("--dryrun", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.handler(args)
