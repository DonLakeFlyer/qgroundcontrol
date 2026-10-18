"""Upload generated tiles to the private qgc-terrain-assets bucket (served by CloudFront)."""

from __future__ import annotations

import re
import subprocess
from typing import TYPE_CHECKING

from .metadata import write_metadata

if TYPE_CHECKING:
    from pathlib import Path

BUCKET = "qgc-terrain-assets"
PUBLIC_URL = "https://terrain.assets.qgroundcontrol.com"
IMMUTABLE_CACHE_CONTROL = "public, max-age=31536000, immutable"
SCRATCH_CACHE_CONTROL = "public, max-age=300"
_RELEASE_PREFIX = re.compile(r"v\d+")
_PREFIX = re.compile(r"[a-z0-9][a-z0-9._-]*")


def is_release(prefix: str) -> bool:
    """Release prefixes (v1, v2, ...) are immutable; anything else is scratch."""
    return bool(_RELEASE_PREFIX.fullmatch(prefix))


def cache_control_for(prefix: str) -> str:
    return IMMUTABLE_CACHE_CONTROL if is_release(prefix) else SCRATCH_CACHE_CONTROL


def _validate_prefix(prefix: str) -> None:
    if not _PREFIX.fullmatch(prefix):
        raise ValueError(f"Invalid prefix {prefix!r}: use a single lowercase path segment")


def _aws_options(profile: str | None, dryrun: bool) -> list[str]:
    options = ["--no-progress"]
    if profile:
        options += ["--profile", profile]
    if dryrun:
        options.append("--dryrun")
    return options


def sync_command(out: Path, prefix: str, *, profile: str | None, dryrun: bool) -> list[str]:
    """Tile sync; scratch prefixes mirror *out* exactly, release prefixes never delete."""
    _validate_prefix(prefix)
    # The bucket blocks ACLs, so no --acl; CloudFront reads it through Origin Access Control.
    command = [
        "aws",
        "s3",
        "sync",
        str(out / "terrarium"),
        f"s3://{BUCKET}/{prefix}/terrarium",
        "--exclude",
        "*",
        "--include",
        "*.png",
        "--content-type",
        "image/png",
        "--cache-control",
        cache_control_for(prefix),
    ]
    if not is_release(prefix):
        command.append("--delete")
    return command + _aws_options(profile, dryrun)


def metadata_commands(
    paths: list[Path], prefix: str, *, profile: str | None, dryrun: bool
) -> list[list[str]]:
    """Metadata always gets short caching: it may be corrected without a new prefix."""
    _validate_prefix(prefix)
    content_types = {".json": "application/json", ".txt": "text/plain; charset=utf-8"}
    return [
        [
            "aws",
            "s3",
            "cp",
            str(path),
            f"s3://{BUCKET}/{prefix}/{path.name}",
            "--content-type",
            content_types[path.suffix],
            "--cache-control",
            SCRATCH_CACHE_CONTROL,
            *_aws_options(profile, dryrun),
        ]
        for path in paths
    ]


def upload(out: Path, prefix: str, *, profile: str | None, dryrun: bool) -> int:
    if not (out / "terrarium").is_dir():
        raise ValueError(f"No tiles found under {out / 'terrarium'}")
    _validate_prefix(prefix)
    metadata = write_metadata(out, PUBLIC_URL, prefix)
    commands = [sync_command(out, prefix, profile=profile, dryrun=dryrun)]
    commands += metadata_commands(metadata, prefix, profile=profile, dryrun=dryrun)
    for command in commands:
        print(" ".join(command), flush=True)
        returncode = subprocess.run(command, check=False).returncode
        if returncode:
            return returncode
    return 0
