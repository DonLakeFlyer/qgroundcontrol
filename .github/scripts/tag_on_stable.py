#!/usr/bin/env python3
"""Report whether HEAD (a pushed release tag) is reachable from a Stable* branch.

Release tags are only cut on ``Stable_V*`` branches. Tags anywhere else (e.g.
version markers on master) must not trigger the release pipeline or overwrite
the public S3 ``latest/`` downloads. Requires a checkout with full history and
remote branch refs.

Emits ``on_stable=true|false`` to ``GITHUB_OUTPUT``.

Usage:
    python3 .github/scripts/tag_on_stable.py [--context "skipping build"]
"""

from __future__ import annotations

import argparse
import os

from ci_bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common.gh_actions import gh_error, gh_warning, write_github_output
from common.git import run_git

_STABLE_BRANCH_GLOB = "origin/Stable*"


class GitQueryError(RuntimeError):
    """git branch --contains failed; the verdict is unknown, not "false"."""


def stable_branches_containing(ref: str = "HEAD") -> list[str]:
    result = run_git("branch", "-r", "--contains", ref, "--list", _STABLE_BRANCH_GLOB)
    if result.returncode != 0:
        raise GitQueryError(result.stderr.strip() or f"git exited {result.returncode}")
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--context",
        default="",
        help="Appended to the warning when the tag is not on a Stable* branch",
    )
    args = parser.parse_args(argv)

    try:
        branches = stable_branches_containing()
    except GitQueryError as exc:
        gh_error(f"Cannot determine whether tag is on a Stable* branch: {exc}")
        return 1
    if branches:
        print(f"Tag is on Stable branch(es): {', '.join(branches)}")
    else:
        ref_name = os.environ.get("GITHUB_REF_NAME", "HEAD")
        suffix = f"; {args.context}" if args.context else ""
        gh_warning(f"Tag {ref_name} is not on a Stable* branch{suffix}")
    write_github_output({"on_stable": "true" if branches else "false"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
