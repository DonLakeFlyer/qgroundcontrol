"""Tests for tag_on_stable.py."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
import tag_on_stable
from _helpers import completed

if TYPE_CHECKING:
    from pathlib import Path


def _stub_git(monkeypatch: pytest.MonkeyPatch, stdout: str, returncode: int = 0) -> None:
    monkeypatch.setattr(tag_on_stable, "run_git", lambda *a, **k: completed(stdout, returncode))


def test_lists_containing_stable_branches(monkeypatch: pytest.MonkeyPatch) -> None:
    _stub_git(monkeypatch, "  origin/Stable_V5.1\n\n  origin/Stable_V5.0\n")
    assert tag_on_stable.stable_branches_containing() == [
        "origin/Stable_V5.1",
        "origin/Stable_V5.0",
    ]


def test_empty_when_not_on_stable(monkeypatch: pytest.MonkeyPatch) -> None:
    _stub_git(monkeypatch, "")
    assert tag_on_stable.stable_branches_containing() == []


def test_raises_when_git_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    _stub_git(monkeypatch, "", returncode=128)
    with pytest.raises(tag_on_stable.GitQueryError):
        tag_on_stable.stable_branches_containing()


def test_main_fails_when_git_fails(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _stub_git(monkeypatch, "", returncode=128)
    output = tmp_path / "output"
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))
    assert tag_on_stable.main([]) == 1
    assert not output.exists()


@pytest.mark.parametrize(("stdout", "expected"), [("origin/Stable_V5.1\n", "true"), ("", "false")])
def test_main_writes_on_stable_output(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stdout: str, expected: str
) -> None:
    _stub_git(monkeypatch, stdout)
    output = tmp_path / "output"
    monkeypatch.setenv("GITHUB_OUTPUT", str(output))
    assert tag_on_stable.main([]) == 0
    assert f"on_stable={expected}" in output.read_text()
