#!/usr/bin/env python3
"""moccache — a content-addressed cache for Qt's moc.

Speeds up clean builds / CI / branch switches by caching moc output keyed on
moc identity + arguments + input content + all transitive include contents
(tracked via moc's --output-dep-file, Qt 5.15+).

Usage (as CMake AUTOMOC executable, via a shim that bakes the real moc path):
    moccache.py --real-moc /path/to/moc [moc args...]
or set MOCCACHE_MOC=/path/to/moc and invoke:
    moccache.py [moc args...]

Environment:
    MOCCACHE_MOC      Path to the real moc (if --real-moc not given).
    MOCCACHE_DIR      Cache directory (default ~/.cache/moccache).
    MOCCACHE_BASEDIR  Build dir root; rewritten to a token in cache keys and
                      manifests so different build trees share cache entries
                      (same idea as ccache's base_dir).
    MOCCACHE_DISABLE  If set, always pass through to real moc.
    MOCCACHE_STATS    If set, append hit/miss lines to $MOCCACHE_DIR/stats.log.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# Options whose value is a separate following argument.
_VALUE_OPTS = {
    "-o",
    "--include",
    "--dep-file-path",
    "--dep-file-rule-name",
    "-F",
    "-M",
    "--collect-json",
    "-b",
    "-f",
    "-p",
    "-n",
    "--compiler-flavor",
    "-t",
    "-A",
    "--json-output",
}


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _moc_identity(moc: Path) -> str:
    try:
        out = subprocess.run(
            [str(moc), "--version"], capture_output=True, text=True, check=True
        ).stdout.strip()
        if out:
            return out
    except (OSError, subprocess.CalledProcessError):
        pass
    st = moc.stat()
    return f"{st.st_mtime_ns}:{st.st_size}"


def _parse_args(argv: list[str]):
    """Return (output, input_file, dep_file_path, wants_dep_file, wants_json,
    hashable_args, include_files)."""
    output = None
    dep_file_path = None
    wants_dep_file = False
    wants_json = False
    hashable: list[str] = []
    positional: list[str] = []
    include_files: list[str] = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "-o" or a == "--output":
            i += 1
            if i < len(argv):
                output = argv[i]
        elif a == "--output-dep-file":
            wants_dep_file = True
        elif a == "--output-json":
            wants_json = True
            hashable.append(a)
        elif a == "--dep-file-path":
            wants_dep_file = True
            i += 1
            if i < len(argv):
                dep_file_path = argv[i]
        elif a == "--include":
            # Force-included files (e.g. AUTOMOC's moc_predefs.h) live in the
            # build dir, so their *path* varies across build trees. Key on
            # their content instead; the path is excluded from the key.
            i += 1
            if i < len(argv):
                include_files.append(argv[i])
                p = Path(argv[i])
                content_id = _sha256_file(p) if p.is_file() else argv[i]
                hashable.append("--include")
                hashable.append(content_id)
        elif a in _VALUE_OPTS:
            hashable.append(a)
            i += 1
            if i < len(argv):
                hashable.append(argv[i])
        elif a.startswith("-") and a != "-":
            hashable.append(a)
        else:
            positional.append(a)
            hashable.append(a)
        i += 1
    input_file = positional[-1] if positional else None
    return output, input_file, dep_file_path, wants_dep_file, wants_json, hashable, include_files


def _parse_dep_file(text: str) -> list[str]:
    """Parse a Make-style dep file into a list of dependency paths."""
    # Strip line continuations, then split on the first unescaped ':'.
    text = text.replace("\\\n", " ").replace("\\\r\n", " ")
    colon = -1
    i = 0
    while i < len(text):
        if text[i] == ":" and (i + 1 >= len(text) or text[i + 1] in " \t\n\r"):
            colon = i
            break
        i += 1
    deps_part = text[colon + 1 :] if colon >= 0 else text
    # Unescape "\ " (escaped spaces in paths) by tokenizing manually.
    deps: list[str] = []
    cur: list[str] = []
    j = 0
    while j < len(deps_part):
        c = deps_part[j]
        if c == "\\" and j + 1 < len(deps_part) and deps_part[j + 1] == " ":
            cur.append(" ")
            j += 2
            continue
        if c in " \t\n\r":
            if cur:
                deps.append("".join(cur))
                cur = []
        else:
            cur.append(c)
        j += 1
    if cur:
        deps.append("".join(cur))
    return deps


def _atomic_copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(dst.parent), prefix=".moccache-")
    os.close(fd)
    try:
        shutil.copyfile(src, tmp)
        os.replace(tmp, dst)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _atomic_write(dst: Path, data: str) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(dst.parent), prefix=".moccache-")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(data)
        os.replace(tmp, dst)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _make_escape(path: str) -> str:
    return path.replace(" ", "\\ ")


_BASEDIR_TOKEN = "<<MOCCACHE_BASEDIR>>"


def _basedir_prefixes() -> list[str]:
    """Build-dir spellings to rewrite (raw and resolved), longest first."""
    raw = os.environ.get("MOCCACHE_BASEDIR", "").rstrip("/")
    if not raw:
        return []
    prefixes = {raw}
    try:
        prefixes.add(os.path.realpath(raw))
    except OSError:
        pass
    return sorted(prefixes, key=len, reverse=True)


def _normalize_basedir(s: str, prefixes: list[str]) -> str:
    for p in prefixes:
        s = s.replace(p, _BASEDIR_TOKEN)
    return s


def _denormalize_basedir(s: str) -> str:
    raw = os.environ.get("MOCCACHE_BASEDIR", "").rstrip("/")
    return s.replace(_BASEDIR_TOKEN, raw) if raw else s


def _write_dep_file(dst: Path, target: str, deps: list[str]) -> None:
    body = " \\\n  ".join(_make_escape(d) for d in deps)
    _atomic_write(dst, f"{_make_escape(target)}: \\\n  {body}\n")


def _log_stat(cache_dir: Path, what: str, input_file: str) -> None:
    if not os.environ.get("MOCCACHE_STATS"):
        return
    try:
        with (cache_dir / "stats.log").open("a") as f:
            f.write(f"{what}\t{input_file}\n")
    except OSError:
        pass


def main() -> int:
    argv = sys.argv[1:]
    real_moc = None
    if argv and argv[0] == "--real-moc":
        if len(argv) < 2:
            print("moccache: --real-moc requires a path", file=sys.stderr)
            return 2
        real_moc = Path(argv[1])
        argv = argv[2:]
    elif os.environ.get("MOCCACHE_MOC"):
        real_moc = Path(os.environ["MOCCACHE_MOC"])
    if real_moc is None or not real_moc.is_file():
        print("moccache: real moc not found (set MOCCACHE_MOC or use --real-moc)", file=sys.stderr)
        return 2

    def passthrough() -> int:
        return subprocess.run([str(real_moc), *argv]).returncode

    if os.environ.get("MOCCACHE_DISABLE"):
        return passthrough()

    output, input_file, dep_file_path, wants_dep_file, wants_json, hashable, include_files = (
        _parse_args(argv)
    )
    if not output or not input_file or not Path(input_file).is_file():
        return passthrough()

    cache_dir = Path(os.environ.get("MOCCACHE_DIR", str(Path.home() / ".cache" / "moccache")))
    basedir_prefixes = _basedir_prefixes()

    # Manifest key: moc identity + args (minus output/dep paths) + input content.
    # Build-dir paths in the args are rewritten to a token so different build
    # trees produce the same key.
    key_parts = [
        _moc_identity(real_moc),
        *(_normalize_basedir(a, basedir_prefixes) for a in hashable),
        _sha256_file(Path(input_file)),
    ]
    h = hashlib.sha256()
    for part in key_parts:
        h.update(part.encode())
        h.update(b"\x00")
    mkey = h.hexdigest()
    mdir = cache_dir / mkey[:2] / mkey

    manifest = mdir / "manifest"
    cached_out = mdir / "output.cpp"
    cached_json = mdir / "output.json"
    out_path = Path(output)
    dep_out = Path(dep_file_path) if dep_file_path else Path(str(out_path) + ".d")
    json_out = Path(str(out_path) + ".json")

    # --- Try for a hit ------------------------------------------------------
    if manifest.is_file() and cached_out.is_file() and (not wants_json or cached_json.is_file()):
        hit = True
        manifest_deps: list[str] = []
        try:
            for line in manifest.read_text().splitlines():
                dep, _, dep_hash = line.partition("\t")
                dep = _denormalize_basedir(dep)
                p = Path(dep)
                if not p.is_file() or _sha256_file(p) != dep_hash:
                    hit = False
                    break
                manifest_deps.append(dep)
        except OSError:
            hit = False
        if hit:
            try:
                out_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(cached_out, out_path)
                if wants_dep_file:
                    # Synthesize for this tree: the cached dep file references
                    # the originating tree's output and moc_predefs.h paths.
                    deps = list(dict.fromkeys([input_file, *include_files, *manifest_deps]))
                    _write_dep_file(dep_out, output, deps)
                if wants_json:
                    shutil.copyfile(cached_json, json_out)
                _log_stat(cache_dir, "hit", input_file)
                return 0
            except OSError:
                pass  # fall through to a real run

    # --- Miss: run real moc with dep-file capture ---------------------------
    tmp_dep = None
    cmd = [str(real_moc), *argv]
    if not wants_dep_file:
        fd, tmp_dep = tempfile.mkstemp(suffix=".d", prefix="moccache-")
        os.close(fd)
        cmd += ["--output-dep-file", "--dep-file-path", tmp_dep]
    try:
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            return rc

        dep_src = Path(tmp_dep) if tmp_dep else dep_out
        try:
            dep_text = dep_src.read_text()
        except OSError:
            _log_stat(cache_dir, "miss-nodep", input_file)
            return 0  # moc succeeded; just can't cache

        # Force-included files are keyed by content, so their (build-dir
        # specific) paths must stay out of the manifest. Build-dir dep paths
        # are stored token-relative so other trees can validate them.
        include_reals = {os.path.realpath(f) for f in include_files}
        lines = []
        for dep in _parse_dep_file(dep_text):
            p = Path(dep)
            if p.is_file() and os.path.realpath(dep) not in include_reals:
                lines.append(f"{_normalize_basedir(dep, basedir_prefixes)}\t{_sha256_file(p)}")
        try:
            _atomic_copy(out_path, cached_out)
            if wants_json and json_out.is_file():
                _atomic_copy(json_out, cached_json)
            _atomic_write(manifest, "\n".join(lines) + "\n")
            _atomic_write(mdir / "keyinfo", "\n".join(key_parts) + "\n")
        except OSError:
            pass  # cache write failure must not fail the build
        _log_stat(cache_dir, "miss", input_file)
        return 0
    finally:
        if tmp_dep:
            try:
                os.unlink(tmp_dep)
            except OSError:
                pass


if __name__ == "__main__":
    sys.exit(main())
