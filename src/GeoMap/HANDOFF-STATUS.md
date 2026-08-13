# HANDOFF STATUS — continuous-drape-heightfield

Machine handoff notes (laptop → desktop). Read together with
`DESIGN-cesium-style-sampling.md` and `DEVELOPMENT-PROCESS.md` in this
directory. Delete all working .md files in this directory before the final PR
(this file, the design docs, `DEVELOPMENT-PROCESS.md`, `CESIUM-ANALYSIS.md`,
`CESIUM-BACKEND-EVALUATION.md`, `THREADING-ANALYSIS.md`).

## Where things stand

Branch `continuous-drape-heightfield`.
HEAD = de00cf4986 "feat(GeoMap): cut samplePatch over to cesium-style
single-tile sampling" (Step 1, **not pushed**).
**Step 2 is complete, reviewed, and green but UNCOMMITTED** — 15 modified
files in the working tree (~483 insertions). Commit/push only on explicit
instruction.

### Step 1 — committed (de00cf4986)

- `HeightField::samplePatch` cutover to cesium-style single-tile sampling:
  `bestTileFor(key)` + subwindow UV sampling; dyadic vertex positions.
- Pinning in ElevationTilePyramid (replaces feathering, fully reverted);
  SurfaceModel maintains the pinned set (resident keys + ancestor chains).
- Review hardening: `_stitchAtWorldEdgeFallsBack`,
  `_stitchedNorthEdgeUsesAncestorSouthRow` (mutation-tested), corner
  precedence documented in `_heightAt`.

### Step 2 — done, reviewed, uncommitted

- `HeightField::backingKeyFor()` (inline in .h; returns `{0,0,-1}` when the
  view is invalid) + `HeightFieldTest::_backingKeyFor`.
- Retired the 0.5 m contract: `_renderedEdgesContinuousAcrossLodBoundaries`
  → `_renderedEdgeContractAcrossLodBoundaries` (three-part contract per
  design §5: (a) same-backing exact 0.0, (T) ≤1e-3, (b) source-derived
  bound). TEMP DIAGNOSTIC removed. SurfaceModelTest 25/25.
- **Renderer wiring** (gap found during Step 2, user chose to wire now):
  PatchGeometry got declarative Q_PROPERTYs `heightField`/`tileX`/`tileY`/
  `tileZoom` (shared notify `tileKeyChanged`); `setHeightField` connects
  `regionChanged` → `_fieldRegionChanged` (early-outs: all deltas 0, invalid
  key, no inflated-rect intersect) and `destroyed` → null-and-rebuild.
  SurfacePatchModel exposes `TileXRole`/`TileYRole` + `heightField`
  Q_PROPERTY; GeoMap.qml delegate binds all four. FlyViewGeoUITest 7/7
  (strict log mode) validates the real QML path.
- Review-round fixes (all verified):
  - destroyed-handler in `setHeightField` nulls, emits, rebuilds.
  - HeightField.h: comment moved above `QML_ANONYMOUS` — `.clang-format`
    lacks `StatementMacros`, so trailing comments on QML macros get mangled;
    repo-wide StatementMacros fix deferred to a **separate PR**.
  - SurfacePatchModel: comment documenting the unconditional
    `heightFieldChanged` emit (pointer ABA hazard).
  - Removed unused `#include <array>` from SurfaceModelTest.cc.
  - **Rebuild coalescing in PatchGeometry**: setters/`_fieldRegionChanged`/
    destroyed-lambda call `_requestRebuild()` (gated on
    `isComponentComplete()`); `componentComplete()` override does base + one
    `_rebuild()`. QML instantiation: 8 rebuilds → 1. C++-constructed objects
    are complete from birth, so tests and `sampleFromField` (direct
    `_rebuild()`) keep synchronous semantics.
- Final pass done: 11/11 GeoMap ctest suites green; pre-commit clean on all
  modified files (clang-tidy fails only where not installed locally).

### Coverage (fresh counters, 11 suites + FlyViewGeoUITest)

- HeightField.cc/.h 100%; PatchGeometry.cc 97%; SurfacePatchModel.cc 65%
  (gaps are pre-existing stats/capture/imagery code, untouched).
- Known uncovered **new** branches, user deferred adding tests ("not right
  now") — candidate follow-up:
  1. `setHeightField` replacing one live field with another (disconnect of
     the old field).
  2. `_fieldRegionChanged` invalid-key early-out.
  3. `_coarseEdgeSamples` empty `ancestorHeights` return.
  4. Same-value early-return guards in the new setters.
- Coverage gotcha: a rebuild orphans old `.gcda` (stale-counter 1% readings);
  `find build-coverage -name "*.gcda" -delete`, rerun suites, then gcovr:
  `gcovr --root . build-coverage --filter 'src/GeoMap/...' --txt -`.

## Next steps (in order)

1. **Commit Step 2 first** (when instructed) — the working tree is the whole
   step.
2. Step 3: skirt sizing by coarse-level geometric error (design §3):
   `TileMath::levelGeometricError`, skirt depth = 5 × geometric error of the
   coarsest constraining level, replacing `kSkirtDepthFraction = 0.05`.
   TDD: contract (c) red test first (uses `backingKeyFor`).
3. Step 4 / wrap-up: full regression (11 suites + FlyViewGeoUITest),
   `just build`, `just lint`, clang-format touched files.
4. Deferred: kMaxTiles=128 capacity analysis (user explicitly wants this
   revisited after the cutover); optional coverage tests above; separate PR
   for `.clang-format` StatementMacros.
## Build/test loop on this branch

- Check for a running build first: `pgrep -fl "ninja|cmake --build"` (user
  often builds via VS Code).
- Build: `cmake --build build-coverage --target QGroundControl`
- One suite:
  `./build-coverage/Debug/QGroundControl.app/Contents/MacOS/QGroundControl --unittest:<Name> --allow-multiple 2>&1 | grep -vE '^profiling' | grep -E "FAIL|Totals"`
- Full regression: 11 GeoMap suites
  `ctest --test-dir build-coverage -R "SurfaceModel|SurfacePatchModel|SurfaceAnalysis|SurfacePatchImagery|TerrariumTileFetcher|HeightSource|PatchGeometry|HeightField|ElevationTilePyramid|TileMath|GeoMapCamera|GeoMapProj"`
  — plus FlyViewGeoUITest separately (not matched by that regex; it is the
  only suite exercising the QML delegate path, incl. `componentComplete`).
- Strict log mode: unexpected qCWarnings fail tests — use
  `expectLogMessage`/`ignoreLogMessage`.
- clang-format: `CF=$(ls ~/.cache/pre-commit/*/py_env-*/bin/clang-format | head -1); "$CF" -i <files>`
- pre-commit needs `source .venv/bin/activate`. `just` was not installed on
  the laptop — `pre-commit run --files <changed>` is the equivalent lint gate.

## Working agreements

- TDD red-first for behavioral changes (skip ceremony tests).
- No commit/push without explicit instruction.
- Cesium-native source for reference: `~/repos/cesium-native`. Key facts:
  upsample-by-parent-clip at UV 0.5; skirts = 5 × levelMaximumGeometricError
  (only crack-hiding mechanism, no feathering anywhere); render list holds
  intrusive refs so eviction of visible tiles is impossible by construction;
  512 MB soft cache cap.
