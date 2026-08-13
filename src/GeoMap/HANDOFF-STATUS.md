# HANDOFF STATUS — continuous-drape-heightfield

Machine handoff notes (desktop → laptop). Read together with
`DESIGN-cesium-style-sampling.md` in this directory. Delete both files before
the final PR.

## Where things stand

Branch `continuous-drape-heightfield`, base commit e4f28a7ac ("Step 6").
Everything below is in the working tree (committed as WIP when this was
pushed).

### Done and green

- **Pinning in ElevationTilePyramid** (cesium-style, replaces feathering):
  - `setPinnedKeys(QSet<TileKey>)`; `_evictLeastRecentlyUsed()` skips pinned
    keys and returns invalid key `{0,0,-1}` when every resident tile is
    pinned (insert then grows past `kMaxTiles` — soft cap).
  - `insertTile(key, grid, TileKey* evictedKey)` out-param only written for
    real evictions.
  - Tests: `_pinnedTilesSurviveEviction`, `_allPinnedGrowsPastCap` —
    ElevationTilePyramidTest 15/15 green.
- **SurfaceModel maintains the pinned set**: after each update pass with
  adds/removals, pins every resident patch key + full ancestor chain via
  `_field->setPinnedKeys(...)`. Test `_pinsPatchBackingTiles` green.
- **Feathering fully reverted** in HeightField (blend band, kFeatherFraction,
  shrunk memo bounds, inflated regionChanged rects all gone). Kept: evicted
  region `regionChanged` emission. HeightFieldTest 18/18 green
  (`_heightAtContinuousAcrossResidencyBorders` replaced by
  `_pinnedTilesSurviveInsertPressure`).
- **Dyadic vertex positions in samplePatch** (global integer fractions) —
  fixed cross-zoom ulp seams. NOTE: superseded by the design cutover (the
  whole pointwise loop goes away), but harmless meanwhile.
- **Memo key-arithmetic hit test** in `heightAt` — fixed warm-memo vs cold
  lookup resolving different tiles at exact tile boundaries. Applied but the
  post-fix build/test run was cancelled; not yet verified. Becomes
  non-critical after the cutover.
- Logging upgrade (TerrariumTileFetcher/TileImageSource throttled warnings) —
  green, from earlier steps.

### Red / in flux

- `SurfaceModelTest::_renderedEdgesContinuousAcrossLodBoundaries` — the 0.5 m
  C0 repro test. Last failure: 176.3 m at z11 (1071,714) E vs (1072,714),
  tilt 80 dist 3000 — root-caused to the memo boundary bug above. Per the
  design, this test's contract is being **replaced**, not fixed (see design
  §5). It still contains a TEMP DIAGNOSTIC block (labeled `// TEMP
  DIAGNOSTIC`) that must be removed.

## Next steps (in order)

1. Implement the design: rewrite `HeightField::samplePatch` to
   `bestTileFor(K)` + subwindow UV sampling (design §1). TDD: write the
   same-backing-tile bit-exact edge test first, verify red, then implement.
2. Retire the 0.5 m contract: replace
   `_renderedEdgesContinuousAcrossLodBoundaries` with the three-part contract
   (design §5); remove the TEMP DIAGNOSTIC block.
3. Skirt sizing by coarse-level geometric error (design §3) + test.
4. Full regression: 11 GeoMap suites
   `ctest --test-dir build-coverage -R "SurfaceModel|SurfacePatchModel|SurfaceAnalysis|SurfacePatchImagery|TerrariumTileFetcher|HeightSource|PatchGeometry|HeightField|ElevationTilePyramid|TileMath|GeoMapCamera|GeoMapProj"`,
   `just build`, `just lint`, clang-format touched files.
5. Deferred: kMaxTiles capacity analysis (user explicitly wants this
   revisited after the cutover).

## Build/test loop on this branch

- Check for a running build first: `pgrep -fl "ninja|cmake --build"` (user
  often builds via VS Code).
- Build: `cmake --build build-coverage --target QGroundControl`
- One suite:
  `./build-coverage/Debug/QGroundControl.app/Contents/MacOS/QGroundControl --unittest:<Name> --allow-multiple 2>&1 | grep -vE '^profiling' | grep -E "FAIL|Totals"`
- Strict log mode: unexpected qCWarnings fail tests — use
  `expectLogMessage`/`ignoreLogMessage`.
- clang-format: `CF=$(ls ~/.cache/pre-commit/*/py_env-*/bin/clang-format | head -1); "$CF" -i <files>`
- pre-commit needs `source .venv/bin/activate`.

## Working agreements

- TDD red-first for behavioral changes (skip ceremony tests).
- No commit/push without explicit instruction.
- Cesium-native source for reference: `~/repos/cesium-native`. Key facts:
  upsample-by-parent-clip at UV 0.5; skirts = 5 × levelMaximumGeometricError
  (only crack-hiding mechanism, no feathering anywhere); render list holds
  intrusive refs so eviction of visible tiles is impossible by construction;
  512 MB soft cache cap.
