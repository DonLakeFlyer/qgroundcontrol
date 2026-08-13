# Design: Cesium-Style Patch Sampling (GeoMap continuous drape)

Issue: #14823 — GeoMap: continuous drape terrain design.
Branch: `continuous-drape-heightfield`.

## Principle

**A patch's mesh comes from exactly one tile's grid, chosen by integer key
arithmetic. Doubles only interpolate *within* a grid — they never decide
*which data* answers.**

This is how cesium-native works (reference source in `~/repos/cesium-native`):
a tile renders its own data or an exactly-addressed ancestor window
(`upsampleGltfForRasterOverlays` clips the parent mesh at UV 0.5), never a
per-vertex "finest resident cover at this world point" lookup.

## Why (post-mortem of the previous design)

The previous design defined the rendered surface as pointwise sampling of a
continuous world-coordinate function `HeightField::heightAt(QPointF)`. Edge
continuity between adjacent patches then required **bit-identical doubles**
producing **bit-identical tile resolution** at shared vertices. That contract
was smeared across:

- `samplePatch` vertex world-coordinate arithmetic (differed per zoom level
  by an ulp → 176 m rendered walls),
- `TileMath::tileForWorld`'s half-open floor assignment,
- the `heightAt` memo cache hit test (world-bounds check disagreed with
  `tileForWorld` by an ulp exactly on tile boundaries → warm-memo vs cold
  lookup resolved *different tiles* for the same coordinate),
- the continuity test's bit-exact stitch replica.

Every fix was "make one more code path ulp-consistent". The class of bugs is
unrepresentable in index-space sampling, which is why we're cutting over.

Separately, the original field-report cliffs (100+ m walls) were caused by
silent LRU eviction of tiles backing rendered patches. That is fixed by
**pinning** (already implemented, keep it — see below).

## The design

1. **`HeightField::samplePatch(K, G)` rewrite** — resolve the backing view
   once per patch: `view = _pyramid.bestTileFor(K)` (existing integer
   ancestor walk, exact). Sample `view.grid` over `view.subwindow` at dyadic
   UV offsets: `u = sub.x() + sub.width() * col / G`,
   `v = sub.y() + sub.height() * row / G` (subwindow coords are dyadic
   rationals built from shifts — exact in double). No `heightAt` calls, no
   world coordinates, no memo, no half-open convention in the render path.

2. **`heightAt(QPointF)` demoted** — kept only for genuine point queries
   (camera ground height, SurfaceAnalysis, picking) where an ulp is harmless.
   The memo becomes a pure performance detail with no correctness coupling to
   rendering. The in-flight key-arithmetic memo fix (already applied in
   HeightField.cc) is harmless; keep or simplify.

3. **Edge continuity is layered, like cesium's:**
   - Neighbors resolving to the **same backing tile**: bit-exact by dyadic
     subwindow arithmetic.
   - **Fine patch vs coarse patch** at an LOD boundary: existing
     `PatchGeometry` edge-lerp stitch constrains the fine edge.
   - **Different backing tiles at the same level**: terrarium grids are
     sample-center (no shared edge row/col), so there is an inherent ≤
     one-texel mismatch. Cesium does not chase this — **skirts** hide it.
     Size skirts by the backing level's geometric error (cesium:
     `5 × levelMaximumGeometricError`, see
     `QuantizedMeshLoader.cpp::calculateSkirtHeight`) instead of the current
     fixed `kSkirtDepthFraction = 0.05` of patch span.

4. **Unchanged / keep:**
   - **Pinning** (`ElevationTilePyramid::setPinnedKeys`): SurfaceModel pins
     every resident patch key + full ancestor chain after each update pass.
     Exactly matches what `bestTileFor(K)` can return, so a rendered patch's
     backing tile can never be evicted. `kMaxTiles` is a soft cap: when all
     resident tiles are pinned, inserts grow past it.
   - Eviction `regionChanged` notification (safety net: any unpinned
     eviction announces its region so touching patches re-mesh).
   - SurfaceModel LOD selection, culling, add/remove caps.

5. **Test contract changes** — the 0.5 m C0-exactness repro test
   (`_renderedEdgesContinuousAcrossLodBoundaries`) asserted something cesium
   never promises. Replace with:
   - (a) same-backing-tile neighbors are **bit-exact** along shared edges;
   - (b) residual edge mismatch is **bounded by the coarser backing level's
     geometric error**;
   - (c) skirt depth ≥ that bound (so seams cannot show as holes).
   Remove the TEMP DIAGNOSTIC block from SurfaceModelTest.cc when retiring
   the old assertion.

## Deletions this design enables

- Pointwise vertex loop + dyadic world-coordinate math in `samplePatch`.
- Memo boundary criticality (memo may even go entirely if point-query perf
  allows).
- The bit-exact stitch replica in the continuity test.
- (Already deleted) the feathering blend — cesium has no feathering anywhere;
  it distorts `heightAt` truthfulness.

## Deferred / open items

- `kMaxTiles = 128` capacity review: is the soft cap growing often, and why
  was demand exceeding it? (User explicitly deferred until after cutover.)
- Skirt sizing implementation (`5 × level geometric error`) and its test.
- Whether coarse patches should ever show finer resident data (cesium: no —
  finer detail comes from splitting the tile, which SurfaceModel LOD already
  does; adopting that is part of this design).
