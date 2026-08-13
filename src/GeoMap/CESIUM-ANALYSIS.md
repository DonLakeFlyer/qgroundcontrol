# Cesium Native: Terrain Continuity Analysis

Research report from reading the cesium-native C++ source (`~/repos/cesium-native`)
with respect to the GeoMap continuous-drape cliffs problem. Source references
are to files in that repo. Delete before the final PR.

## A. Filling in missing tiles — upsampling by clipping the parent mesh

**Where the decision is made:** `Cesium3DTilesSelection/src/LayerJsonTerrainLoader.cpp`,
`createTileChildrenImpl()` (~line 1071):

```cpp
// If _any_ child is available, we create _all_ children
bool sw = this->tileIsAvailableInAnyLayer(swID);
...
if (sw || se || nw || ne) {
  createChildTile(tile, children, swID, sw);
  ...
```

and in `createChildTile()`:

```cpp
if (isAvailable) {
  child.setTileID(childID);                          // real QuadtreeTileID -> fetch from network
} else {
  child.setTileID(UpsampledQuadtreeNode{childID});   // synthesize from parent
}
```

So the key rule is: **the quadtree never refines partially.** If any of the
four children has real data, all four child slots are populated — the
unavailable ones get a `CesiumGeometry::UpsampledQuadtreeNode` tile ID (a
wrapper around `QuadtreeTileID`, see
`CesiumGeometry/include/CesiumGeometry/QuadtreeTileID.h`). There is no case
where a fine tile renders next to a "hole"; the hole is *always* a
synthesized tile.

**How the upsampled geometry is built:**
`RasterOverlayUtilities::upsampleGltfForRasterOverlays()` in
`CesiumRasterOverlays/src/RasterOverlayUtilities.cpp`
(`upsamplePrimitiveForRasterOverlays`, ~line 1295). It does **not resample a
heightfield** — it geometrically clips the parent's triangle mesh at the
quadrant midlines (texture coordinate 0.5 in a projected UV attribute):

```cpp
clipTriangleAtAxisAlignedThreshold(0.5, keepAboveU, i0, i1, i2, uv0.x, uv1.x, uv2.x, clippedA);
// then clip result against the N-S boundary at v = 0.5
```

Clip vertices are either original parent vertices (`int` index) or
`InterpolatedVertex{first, second, t}` (see
`CesiumGeometry/include/CesiumGeometry/clipTriangleAtAxisAlignedThreshold.h`)
— an exact lerp *along parent triangle edges*. All vertex attributes
(position, normals, UVs) are lerped with the same `t`.

**Why this guarantees continuity with siblings:** two upsampled siblings
sharing an interior edge are both clipped against the *same* line (uv = 0.5)
of the *same* parent triangles, so their shared-edge vertices are
bitwise-identical interpolations → **exact agreement, no crack between
upsampled siblings.** The upsampled child's surface is a strict subset of the
parent's surface, so it also agrees exactly with anything that agreed with
the parent.

**Continuity with a sibling that has *real* data is NOT vertex-exact** — the
real child tile's edge heights come from finer source data and generally
differ from the parent-derived (upsampled) surface at the shared border.
Cesium does not stitch that seam; it hides it with skirts (Part B).

**Parent-first / lifetime guarantees:**
`Cesium3DTilesSelection/src/TilesetContentManager.cpp`, `loadTileContent()`
~line 1180:

```cpp
// - Any tile that is marked as upsampled tile, we will guarantee that the
// parent is always loaded. ...
// - This manager will also guarantee that the parent tile will be alive until
// the upsampled tile content returns to the main thread.
```

If the parent isn't `Done`, it loads the parent instead and retries later.
`Cesium3DTilesSelection/src/RasterOverlayUpsampler.cpp` `loadTileContent()`
asserts this: `CESIUM_ASSERT(pParent->getState() == TileLoadState::Done &&
"Parent must be loaded before upsampling")`, then runs
`upsampleGltfForRasterOverlays` in a worker thread capturing `parentModel` by
reference.

## B. Skirts — the primary (and only) crack-hiding mechanism between different data

**Real terrain tiles**
(`CesiumQuantizedMeshTerrain/src/QuantizedMeshLoader.cpp`):

```cpp
double calculateSkirtHeight(ellipsoid, rectangle) noexcept {
  const double levelMaximumGeometricError =
      calcQuadtreeMaxGeometricError(ellipsoid) * rectangle.computeWidth();
  return levelMaximumGeometricError * 5.0;   // 5x the level's max geometric error
}
```

Skirt vertices duplicate each border-edge vertex, dropped by `skirtHeight`
along the ellipsoid normal (`heightMeters = lerp(min,max,ratio) -
skirtHeight`), and are additionally **nudged slightly outward** to avoid
coincident-wall z-fighting:

```cpp
const double longitudeOffset = (east - west) * 0.0001;
const double latitudeOffset  = (north - south) * 0.0001;
```

The quantized-mesh format itself provides explicit per-edge vertex index
lists (`westEdgeIndicesBuffer`, `southEdgeIndicesBuffer`, etc., parsed ~line
270), which `addSkirt`/`addSkirts` sort along the edge and triangulate into a
vertical curtain.

**Skirt metadata survives upsampling:**
`CesiumGltfContent/include/CesiumGltfContent/SkirtMeshMetadata.h` records
`noSkirtIndicesBegin/Count`, `noSkirtVerticesBegin/Count`, `meshCenter`, and
the four per-side skirt heights in glTF `extras`. When upsampling
(`RasterOverlayUtilities.cpp` ~line 1268):

- The parent's skirt triangles are **excluded** from clipping
  (`indicesBegin = parentSkirtMeshMetadata->noSkirtIndicesBegin`) — only real
  surface is upsampled.
- `addEdge()` (~line 1575) collects clip vertices landing on the child's four
  borders into `EdgeIndices{west, south, east, north}`.
- `addSkirts()` (~line 1707) rebuilds fresh skirts on the child. Sides
  inherited from the parent's outer boundary keep the parent's skirt height;
  the new interior edges get `shortestSkirtHeight * 0.5`:

```cpp
if (isWestChild(childID)) {
  currentSkirt.skirtWestHeight = parentSkirt.skirtWestHeight;
} else {
  currentSkirt.skirtWestHeight = shortestSkirtHeight * 0.5;
}
```

**There is no vertex-level cross-tile stitching anywhere.** A repo-wide grep
for `stitch`/`feather` returns nothing. Skirts are the sole mechanism for
hiding cross-LOD and real-vs-upsampled seams.

## C. Blending/feathering vs. exact agreement

Cesium-native **never blends or feathers heights near borders.** The
continuity model is three-tiered:

1. **Same-level real neighbors:** the quantized-mesh *data format* guarantees
   adjacent tiles at the same level share identical edge vertices (that's
   what the explicit west/south/east/north edge-index lists are for) — exact
   agreement is a property of the source data, not runtime code.
2. **Upsampled tiles vs. their source:** exact by construction — clipping
   with `InterpolatedVertex` lerps means the child surface lies exactly on
   the parent surface (Part A).
3. **Everything else (cross-LOD edges, real child vs. upsampled sibling):**
   *no agreement attempted*; cracks are hidden by skirts sized to 5× the
   level's maximum geometric error, which bounds the largest possible height
   discrepancy at the seam.

## D. Eviction vs. rendering — a visible tile cannot be evicted

**Cache limit:** `TilesetOptions::maximumCachedBytes = 512MB`
(`Cesium3DTilesSelection/include/Cesium3DTilesSelection/TilesetOptions.h`
~line 162). `Tileset::_unloadCachedTiles` →
`TilesetContentManager::unloadCachedBytes()` walks an explicit LRU:

`Cesium3DTilesSelection/include/Cesium3DTilesSelection/TileUnloadQueue.h`:

```cpp
/** @brief LRU list of tiles eligible for content eviction.
 *  Head is least-recently-used, tail is most-recently-used. */
class TileUnloadQueue { ... Tile::UnusedLinkedList _queue; };
```

**Protection is reference counting, not a "visible" flag.** In
`Cesium3DTilesSelection/src/Tile.cpp` `addReference()`/`releaseReference()`
(~lines 386–470): when a tile's refcount goes 0→1 it calls
`markTileIneligibleForContentUnloading(tile)` (removes it from the LRU queue)
and also **add-refs its parent** ("Parent of referenced tile"), so the whole
ancestor chain of anything referenced is pinned. When the last reference
drops, `markTileEligibleForContentUnloading` re-inserts at the LRU tail.

The render list holds those references:
`ViewUpdateResult::tilesToRenderThisFrame` is a
`std::vector<Tile::ConstPointer>` (intrusive pointers) — see
`Cesium3DTilesSelection/src/Tileset.cpp` ~line 291. So a tile selected for
rendering (in any view group) simply is not in the unload queue at all.

`unloadTileContent()` (`TilesetContentManager.cpp` ~line 1365) adds further
guards even for queued tiles:

- `TileLoadState::ContentLoading` → `Keep`.
- **A parent whose child is currently being upsampled from it → `Keep`**
  (sets `TileLoadState::Unloading` and defers), because the worker thread
  holds the parent's model by reference:

```cpp
for (const Tile& child : tile.getChildren()) {
  if (child.getState() == TileLoadState::ContentLoading &&
      std::holds_alternative<CesiumGeometry::UpsampledQuadtreeNode>(child.getTileID())) {
    tile.setState(TileLoadState::Unloading);
    return UnloadTileContentResult::Keep;
```

Eviction is also time-budgeted (`timeBudgetMilliseconds`) to spread cost
across frames.

## E. LOD selection — no neighbor-level constraint; hole-freeness via "kicking"

There is **no rule limiting adjacent rendered tiles to within N levels of
each other**. Neighbor LOD deltas are unbounded and skirts absorb the seams.
What the traversal *does* guarantee, in
`Cesium3DTilesSelection/src/TilesetSelection.cpp` (`visitTile`, ~line 820),
is hole-free refinement *within a subtree*:

```cpp
// Descendants will be kicked if any are not ready to render yet and none
// were rendered last frame.
bool kickDueToNonReadyDescendant = !traversalDetails.allAreRenderable &&
                                   !traversalDetails.anyWereRenderedLastFrame;
...
bool willKick = wantToKick && (traversalDetails.notYetRenderableCount >
                                   options.loadingDescendantLimit ||
                               tile.isRenderable());
if (willKick) {
  // Kick all descendants out of the render list and render this tile instead.
  // Continue to load them though!
  queuedForLoad = kickDescendantsAndRenderTile(...);
```

I.e., if you want to refine a tile but *any* needed descendant isn't
renderable yet, all already-rendered descendants get "kicked" out of the
render list (`TileSelectionState::kick()`, `TileSelectionState.h` ~line 127)
and the coarser parent is rendered instead — an **all-or-nothing swap per
parent**. Combined with `mustContinueRefiningToDeeperTiles` (which keeps
rendering fine tiles from last frame while a newly-desired coarser tile
loads, avoiding detail pop-out), the visible set never has holes and never
partially refines a quadrant. `loadingDescendantLimit` (default 20) bounds
how long a coarse tile waits before force-kicking.

## Summary — how they avoid cliffs between real and coarse/upsampled neighbors

1. **Never render a raw coarse ancestor next to a fine tile at the leaf
   boundary of loaded data.** The gap is filled by a same-level *upsampled*
   tile clipped from the parent's mesh. Sibling upsampled tiles agree exactly
   (same clip line, same interpolants); an upsampled tile lies exactly on its
   parent's surface.
2. **Same-level real tiles agree exactly by data contract** (quantized-mesh
   shared edge vertices) — continuity is baked into the tile format, not
   computed at runtime.
3. **Every remaining seam (cross-LOD, real-vs-upsampled) is hidden by
   skirts**, sized `5 × levelMaximumGeometricError` and inherited/halved
   through upsampling — no height blending, feathering, or runtime edge
   stitching exists anywhere in the codebase.
4. **Selection guarantees all-or-nothing refinement** ("kick" mechanism): a
   parent is swapped for its four children only when all are renderable, so
   there is never a partially-refined quadrant.
5. **Eviction can't touch what's visible**: rendered tiles are held by
   intrusive pointers, which removes them (and their ancestor chain) from the
   LRU unload queue; parents feeding an in-flight upsample are also pinned.

## Transferable ideas for GeoMap

- (a) Make the "missing data" fallback a *same-level patch sampled exactly
  from the ancestor's mesh/interpolant* — bilinear on the ancestor grid at
  the patch's vertex locations makes shared edges of two fallback siblings
  identical, and the fallback lies exactly on the ancestor surface.
- (b) Accept that a real-data patch vs. ancestor-derived neighbor won't
  match, and size skirts proportionally to the *coarser* level's max height
  error rather than a fixed % of span.
- (c) Pin any ancestor tile that is currently the sampling source for a
  resident fallback patch so LRU eviction can't pull data out from under a
  rendered border.
