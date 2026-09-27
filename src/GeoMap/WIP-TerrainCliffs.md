# WIP: transient terrain "cliffs" at patch edges

Branch: `geomap-cliff-monitor` (based on `upstream/master`). Delete this file before the PR.

## The problem

In 3D, GeoMap briefly draws near-vertical walls along patch (tile) edges that clean
themselves up within ~100 ms. Goal: this should never happen.

## Root cause

`PatchSampler` samples a patch's **interior** vertices from the patch's own best
elevation tile, but its **boundary** vertices canonically (east/south cell first, so
neighbors agree bit-for-bit and never crack). When a patch and its neighbor are backed
by elevation data of different quality, the whole difference drops across the single
grid cell just inside the edge: a cliff. It clears when the missing tile arrives and
both sides re-mesh.

Two cases, told apart by the log's `own zoom`:

| Case | Log | Cause |
|---|---|---|
| No data | `own zoom -1` | Patch has no elevation data at all (no tile, no ancestor): interior renders 0 m, edge copies the neighbor's real height. Wall = full terrain height. |
| Fine vs coarse | e.g. `own zoom 10 boundary zoom 15` | One side has its fine tile, the other still uses a coarse ancestor estimate. Step = coarse estimate error. |

Why "no data" happened at all: patches are only added at their final LOD, so coarse
ancestor tiles over the camera area were never requested.

## Done on this branch

1. **Cliff monitor (diagnostic)**
   - `PatchSampler::EdgeStep` (optional out-param through `HeightField::samplePatch`)
     records the largest gap between a boundary vertex resolved from a different-zoom
     tile and the patch's own view at that vertex. Same-zoom neighbors are ignored
     (edge clamping of real data, not a cliff).
   - `SurfaceModel::_samplePatch()` runs it only while the **GeoMap debug UI** setting
     (`flyViewSettings.geoMapDebugUI`) is on, read directly from `SettingsManager`.
   - Logs as `qCWarning` in `GeoMap.SurfaceModel.Cliffs` (visible with no category
     enabled), threshold `kCliffLogThreshold` = 5 m:
     - `cliff appeared: patch z/x/y edge E step N m at <coord> own zoom A boundary zoom B`
     - `cliff cleared: patch z/x/y after N ms` (the missing tile arrived)
     - `cliff removed with patch z/x/y after N ms` (patch replaced by LOD churn first)
   - Tests: `HeightFieldTest::_edgeStep*`.
2. **z10 anchor tiles (fix for the "no data" case)**
   - `TerrariumTileFetcher::requestTile()` also requests the `kAnchorZoom` (10) ancestor
     of every finer tile, anchor first.
   - A finer tile delivered before its anchor is held (`_heldForAnchor`) and inserted
     right after the anchor, so fine data never lands next to a region with no data.
   - If the anchor fetch fails, held tiles are released anyway (debug log).
   - Anchors are pinned against eviction by `SurfaceModel::_repinAncestors()` (they are
     ancestors of resident patches).
   - Tests: `TerrariumTileFetcherTest::_fineTileHeldUntilAnchorArrives`; three field
     tests updated for the extra anchor insert. `_patchSamplingMatchesFieldSampling` now
     compares only vertices owned by the tile (south row / east column resolve into the
     anchor).

## Manual testing still to do

With the Fly View GeoMap debug UI setting on, watch the console for
`GeoMap.SurfaceModel.Cliffs` warnings.

- [ ] Rerun the startup repro (camera starts near 47.633, -122.088): the `own zoom -1`
      burst at ~5 s (column `17/21088/*`, Lake Sammamish) should be gone.
- [ ] Remaining `own zoom -1`: expected only at startup before the first anchor arrives,
      or where the view crosses into a z10 tile that has not loaded yet. Confirm rare.
- [ ] Record the new dominant case (`own zoom 10 boundary zoom 15`-style): step sizes and
      durations, especially over mountains (tall relief makes steps large).
- [ ] Stress: fast zoom-in, pan into unvisited mountainous terrain, jump/recenter far away,
      throttled network (Network Link Conditioner) and a cleared tile cache.
- [ ] Visually confirm cliffs seen on screen line up with logged ones. A visible cliff
      with no log line means a different mechanism (candidates below).
- [ ] Check startup still looks right: before any anchor lands everything is flat 0 m,
      then coarse terrain, then fine.

## Coding still to do

- [ ] **Fine-vs-coarse steps.** Make "never" true: spread each edge's source mismatch
      smoothly over the patch interior instead of one cell (e.g. inverse-distance
      weighting of boundary deltas; Coons patches overshoot on single-vertex corner
      spikes). Update the cliff metric to measure the *rendered* one-cell excess step.
- [ ] **Anchor tile boundaries.** A patch just across a z10 boundary whose anchor has not
      loaded still renders 0 m. Options: prefetch the neighbouring anchors of the visible
      region, or also hold fine tiles whose *neighboring* anchors are missing.
- [ ] **Corner labelling.** When the worst vertex is a corner, the edge letter picks N/S
      first (`'N'` may mean the NW corner). Cosmetic.
- [ ] **SurfaceAnalysis.** The Analyze button's seam check compares neighbors' boundary
      heights (identical by design), so it never sees in-patch steps. Add the EdgeStep
      metric to its report.
- [ ] **Other candidate mechanisms** if unexplained cliffs remain: parent/child overlap
      during budgeted splits/merges (`kMaxPatchAddsPerUpdate` = 2), LOD deltas beyond the
      stitchable limit (skirts only), three-LOD corners.
- [ ] Decide whether the cliff monitor stays long term; if so, document it in the user
      guide next to the other GeoMap debug UI tools.
- [ ] Before PR: delete this file, squash to one commit, `pre-commit run --files <changed>`.
      Known pre-existing lint noise: `typos` flags "LOD" in the README; some
      clang-format violations on untouched lines (constructor initializer lists, includes).

## Test commands

```bash
cmake --build --preset don-mac-debug --parallel 24
ctest --preset don-mac-debug -I 63,81 --output-on-failure   # all GeoMap unit tests
```
