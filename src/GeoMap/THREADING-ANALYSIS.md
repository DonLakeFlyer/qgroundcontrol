# GeoMap Threading Analysis

Which parts of the GeoMap engine are worth moving off the QML thread, and which are not.
Informed by how cesium-native splits work between worker jobs and its main thread (see
CESIUM-ANALYSIS.md). Everything listed below currently runs on the QML thread.

## Context: what cesium-native parallelizes (and why we're different)

cesium-native dispatches per-tile content loading to an engine-supplied thread pool:
glTF parsing, Draco mesh decode, KTX2/image decode, terrain upsampling, raster overlay
rasterization. Its tileset traversal / LOD selection stays single-threaded on the main
thread; only the load jobs fan out, and results marshal back for the render handoff.

It needs that parallelism because each tile load costs tens of milliseconds (multi-MB
glTF + Draco + KTX2). GeoMap's per-patch work is orders of magnitude cheaper: sample a
(gridSize+1)^2 grid from an in-memory tile and emit vertex buffers. Our real CPU cost is
tile *decode*, which arrives in bursts. So the cesium lesson applies to the decode stage,
not the mesh stage.

## Worth threading (burst work, per-tile, embarrassingly parallel)

### 1. Tile PNG decode + format conversion — the clearest win

- `decodeTile()` in TerrariumTileFetcher.cc: `QImage::loadFromData` +
  `convertToFormat(Format_RGB32)`
- The same pattern in TileImageSource.cc for raster tiles

A pan/zoom burst delivers many tiles in one event-loop window, each costing low
single-digit ms on the QML thread. `QImage` is safe off-thread (unlike `QPixmap`), so:
`QtConcurrent::run` the decode, marshal the finished image back with `.then(this, ...)`.
This is the QGC analogue of cesium's image-decode worker jobs.

### 2. Terrarium grid extraction — ride along with decode

`gridFromImage` / `sampleGrid` (terrarium RGB -> float heights) is cheap alone, but it is
downstream of decode; fold it into the same worker job so the main thread receives a
ready `ElevationTilePyramid::Grid`.

### 3. PatchTextureData's `convertToFormat(Format_RGBA8888)`

Better fix than threading `setImage` itself: do the conversion in the decode worker
(deliver RGBA8888 from the start). Texture upload must stay on the QML thread anyway
(QQuick3DTextureData rule).

### 4. `_fallbackImage` canvas composition (SurfacePatchModel) — only if profiling says so

QPainter-on-QImage is thread-safe, and this bursts exactly when the frame budget is
tightest (LOD transitions). But the fallback cache may already make it rare — measure
before touching it.

## Probably not worth threading

### PatchGeometry::_rebuild

Even at 257x257 (~66k verts) it's a couple ms, and typical grids are far smaller.
Threading it requires extracting the mesh builder out of the QQuick3DObject into a pure
function over immutable inputs (key, gridSize, span, lodDeltas, resolved edge samples,
snapshot of the backing tile -> vertex/index QByteArrays), with PatchGeometry applying
the result on the QML thread. Do that refactor only if profiling shows rebuild bursts
hurting — e.g. every patch re-stitching when LOD deltas ripple.

That "pure builder + snapshot" shape is worth adopting eventually regardless (it's what
makes cesium's threading trivial: worker jobs never touch mutable tileset state), and
implicitly-shared QList makes snapshots free. But it's an enabling refactor, not a
milestone need.

### SurfacePatchModel orchestration / HeightField / pyramid bookkeeping

Hash lookups and model role updates. This is our "selection algorithm", and cesium keeps
the equivalent single-threaded on the main thread too.

### Network

Already async via the fetch path.

## Bottom line

One worker stage — "bytes -> converted QImage (+ grid for elevation tiles)" — inserted in
the two fetchers is ~90% of the achievable win for ~5% of the refactor, and it doesn't
disturb the mesh/stitching code. Qt 6 already provides the cesium AsyncSystem pattern
natively: `QtConcurrent::run` -> `.then(QtFuture::Launch::Async, ...)` for chained worker
stages -> `.then(this, ...)` to marshal back to the QML thread. No framework needed.
Measure before threading anything beyond the decode stage.
