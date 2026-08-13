# Cesium Native as a Backend: Evaluation

Should GeoMap adopt cesium-native as its terrain backend, and what would the Qt frontend
look like? Research notes from reading `~/repos/cesium-native` and `~/repos/vsgCs`.
Companion docs: CESIUM-ANALYSIS.md (terrain-continuity algorithms),
THREADING-ANALYSIS.md (what GeoMap should thread).

## Verdict

Stay the course. Keep GeoMap Qt-native and small; treat cesium-native as a reference
implementation to crib algorithms from (single-tile dyadic sampling, real-neighbor edge
resolution, skirts, screen-space-error LOD — all already adopted or planned). Revisit
only if QGC's ambition becomes streaming photogrammetry / 3D city tiles, in which case
cesium-native is the only sane path and none of the patch code survives anyway.

## What cesium-native is (and isn't)

Pure CPU-side library: 3D Tiles / quantized-mesh streaming, tileset traversal and LOD
selection, glTF/Draco/KTX2 decode, terrain upsampling, raster overlay mapping. It
contains **no GPU code at all** — no renderer, no Vulkan/GL/Metal/D3D. It ends at
"here's a `CesiumGltf::Model`"; the consuming engine owns upload and rendering.

Threading model: an `AsyncSystem` dispatches continuations to an engine-supplied
`ITaskProcessor` (thread pool). Per-tile content loading fans out to workers (parse,
decode, upsample, rasterize overlays); the traversal/selection algorithm stays
single-threaded on the main thread. Qt 6 has the same machinery natively
(`QtConcurrent::run` -> `.then(...)` chains) — see THREADING-ANALYSIS.md.

## Why not as GeoMap's backend

1. **It ships no renderer — the adapter is the hard part.** Cesium for Unreal/Unity are
   each large, multi-year plugin codebases that mostly consist of engine glue:
   model -> native mesh/texture conversion, render-resource lifecycle
   (`prepareInLoadThread` / `prepareInMainThread`), `IAssetAccessor`, `ITaskProcessor`,
   camera bridging, ECEF <-> engine transforms with float-precision handling. No
   QtQuick3D adapter exists; we'd write the first one, and QtQuick3D's scene-graph
   threading (geometry updates only at the sync point) makes the glue awkward. The
   adapter would likely dwarf the entire current GeoMap engine.

2. **Data-source mismatch.** Its terrain paths are 3D Tiles and quantized-mesh
   (layer.json). QGC's world is XYZ raster tiles plus its own elevation scheme, with
   users bringing their own provider keys. The flagship data (Cesium World Terrain,
   Google Photorealistic 3D Tiles) is gated behind Cesium ion tokens/terms — a
   problematic default for an open-source GCS. Self-hosted quantized-mesh exists but
   isn't what QGC's ecosystem serves.

3. **Model mismatch.** cesium-native is a globe: ECEF, ellipsoid math, origin-rebasing
   for precision. GeoMap is a local flat-ENU patch around the vehicle — the right scope
   for a flight display, and dramatically simpler.

4. **Dependency and CI weight.** vcpkg-managed tree (curl, draco, ktx, spdlog, glm,
   async++, s2geometry, ...) across six platforms including Android and iOS. License
   itself is fine (Apache 2.0); the build/CI tax is permanent.

## Closest frontend to read: vsgCs

`vsgCs` (github.com/timoore/vsgCs, cloned at `~/repos/vsgCs`) is a cesium-native client
for VulkanSceneGraph — by far the closest analog to a hypothetical QtQuick3D frontend:

- Same architectural shape as Qt Quick 3D: a C++ retained scene graph you attach nodes
  to, not a game editor. No UObject machinery (Unreal) or C# interop codegen (Unity).
- Small and readable: ~93% C++, MIT license, a few dozen files under `src/vsgCs/`.
- Complete for the terrain use case: whole-earth paging, layered raster overlays, a
  working `worldviewer` app.

Mapping vsgCs pieces to what a Qt integration would need:

| vsgCs piece | Qt equivalent |
|---|---|
| `vsgResourcePreparer` (`IPrepareRendererResources`: glTF -> vsg geometry/textures, load-thread vs main-thread halves) | CesiumGltf -> `QQuick3DGeometry` / `QQuick3DTextureData`, worker vs QML-thread split |
| `OpThreadTaskProcessor` (`ITaskProcessor`) | `QThreadPool` wrapper |
| `UrlAssetAccessor` (`IAssetAccessor`) | `QNetworkAccessManager` wrapper + tile cache |
| `TilesetNode` / `WorldNode` per-frame `updateView` -> attach/detach of tile nodes | Syncing selection results into the QtQuick3D scene |
| `CRS` / `WorldAnchor` (ECEF <-> scene-graph transform, origin handling) | ENU/local-origin math |

cesium-native's own `Cesium3DTilesSelection` tests include a
`SimplePrepareRendererResources` stub — the minimal skeleton of the renderer contract
without engine noise. Reading vsgCs gives a realistic size estimate for a Qt frontend;
even the thinnest one is substantial compared to all of GeoMap.

## What vsgCs puts on the GPU

From the shader interfaces (`data/shaders/csstandard.vert`, `csstandard_pbr.frag`,
`descriptor_defs.glsl`):

**Geometry (per tile primitive)**
- Vertex buffers: position, normal, color, up to 4 UV sets, per-instance `mat3x4`
  transforms for `EXT_mesh_gpu_instancing`; index buffers.
- A `displacementMap` sampler bound in the **vertex shader** — vertices can be displaced
  on the GPU from a height texture instead of baking heights into positions on the CPU.

**Textures**
- Full PBR set per primitive: diffuse, metallic-roughness, normal, AO, emissive,
  specular — KTX2-transcoded to GPU-compressed formats (no CPU re-encode on upload).
- Raster overlays as a per-tile texture array (`overlayTextures[maxOverlays]`, default 4)
  plus a `TileParams` UBO with per-overlay UV transforms/alpha — **overlay draping is
  composited in the fragment shader**, layered and blended per pixel.

**Scene-level**
- Lights UBO, shadow-map array, blue-noise texture (dithering), matrices via push
  constants.

Division of labor is the classic one: CPU workers produce final meshes and decoded
textures; the GPU does zero geometry *generation*.

## Relevance to GeoMap

Two things vsgCs does on the GPU that GeoMap currently does on the CPU:

1. **Overlay compositing.** Where `SurfacePatchModel::_fallbackImage` composes imagery
   with QPainter, vsgCs binds overlay textures + UV transforms and composites in the
   fragment shader; a parent-tile fallback is just a UV transform into the parent
   texture. This is the parent-texture-UV idea already evaluated and rejected for
   GeoMap in issue #14822 — see that issue for the rationale.

2. **Height displacement in the vertex shader.** A flat shared grid mesh + per-tile
   height texture could replace per-patch CPU rebuilds entirely. Plausible future
   direction for PatchGeometry if rebuild cost ever matters, but it moves T-junction
   stitching into the shader — exactly the complexity just solved on the CPU side.

## Frontend patterns worth noting

From reading `TilesetNode.cpp`, `vsgResourcePreparer.cpp`, and `WorldNode.cpp`:

1. **The scene graph is virtual — selection output *is* the scene.**
   `TilesetNode::traverse()` never attaches/detaches child nodes; every frame it
   iterates `_viewUpdateResult->tilesToRenderThisFrame` and dispatches the visitor into
   each tile's model. No node lifecycle churn; the render list swaps atomically.
   GeoMap's SurfacePatchModel -> QML delegates is more retained, which costs delegate
   create/destroy on LOD changes but buys QML bindability. Alternative to know about if
   delegate churn ever shows in profiles.

2. **Cross-fade LOD transitions, not just hold-until-ready.**
   `enableLodTransitionPeriod` + `lodTransitionLength = 1.0s`: tiles in `tilesFadingOut`
   keep rendering while fade < 1.0, driven by a per-tile fade uniform
   (`pbr::setFadeValue`) dithered with the blue-noise texture. GeoMap's retiring-covers
   is the same idea but binary. Upgrade path if cover swaps ever look poppy (MapLibre's
   `holdForFade` is the same family).

3. **Deferred GPU deletion queue (3-frame delay).** `DeletionQueue` in
   vsgResourcePreparer.cpp exists because in-flight frames may still reference buffers;
   flagged in-code as a "hack". QtQuick3D absorbs this hazard internally — part of what
   going lower-level would cost.

4. **Per-frame millisecond budgets for main-thread work.**
   `mainThreadLoadingTimeLimit = 5.0`, `tileCacheUnloadTimeLimit = 5.0`: main-thread
   halves of tile loads and cache eviction are amortized across frames under a time
   budget rather than done as they arrive. Most directly stealable idea: if a burst of
   tile deliveries ever hitches GeoMap, budget the apply-side work (texture sets, patch
   rebuilds) per frame instead of draining the queue.

5. **GPU compile happens on the load thread.** `readAndCompile()` runs
   `compileManager->compile()` in the worker; the main thread just merges
   (`updateViewer`). QtQuick3D can't do that (upload happens at scene-graph sync), so
   our version is "make the worker output so complete that sync-point application is
   trivial" — see THREADING-ANALYSIS.md.

6. **Frame-driven async pump.** `UpdateTileset::run()` (an `ALL_FRAMES` update op)
   calls `dispatchMainThreadTasks()` once per frame, then selection, then fades, then
   `loadTiles()`. One predictable place where all async results land, versus Qt signal
   deliveries scattered through the event loop. Worth imitating if ordering bugs appear.

7. **Lifetime management is their biggest pain.** In-code comments: "kind of gross"
   ref-from-this, observer_ptrs to the viewer, and tileset destruction is *async* — a
   `_tilesetsBeingDestroyed` counter gates final teardown until cesium-native's
   in-flight work drains. QObject ownership and signals spare GeoMap most of this; a
   real hidden cost of the cesium adapter path.
