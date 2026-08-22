# GeoMap FlyView Parity — Remaining Work

Branch: `geomap-flyview-engine-switch` (issue #14901 Phase 2).
Reference for parity: [FlyViewMap.qml](../FlyView/FlyViewMap.qml) (QtLocation engine) vs
[FlyViewGeoMap.qml](../FlyView/FlyViewGeoMap.qml) (GeoMap engine).

## Done (committed in `feat(GeoMap): FlyView guided/telemetry visuals parity`)

- Home pin, vehicles, flight path, mission path/items/direction arrows, rally points
- Proximity radar (FlyViewGeoProximityRadar.qml)
- Goto indicator, fwd-flight loiter circle, orbit edit + telemetry circles, ROI (all at
  bias-corrected vehicle altitude in 3D; live orbit bindings use 1 m altitude deadband)
- PiP forces 2D camera, restores mode/distance on exit
- ObstacleDistanceOverlayMap engine-agnostic via required `mapControl`
- GeoMapMissionLabel: single-char labels render inside the indicator dot

## Done (uncommitted)

- GeoFence visuals: GeoMapGeoFenceVisuals.qml + GeoMapPolygon.qml, `fillColor` on
  GeoMapCircle.qml, wired into FlyViewGeoMap + CMake. Built, linted, FlyViewGeoUITest 8/8.
  Needs manual test (fence polygons/circles, param circular fence, breach point "B").

## Remaining parity items (in suggested order)

1. **ADSB vehicles** — [FlyViewMap.qml L239](../FlyView/FlyViewMap.qml): `MapItemView` over
   `QGroundControl.adsbVehicleManager.adsbVehicles` with `ADSBVehicleMapItem`. Needs a
   GeoMap equivalent (marker + heading + callsign label; consider altitude rendering in 3D).
2. **Camera trigger points** — [FlyViewMap.qml L305](../FlyView/FlyViewMap.qml):
   `CameraTriggerIndicator` per trigger point on the active vehicle.
3. **Per-vehicle plan items** — [FlyViewMap.qml L254](../FlyView/FlyViewMap.qml): QtLocation
   shows plan items for *every* connected vehicle (Repeater + PlanMapItems); GeoMap engine
   currently renders only the active vehicle's mission via `_missionController`.
4. **MapScale** — [FlyViewMap.qml L642](../FlyView/FlyViewMap.qml): on-map scale bar.
   Only meaningful in 2D mode (scale is ill-defined under 3D tilt) — gate on camera mode.
5. **CustomMapItems** — [FlyViewMap.qml L273](../FlyView/FlyViewMap.qml): custom-build hook
   that hands builds a QtLocation `Map`. Not directly portable; likely a documented
   limitation rather than a port.

## Review follow-ups (deferred, affect both engines unless noted)

- ROI initial-coordinate gap: marker coordinate only set via `onRoiCoordChanged`; if ROI is
  already active when the map component is created (engine swap), the marker shows at a
  default coordinate until the next signal.
- `globals.guidedControllerFlyView` registrations (`orbitMapCircle`,
  `fwdFlightGotoMapCircle`) are never cleared on destruction — transient dangling refs
  during engine swap. Consider `Component.onDestruction` cleanup.
- #14927: map-click drop panel leak (`mapClickDropPanelComponent.createObject` without
  `onClosed: destroy()`) — the geo adapter call site inherits the leaky pattern; the new
  geo ROI panel already has the fix.
- Proximity radar meters-per-pixel ratio only recalcs on camera changes, not vehicle
  motion — minor drift when camera static and vehicle moves in 3D.

## Workflow notes

- Build: `cmake --build --preset don-mac-debug --parallel 24` (check
  `pgrep -fl "ninja|cmake --build"` first).
- Tests: `./build/macOS-debug/Debug/QGroundControl.app/Contents/MacOS/QGroundControl
  --unittest:FlyViewGeoUITest` (also FlyViewGeoROIUITest, FlyViewROIUITest).
- Lint touched files only: `pre-commit run --files <files>` (venv active). cmake-format
  failures on src/GeoMap/CMakeLists.txt and src/FlyView/CMakeLists.txt are pre-existing
  whole-file reflows at HEAD — leave alone.
- Delete this file before the PR goes up.
